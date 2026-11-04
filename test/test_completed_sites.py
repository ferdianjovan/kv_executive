"""Source-time completed-site association and automatic transition admission."""

from copy import deepcopy
import math
import threading
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, Pose, Transform, TransformStamped, Vector3
from kv_executive.completed_sites import COMPLETED_SITE_DEFAULTS, CompletedSitesMixin
from kv_executive.mission_executive import MissionExecutive
from kv_executive.mission_geometry import quaternion_from_yaw, yaw_from_quaternion
from kv_executive.msg import InspectionReport
from kv_perception.msg import FireDetection, SmokeDetection
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import Header
from tf2_ros import TransformException
from unique_identifier_msgs.msg import UUID


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class Harness(CompletedSitesMixin):
    request_mission_state = MissionExecutive.request_mission_state

    def __init__(self):
        self.config = {
            **COMPLETED_SITE_DEFAULTS, 'local_frame': 'px4_odom',
            'robot_base_frame': 'kv_drone/base_link', 'telemetry_timeout': 2.0,
            'verify_observation_timeout': 2.0, 'verify_future_tolerance': 0.25,
            'fire_exclusion_radius': 15.0,
            'verify_planning_frame': 'px4_odom',
        }
        self.seconds = 20.0
        self._data_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._stop = threading.Event()
        self._finalizing = self._recovering = False
        self._goal_handle = SimpleNamespace(
            is_active=True, is_cancel_requested=False, goal_id=UUID(uuid=[1] * 16))
        self._parent_state = 'Exploration'
        self._requested_mission_state = None
        self._requested_verify_automatic = False
        self._initialize_completed_sites()
        self.smoke = self.fire = None
        self._verify = SimpleNamespace(
            fresh_smoke=lambda: self.smoke, fresh_fire=lambda: self.fire)
        self.camera_transform = Transform(rotation=quaternion_from_yaw(0.0))
        self.tf_calls = []
        self._tf_buffer = SimpleNamespace(lookup_transform=self.lookup)
        self.odometry(19.9)
        self.odometry(20.1)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=round(self.seconds * 1e9), to_msg=lambda: stamp(self.seconds)))

    def lookup(self, target, source, acquired, timeout):
        self.tf_calls.append((target, source, acquired.nanoseconds, timeout.nanoseconds))
        return TransformStamped(transform=deepcopy(self.camera_transform))

    def odometry(self, seconds, x=0.0, y=0.0, z=5.0, yaw=0.0):
        message = Odometry(header=Header(frame_id='px4_odom', stamp=stamp(seconds)))
        message.pose.pose = Pose(
            position=Point(x=float(x), y=float(y), z=float(z)),
            orientation=quaternion_from_yaw(yaw))
        with self._data_lock:
            self._completed_odometry_locked(message)

    def report(self, point=(30.0, 0.0, 5.0), fire=True, uncertainty=0.2, site_id=1):
        report = InspectionReport(
            site_id=site_id, status='fire_found' if fire else 'fire_not_found',
            fire_found=fire, track_id=1 if fire else 0)
        prefix = 'fire' if fire else 'smoke'
        setattr(report, prefix + '_location_valid', True)
        setattr(report, prefix + '_local_position', PointStamped(
            header=Header(frame_id='px4_odom', stamp=stamp(self.seconds)),
            point=Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))))
        setattr(report, prefix + '_uncertainty_m', float(uncertainty))
        # Simulate the actual camera image used to create this report. Target
        # coordinates are merely optional metadata; the completion API receives
        # the source image explicitly, as the real controller does.
        body = self._completed_source_pose(round(self.seconds * 1e9))
        if body is None:
            self._reported_message = None
        else:
            body_yaw = yaw_from_quaternion(body.orientation)
            camera = self.camera_transform
            camera_x = (body.position.x + camera.translation.x * math.cos(body_yaw)
                        - camera.translation.y * math.sin(body_yaw))
            camera_y = (body.position.y + camera.translation.x * math.sin(body_yaw)
                        + camera.translation.y * math.cos(body_yaw))
            yaw = math.atan2(point[1] - camera_y, point[0] - camera_x) - (
                body_yaw + yaw_from_quaternion(camera.rotation))
            self._reported_message = self.detection(prefix, yaw=yaw)
            if getattr(self, prefix) is None:
                setattr(self, prefix, deepcopy(self._reported_message))
        self._latest_report = report
        return report

    def _verify_complete_site(self, report, message=None, source=None):
        if report is getattr(self, '_latest_report', None) and message is None:
            message = self._reported_message
        return super()._verify_complete_site(report, message, source)

    def detection(self, source='fire', yaw=0.0, vertical=0.0, **overrides):
        fields = {
            'header': Header(frame_id='thermal_camera', stamp=stamp(self.seconds)),
            'detected': True, 'yaw_correction_rad': float(yaw),
            'vertical_correction_rad': float(vertical),
        }
        if source == 'smoke':
            fields.update(status=SmokeDetection.CONFIRMED, background_ready=True)
            fields.update(overrides)
            return SmokeDetection(**fields)
        fields.update(status=FireDetection.CONFIRMED, source_count=1, track_id=1)
        fields.update(overrides)
        return FireDetection(**fields)


@pytest.fixture
def executive():
    return Harness()


def test_completed_bearing_matches_track_reset_but_translation_reopens_inspection(executive):
    assert executive._verify_complete_site(executive.report())
    assert executive._verify_is_completed_detection(executive.detection(track_id=99), 'fire')
    executive.seconds = 21.0
    executive.odometry(20.9, y=5.0)
    executive.odometry(21.1, y=5.0)
    assert not executive._verify_is_completed_detection(
        executive.detection(track_id=100, yaw=math.atan2(-5.0, 30.0)), 'fire')
    assert all(call[-1] == 0 for call in executive.tf_calls)


@pytest.mark.parametrize('distance', [10.0, 30.0])
def test_distinct_source_is_not_hidden_by_large_exclusion_region(executive, distance):
    executive._verify_complete_site(executive.report(point=(distance, 0.0, 5.0)))
    assert not executive._verify_is_completed_detection(
        executive.detection(yaw=math.radians(40.0)), 'fire')
    assert not executive._verify_is_completed_detection(
        executive.detection(vertical=math.radians(40.0)), 'fire')


def test_smoke_above_completed_fire_matches_its_column_but_distinct_azimuth_does_not(executive):
    executive._verify_complete_site(executive.report())
    assert executive._verify_is_completed_detection(
        executive.detection('smoke', vertical=math.radians(40.0)), 'smoke')
    assert not executive._verify_is_completed_detection(
        executive.detection('smoke', yaw=math.radians(40.0), vertical=math.radians(40.0)),
        'smoke')
    assert not executive._verify_is_completed_detection(
        executive.detection('smoke', vertical=math.radians(-20.0)), 'smoke')


def test_selected_completed_fire_matches_with_multiple_blobs_but_new_selected_ray_is_eligible(
        executive):
    executive._verify_complete_site(executive.report())
    assert executive._verify_is_completed_detection(
        executive.detection(source_count=2, track_id=99), 'fire')
    assert not executive._verify_is_completed_detection(
        executive.detection(source_count=2, track_id=99, yaw=math.radians(40.0)), 'fire')


def test_association_interpolates_acquisition_pose_instead_of_latest_pose(executive):
    executive._inspection_odometry_history.clear()
    executive.odometry(19.5, yaw=0.0)
    executive.odometry(20.5, x=2.0, yaw=math.pi / 2.0)
    executive._verify_complete_site(executive.report(point=(20.0, 20.0, 5.0)))
    correction = math.atan2(20.0, 19.0) - math.pi / 4.0
    assert executive._verify_is_completed_detection(executive.detection(yaw=correction), 'fire')
    assert executive.tf_calls[-1][2] == 20_000_000_000


def test_camera_body_rotation_and_translation_are_applied(executive):
    executive.camera_transform = Transform(
        translation=Vector3(x=2.0), rotation=quaternion_from_yaw(math.pi / 2.0))
    executive._verify_complete_site(executive.report(point=(2.0, 30.0, 5.0)))
    assert executive._verify_is_completed_detection(executive.detection(), 'fire')


@pytest.mark.parametrize('condition', ['missing_tf', 'missing_pose', 'future_pose', 'wide_gap'])
def test_unknown_association_permits_new_inspection(executive, condition):
    executive._verify_complete_site(executive.report())
    if condition == 'missing_tf':
        def unavailable(*_args, **_kwargs):
            raise TransformException('not available now')
        executive._tf_buffer.lookup_transform = unavailable
    elif condition == 'missing_pose':
        executive._inspection_odometry_history.clear()
    elif condition == 'future_pose':
        executive.seconds = 20.2  # No extrapolation beyond the last odometry sample.
    else:
        executive._inspection_odometry_history.clear()
        executive.odometry(17.0)
        executive.odometry(23.0)
    assert not executive._verify_is_completed_detection(executive.detection(), 'fire')


@pytest.mark.parametrize('overrides', [
    {'detected': False}, {'status': FireDetection.CONFIRMING},
    {'yaw_correction_rad': math.nan},
    {'header': Header(frame_id='thermal_camera', stamp=stamp(17.9))},
])
def test_incomplete_or_unassociated_source_is_never_suppressed(executive, overrides):
    executive._verify_complete_site(executive.report())
    assert not executive._verify_is_completed_detection(executive.detection(**overrides), 'fire')


def test_automatic_admission_requires_every_visible_source_to_match(executive):
    executive._verify_complete_site(executive.report())
    executive.fire = None
    assert not executive._verify_request_is_completed()
    executive.fire = executive.detection()
    assert executive._verify_request_is_completed()
    assert not executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))
    executive.smoke = executive.detection('smoke', yaw=math.radians(40.0))
    assert not executive._verify_request_is_completed()
    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def test_completed_fire_and_high_plume_keep_exploring_until_a_new_selected_fire_ray(executive):
    executive._verify_complete_site(executive.report())
    executive.smoke = executive.detection('smoke', vertical=math.radians(40.0))
    executive.fire = executive.detection(source_count=2)
    current_goal = UUID(uuid=[1] * 16)

    assert not executive.request_mission_state('Verify', goal_id=current_goal)
    assert executive._parent_state == 'Exploration'
    assert executive._requested_mission_state is None

    executive.fire = executive.detection(source_count=2, yaw=math.radians(40.0))
    assert executive.request_mission_state('Verify', goal_id=current_goal)
    assert executive._requested_mission_state == 'Verify'
    assert executive._requested_verify_automatic


def test_manual_request_and_abort_remain_available_for_completed_source(executive):
    executive._verify_complete_site(executive.report())
    executive.fire = executive.detection()
    assert executive.request_mission_state('Verify')
    assert not executive._requested_verify_automatic
    assert executive.request_mission_state('Abort', goal_id=UUID(uuid=[1] * 16))


@pytest.mark.parametrize('condition', ['old_goal', 'canceling', 'inactive'])
def test_unmatched_source_preserves_goal_and_cancel_fences(executive, condition):
    goal_id = UUID(uuid=[2 if condition == 'old_goal' else 1] * 16)
    if condition == 'canceling':
        executive._goal_handle.is_cancel_requested = True
    if condition == 'inactive':
        executive._goal_handle.is_active = False
    assert not executive.request_mission_state('Verify', goal_id=goal_id)


def _move_observer(executive, *, x=0.0, y=0.0, z=5.0, yaw=0.0, seconds=21.0):
    executive.seconds = seconds
    executive.odometry(seconds - 0.1, x=x, y=y, z=z, yaw=yaw)
    executive.odometry(seconds + 0.1, x=x, y=y, z=z, yaw=yaw)


def test_blocked_viewpoint_cannot_repeat_after_wall_time_without_translation(executive):
    executive.fire = executive.detection()
    assert executive._verify_defer_viewpoint()
    current_goal = UUID(uuid=[1] * 16)

    assert executive._verify_request_is_deferred(current_goal)
    assert not executive.request_mission_state('Verify', goal_id=current_goal)
    _move_observer(executive, seconds=200.0)
    executive.fire = executive.detection(track_id=100)

    assert executive._verify_request_is_deferred(current_goal)
    assert not executive.request_mission_state('Verify', goal_id=current_goal)
    assert executive._requested_mission_state is None
    assert not executive._completed_sites


def test_yaw_sweep_keeps_same_world_source_deferred_but_new_selected_azimuth_opens(executive):
    executive.fire = executive.detection()
    assert executive._verify_defer_viewpoint()
    yaw = math.radians(40.0)
    _move_observer(executive, yaw=yaw)
    executive.fire = executive.detection(yaw=-yaw)
    current_goal = UUID(uuid=[1] * 16)

    assert not executive.request_mission_state('Verify', goal_id=current_goal)
    executive.fire = executive.detection(yaw=-yaw + math.radians(30.0))

    assert executive.request_mission_state('Verify', goal_id=current_goal)
    assert executive._requested_verify_automatic


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_real_translation_reopens_blocked_source_without_completing_it(executive, source):
    setattr(executive, source, executive.detection(source))
    assert executive._verify_defer_viewpoint()
    current_goal = UUID(uuid=[1] * 16)
    _move_observer(executive, x=1.9)
    setattr(executive, source, executive.detection(source))
    assert not executive.request_mission_state('Verify', goal_id=current_goal)

    _move_observer(executive, x=2.0, seconds=22.0)
    setattr(executive, source, executive.detection(source))
    assert executive.request_mission_state('Verify', goal_id=current_goal)
    assert not executive._completed_sites
    executive._requested_mission_state = None
    _move_observer(executive, seconds=23.0)
    setattr(executive, source, executive.detection(source))
    assert executive.request_mission_state('Verify', goal_id=current_goal)


def test_smoke_and_fire_share_blocked_horizontal_view_but_new_source_remains_eligible(executive):
    executive.smoke = executive.detection('smoke', vertical=math.radians(45.0))
    assert executive._verify_defer_viewpoint()
    current_goal = UUID(uuid=[1] * 16)
    executive.smoke = None
    executive.fire = executive.detection(vertical=math.radians(-30.0))

    assert not executive.request_mission_state('Verify', goal_id=current_goal)
    executive.smoke = executive.detection('smoke', yaw=math.radians(30.0))

    assert executive.request_mission_state('Verify', goal_id=current_goal)


def test_deferred_view_does_not_block_manual_override_or_later_mission_goal(executive):
    executive.fire = executive.detection()
    assert executive._verify_defer_viewpoint()
    assert executive.request_mission_state('Verify')
    assert not executive._requested_verify_automatic
    executive._requested_mission_state = None
    executive._goal_handle.goal_id = UUID(uuid=[2] * 16)

    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[2] * 16))
    assert executive._requested_verify_automatic


@pytest.mark.parametrize('condition', ['missing_tf', 'missing_pose', 'stale_image', 'no_source'])
def test_unknown_blocked_source_association_fails_open(executive, condition):
    executive.fire = executive.detection()
    assert executive._verify_defer_viewpoint()
    if condition == 'missing_tf':
        def unavailable(*_args, **_kwargs):
            raise TransformException('not available now')
        executive._tf_buffer.lookup_transform = unavailable
    elif condition == 'missing_pose':
        executive._inspection_odometry_history.clear()
    elif condition == 'stale_image':
        executive.fire.header.stamp = stamp(17.0)
    else:
        executive.fire = None

    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def test_without_fresh_associated_source_no_blocked_viewpoint_is_saved(executive):
    assert not executive._verify_defer_viewpoint()
    executive.fire = executive.detection()
    executive._inspection_odometry_history.clear()
    assert not executive._verify_defer_viewpoint()

    executive.odometry(19.9)
    executive.odometry(20.1)
    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def test_completed_smoke_and_blocked_fire_together_do_not_reenter_verify(executive):
    executive.fire = executive.detection()
    assert executive._verify_defer_viewpoint()
    angle = math.radians(40.0)
    executive._verify_complete_site(executive.report(point=(
        30.0 * math.cos(angle), 30.0 * math.sin(angle), 5.0)))
    executive.smoke = executive.detection('smoke', yaw=angle, vertical=math.radians(30.0))

    assert not executive._verify_request_is_completed()
    assert executive._verify_request_is_deferred(UUID(uuid=[1] * 16))
    assert not executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def test_odometry_clock_rewind_does_not_retain_blocked_view_from_other_epoch(executive):
    executive.fire = executive.detection()
    assert executive._verify_defer_viewpoint()

    _move_observer(executive, seconds=10.0)
    executive.fire = executive.detection()

    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


@pytest.mark.parametrize('automatic,new_source', [(True, False), (False, False), (True, True)])
def test_completion_clears_only_matching_pending_automatic_request(
        executive, automatic, new_source):
    executive._requested_mission_state = 'Verify'
    executive._requested_verify_automatic = automatic
    executive.fire = executive.detection(yaw=math.radians(40.0) if new_source else 0.0)
    executive._verify_complete_site(executive.report())
    assert (executive._requested_mission_state is None) == (automatic and not new_source)


def test_completion_requires_associated_image_evidence_and_history_is_bounded(executive):
    assert not executive._verify_complete_site(InspectionReport(status='fire_found'))
    invalid = executive.report(uncertainty=math.nan)
    assert executive._verify_complete_site(invalid)  # Optional target metadata is irrelevant.
    for index in range(105):
        executive._verify_complete_site(executive.report(site_id=index))
    assert len(executive._completed_sites) == 100
    assert executive._completed_sites[0][0] == 5


def test_unlocalized_visible_fire_completion_suppresses_stationary_repeat(executive):
    executive.fire = executive.detection(track_id=42)
    report = InspectionReport(status='fire_found', fire_found=True, track_id=42, site_id=1)

    assert executive._verify_complete_site(report)
    assert not report.fire_location_valid
    assert executive._verify_request_is_completed()
    assert not executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))

    _move_observer(executive, seconds=200.0)
    executive.fire = executive.detection(track_id=99)
    assert executive._verify_request_is_completed()
    assert not executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def test_unlocalized_completion_uses_world_bearing_through_a_yaw_sweep(executive):
    executive.fire = executive.detection(yaw=0.1)
    report = InspectionReport(status='fire_found', fire_found=True, site_id=1)
    assert executive._verify_complete_site(report)
    _move_observer(executive, yaw=0.6)
    executive.fire = executive.detection(yaw=-0.5)

    assert executive._verify_request_is_completed()

    executive.fire = executive.detection(yaw=-0.5 + math.radians(30.0))
    assert not executive._verify_request_is_completed()
    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


@pytest.mark.parametrize('axis', ['x', 'z'])
def test_translation_reopens_unlocalized_completion_permanently(executive, axis):
    executive.fire = executive.detection()
    assert executive._verify_complete_site(InspectionReport(status='fire_found', site_id=1))
    movement = {axis: 1.9 if axis == 'x' else 6.9}
    _move_observer(executive, **movement)
    executive.fire = executive.detection()
    assert executive._verify_request_is_completed()

    movement[axis] = 2.0 if axis == 'x' else 7.0
    _move_observer(executive, seconds=22.0, **movement)
    executive.fire = executive.detection()
    assert not executive._verify_request_is_completed()
    assert not executive._completed_sites

    _move_observer(executive, seconds=23.0)
    executive.fire = executive.detection()
    assert not executive._verify_request_is_completed()


@pytest.mark.parametrize(
    'condition', ['missing_tf', 'missing_pose', 'invalid_pose', 'stale_image'])
def test_unassociated_unlocalized_completion_cannot_suppress_inspection(executive, condition):
    executive.fire = executive.detection()
    if condition == 'missing_tf':
        def unavailable(*_args, **_kwargs):
            raise TransformException('missing camera transform')
        executive._tf_buffer.lookup_transform = unavailable
    elif condition in {'missing_pose', 'invalid_pose'}:
        executive._inspection_odometry_history.clear()
        if condition == 'invalid_pose':
            executive.odometry(19.9, x=math.nan)
            executive.odometry(20.1, x=math.nan)
    else:
        executive.fire.header.stamp = stamp(17.0)

    assert not executive._verify_complete_site(InspectionReport(status='fire_found', site_id=1))
    assert not executive._completed_sites
    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def test_report_source_image_keeps_a_new_selected_fire_bearing_eligible(executive):
    reported = executive.detection(yaw=0.0, track_id=42)
    report = InspectionReport(status='fire_found', site_id=1, track_id=42)
    executive.fire = executive.detection(yaw=math.radians(40.0), track_id=43)
    executive._requested_mission_state = 'Verify'
    executive._requested_verify_automatic = True

    assert not executive._verify_complete_site(report)  # Current image is another track.
    assert executive._verify_complete_site(report, message=reported, source='fire')
    assert not executive._verify_request_is_completed()
    assert executive._requested_mission_state == 'Verify'


def test_localized_fire_metadata_never_changes_completion_association(executive):
    executive.fire = executive.detection()
    image = deepcopy(executive.fire)
    first = executive.report(point=(30.0, 0.0, 5.0), uncertainty=0.2)
    second = executive.report(point=(-1000.0, 2000.0, 999.0), uncertainty=math.nan)

    assert executive._verify_complete_site(first, message=image)
    first_evidence = executive._completed_sites[-1][1:]
    assert executive._verify_complete_site(second, message=image)
    assert executive._completed_sites[-1][1:] == first_evidence


def test_unlocalized_smoke_negative_cannot_suppress_a_newly_confirmed_fire(executive):
    executive.smoke = executive.detection('smoke')
    assert executive._verify_complete_site(InspectionReport(status='fire_not_found', site_id=1))
    assert executive._verify_request_is_completed()
    executive.fire = executive.detection()

    assert not executive._verify_request_is_completed()
    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def test_clock_rewind_clears_completed_bearing_evidence(executive):
    executive.fire = executive.detection()
    assert executive._verify_complete_site(InspectionReport(status='fire_found', site_id=1))

    _move_observer(executive, seconds=10.0)
    executive.fire = executive.detection()

    assert not executive._completed_sites
    assert executive.request_mission_state('Verify', goal_id=UUID(uuid=[1] * 16))


def _measured_report(executive, *, center=(30.0, 0.0, 5.0), fire=True, site_id=1):
    report = executive.report(point=center, fire=fire, uncertainty=0.0, site_id=site_id)
    report.smoke_wall_confirmed = report.smoke_location_valid = True
    report.smoke_planning_position = PointStamped(
        header=Header(frame_id='px4_odom', stamp=stamp(executive.seconds)),
        point=Point(**dict(zip('xyz', center))))
    report.smoke_uncertainty_m = 0.0
    report.observer_planning_pose.header = Header(
        frame_id='px4_odom', stamp=stamp(executive.seconds))
    report.observer_planning_pose.pose.position = Point(x=0.0, y=0.0, z=5.0)
    report.observer_planning_pose.pose.orientation = quaternion_from_yaw(0.0)
    executive.smoke = executive.detection('smoke')
    executive.fire = executive.detection() if fire else None
    return report


def _next_cluster_frame(executive, *, source='fire', yaw=0.0, vertical=0.0,
                        track_id=1, point=None):
    _move_observer(executive, seconds=executive.seconds + 0.1)
    message = executive.detection(source, yaw=yaw, vertical=vertical,
                                  **({'track_id': track_id} if source == 'fire' else {}))
    if point is not None:
        known = PointStamped(header=Header(frame_id='px4_odom', stamp=stamp(executive.seconds)),
                             point=Point(**dict(zip('xyz', point))))
        executive._verify_candidate_planning_position = lambda *_args: known
    return message


@pytest.mark.parametrize('distance', [0.0, 19.99, 20.0, 20.01, 21.0, 24.99, 25.0])
def test_navigation_buffer_does_not_prove_semantic_identity(executive, distance):
    assert executive._verify_complete_site(_measured_report(executive))
    candidate = _next_cluster_frame(executive, point=(30.0 + distance, 0.0, 5.0))
    assert executive._verify_is_completed_detection(candidate, 'fire') == (distance <= 20.0)
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')
    assert executive._completed_cluster_regions()[0]['radius'] == 25.0


def test_one_metre_edge_crossing_does_not_reopen_cluster(executive):
    executive._verify_complete_site(_measured_report(executive))
    for offset in (19.5, 20.5, 19.8, 20.8, 21.0):
        candidate = _next_cluster_frame(executive, point=(30.0 + offset, 0.0, 5.0))
        assert not executive._verify_candidate_is_eligible(candidate, 'fire')


def test_three_distinct_outside_images_are_required_and_replay_does_not_vote(executive):
    executive._verify_complete_site(_measured_report(executive))
    for index in range(3):
        candidate = _next_cluster_frame(executive, point=(56.0, 0.0, 5.0), track_id=42)
        expected = index == 2
        assert executive._verify_candidate_is_eligible(candidate, 'fire') == expected
        for _ in range(5):
            assert executive._verify_candidate_is_eligible(candidate, 'fire') == expected
        assert not executive._verify_is_completed_detection(candidate, 'fire')


def test_return_to_buffer_establishes_completed_continuity(executive):
    executive._verify_complete_site(_measured_report(executive))
    for offset, expected in ((26.0, False), (26.0, False), (19.9, False),
                             (20.5, False), (21.0, False), (24.0, False)):
        candidate = _next_cluster_frame(executive, point=(30.0 + offset, 0.0, 5.0), track_id=42)
        assert executive._verify_candidate_is_eligible(candidate, 'fire') == expected


@pytest.mark.parametrize('source', ['smoke', 'fire'])
@pytest.mark.parametrize('yaw,vertical', [
    (math.radians(50.0), 0.0), (-math.radians(50.0), -math.radians(25.0))])
def test_distinct_distant_bearing_becomes_eligible_without_ranging(
        executive, source, yaw, vertical):
    executive._verify_complete_site(_measured_report(executive, center=(15.0, 0.0, 5.0)))
    for index in range(3):
        candidate = _next_cluster_frame(executive, source=source, yaw=yaw, vertical=vertical,
                                        track_id=42)
        assert executive._verify_candidate_is_eligible(candidate, source) == (index == 2)
    assert not hasattr(executive, '_verify_candidate_planning_position')


def test_observer_inside_buffer_does_not_blacklist_every_direction(executive):
    executive._verify_complete_site(_measured_report(executive, center=(15.0, 0.0, 5.0)))
    assert executive._verify_is_completed_detection(executive.detection(), 'fire')
    for _ in range(3):
        opposite = _next_cluster_frame(executive, yaw=math.radians(70.0))
        executive._verify_candidate_is_eligible(opposite, 'fire')
    assert executive._verify_candidate_is_eligible(opposite, 'fire')


def test_completed_location_survives_translation_and_track_reset(executive):
    report = _measured_report(executive)
    assert executive._verify_complete_site(report)
    saved = deepcopy(executive._completed_cluster_regions())
    _move_observer(executive, y=12.0)
    direction = math.atan2(-12.0, 30.0)
    candidate = executive.detection(yaw=direction, track_id=999)
    assert executive._completed_candidate_decision(candidate, 'fire') == 'ambiguous'
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')
    assert executive._completed_cluster_regions() == saved


def test_delayed_report_saves_real_point_even_with_stale_passed_image(executive):
    report = _measured_report(executive)
    old = deepcopy(executive.fire)
    original_point = deepcopy(report.smoke_planning_position)
    _move_observer(executive, seconds=23.0)
    executive.fire = executive.detection(track_id=1)
    executive.smoke = executive.detection('smoke')
    assert executive._verify_complete_site(report, message=old, source='fire')
    entry = executive._completed_clusters[0]
    assert entry['wall_point'] == original_point
    assert entry['views']['fire'][0]['message'].header.stamp == stamp(23.0)
    assert entry['views']['smoke'][0]['message'].header.stamp == stamp(23.0)


def test_explicit_report_context_wins_over_old_raw_primary(executive):
    report = _measured_report(executive)
    selected = executive.detection('smoke', yaw=0.2)
    executive._verify_report_completion_context = {'smoke': selected, 'fire': executive.fire}
    executive.smoke = executive.detection('smoke', yaw=-0.8)
    assert executive._verify_complete_site(report)
    context = executive._completed_clusters[0]['views']['smoke'][0]['message']
    assert context.yaw_correction_rad == pytest.approx(0.2)


def test_different_visible_side_plume_is_not_saved_as_reported_context(executive):
    report = _measured_report(executive)
    executive.smoke = executive.detection('smoke', yaw=math.radians(60.0))
    assert executive._verify_complete_site(report)
    assert not executive._completed_clusters[0]['views']['smoke']


@pytest.mark.parametrize('kind', ['missing_tf', 'missing_pose'])
def test_missing_source_association_keeps_cluster_and_waits_for_pose(executive, kind):
    executive._verify_complete_site(_measured_report(executive))
    if kind == 'missing_tf':
        def unavailable(*_args, **_kwargs):
            raise TransformException('temporarily unavailable')
        executive._tf_buffer.lookup_transform = unavailable
    else:
        executive._inspection_odometry_history.clear()
    candidate = executive.detection()
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')
    assert executive._completed_candidate_decision(candidate, 'fire') == 'unknown'
    assert len(executive._completed_cluster_regions()) == 1


def test_negative_smoke_cluster_never_blocks_new_confirmed_fire(executive):
    executive._verify_complete_site(_measured_report(executive, fire=False))
    assert executive._verify_is_completed_detection(executive.smoke, 'smoke')
    executive.fire = executive.detection(track_id=42)
    assert executive._verify_candidate_is_eligible(executive.fire, 'fire')
    assert not executive._verify_is_completed_detection(executive.fire, 'fire')


def test_raw_observations_drive_admission_without_recursive_filtered_getters(executive):
    executive._verify_complete_site(_measured_report(executive))
    smoke, fire = executive.smoke, executive.fire
    executive._verify.raw_fresh_smoke = lambda: smoke
    executive._verify.raw_fresh_fire = lambda: fire

    def filtered():
        raise AssertionError('Completed association must not read filtered observations')

    executive._verify.fresh_smoke = executive._verify.fresh_fire = filtered
    assert executive._verify_request_is_completed()


def test_all_array_candidates_are_checked_even_with_completed_primary(executive):
    executive._verify_complete_site(_measured_report(executive))
    candidates = {'smoke': [executive.smoke], 'fire': [executive.fire]}
    executive._verify.raw_candidates = lambda source: candidates[source]
    assert executive._verify_request_is_completed()
    for index in range(3):
        other = _next_cluster_frame(executive, yaw=0.8, track_id=42)
        candidates['fire'] = [executive.detection(), other]
        candidates['smoke'] = []
        assert executive._verify_request_is_completed() == (index < 2)


def test_registry_snapshot_preserves_point_stamp_and_returns_independent_copies(executive):
    report = _measured_report(executive)
    executive._verify_complete_site(report)
    _move_observer(executive, seconds=30.0, x=10.0)
    snapshot = executive._completed_cluster_snapshot()
    assert snapshot.header.frame_id == 'px4_odom'
    assert snapshot.header.stamp == stamp(30.0)
    assert snapshot.clusters[0].center.header == report.smoke_planning_position.header
    assert snapshot.clusters[0].center.point.x == 55.0
    assert report.smoke_planning_position.point.x == 30.0
    assert snapshot.clusters[0].radius == 25.0
    snapshot.clusters[0].center.point.x = -900.0
    assert executive._completed_cluster_regions()[0]['center'].point.x == 55.0


def test_reordered_odometry_does_not_erase_completed_registry(executive):
    executive._verify_complete_site(_measured_report(executive))
    executive.odometry(19.95)
    assert executive._completed_cluster_regions()
    assert executive._inspection_odometry_history[-1][0] == 20_100_000_000


def test_true_clock_rewind_clears_registry_and_emits_empty_snapshot(executive):
    executive._verify_complete_site(_measured_report(executive))
    previous_version = executive._completed_clusters_version
    _move_observer(executive, seconds=10.0)
    assert not executive._completed_cluster_regions()
    assert not executive._completed_cluster_snapshot().clusters
    assert executive._completed_clusters_version > previous_version


def test_queued_previous_epoch_odometry_cannot_poison_reset_pose_history(executive):
    executive._verify_complete_site(_measured_report(executive))
    executive.seconds = 10.0
    executive.odometry(20.2)  # A queued sample from before the source-clock reset.
    executive.odometry(9.9)
    executive.odometry(10.1)
    assert not executive._completed_cluster_regions()
    assert executive._completed_source_pose(10_000_000_000) is not None


def test_registry_and_candidate_history_are_bounded(executive):
    for index in range(105):
        report = _measured_report(executive, center=(float(index * 50 + 30), 0.0, 5.0),
                                  site_id=index + 1)
        executive._verify_complete_site(report)
    assert len(executive._completed_cluster_regions()) == 100
    assert executive._completed_cluster_regions()[0]['site_id'] == 6
    for identity in range(140):
        candidate = _next_cluster_frame(executive, yaw=0.8, track_id=identity + 1)
        executive._verify_candidate_is_eligible(candidate, 'fire')
    assert len(executive._completed_candidate_history) == 128


def test_nearby_report_enriches_fixed_cluster_without_moving_or_expanding_it(executive):
    first = _measured_report(executive)
    executive._verify_complete_site(first)
    original = deepcopy(executive._completed_cluster_regions()[0])
    for offset in (5.0, 10.0, 15.0, 20.0):
        executive._verify_complete_site(_measured_report(
            executive, center=(30.0 + offset, 0.0, 5.0), site_id=int(offset)))
    assert executive._completed_cluster_regions() == [original]
    assert len(executive._completed_clusters[0]['views']['smoke']) <= 4
    assert [r['site_id'] for r in executive._completed_clusters[0]['reports']] == [15, 20]
    negative = _measured_report(executive, fire=False, site_id=21)
    executive._verify_complete_site(negative)
    assert executive._completed_clusters[0]['site_id'] == 1
    assert executive._completed_clusters[0]['fire_found']


def test_new_goal_does_not_erase_completed_spatial_registry(executive):
    executive._verify_complete_site(_measured_report(executive))
    executive._goal_handle.goal_id = UUID(uuid=[2] * 16)
    _move_observer(executive, x=3.0)
    executive.smoke, executive.fire = executive.detection('smoke'), executive.detection()
    assert executive._verify_request_is_completed()
    assert not executive.request_mission_state('Verify', goal_id=UUID(uuid=[2] * 16))


def test_map_cluster_is_compared_to_current_transformed_camera_ray(executive):
    report = _measured_report(executive)
    executive.config['verify_planning_frame'] = 'map'
    report.smoke_planning_position.header.frame_id = 'map'
    report.observer_planning_pose.header.frame_id = 'map'
    executive._path_frame_transform = lambda _frame: Transform(
        translation=Vector3(x=10.0), rotation=quaternion_from_yaw(0.0))
    assert executive._verify_complete_site(report)
    _move_observer(executive, y=10.0)
    candidate = executive.detection(yaw=math.atan2(-10.0, 40.0), track_id=42)
    assert executive._completed_candidate_decision(candidate, 'fire') == 'ambiguous'
    assert executive._completed_cluster_snapshot().clusters[0].center.header.frame_id == 'map'


def test_report_transform_snapshot_and_nonblocking_candidate_hook_are_used(executive):
    report = _measured_report(executive)
    executive.config['verify_planning_frame'] = 'map'
    report.smoke_planning_position.header.frame_id = 'map'
    report.observer_planning_pose.header.frame_id = 'map'
    conversion = Transform(translation=Vector3(x=10.0), rotation=quaternion_from_yaw(0.0))

    def blocking_lookup(_frame):
        raise AssertionError('Candidate association must not use the blocking planning lookup')

    executive._path_frame_transform = blocking_lookup
    executive._completed_path_frame_transform = blocking_lookup
    executive._verify_report_completion_context = {
        'smoke': executive.smoke, 'fire': executive.fire, 'planning_to_local': conversion}
    assert executive._verify_complete_site(report)
    assert executive._completed_clusters[0]['views']['smoke']
    executive._verify_report_completion_context = None
    executive._completed_path_frame_transform = lambda _frame: conversion
    assert executive._verify_is_completed_detection(executive.fire, 'fire')


@pytest.mark.parametrize('invalid', ['nan', 'frame', 'zero_stamp', 'future', 'uncertainty'])
def test_invalid_retained_point_does_not_create_spatial_cluster(executive, invalid):
    report = _measured_report(executive)
    if invalid == 'nan':
        report.smoke_planning_position.point.x = math.nan
    elif invalid == 'frame':
        report.smoke_planning_position.header.frame_id = 'unknown_map'
    elif invalid == 'zero_stamp':
        report.smoke_planning_position.header.stamp = stamp(0.0)
    elif invalid == 'future':
        report.smoke_planning_position.header.stamp = stamp(30.0)
    else:
        report.smoke_uncertainty_m = math.nan
    executive._verify_complete_site(report)
    assert not executive._completed_cluster_regions()


def test_boundary_uncertainty_requires_clearly_outside_evidence(executive):
    report = _measured_report(executive)
    report.smoke_uncertainty_m = 0.6
    executive._verify_complete_site(report)
    candidate = _next_cluster_frame(executive, point=(55.5, 0.0, 5.0))
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')


def test_independent_position_does_not_inherit_infinite_height_routing_cylinder(executive):
    executive._verify_complete_site(_measured_report(executive))
    for index in range(3):
        candidate = _next_cluster_frame(executive, point=(50.0, 0.0, 100.0))
        assert executive._verify_candidate_is_eligible(candidate, 'fire') == (index == 2)


def test_source_frame_gap_breaks_consecutive_outside_votes(executive):
    executive._verify_complete_site(_measured_report(executive))
    for _ in range(2):
        candidate = _next_cluster_frame(executive, yaw=0.8)
        assert not executive._verify_candidate_is_eligible(candidate, 'fire')
    _move_observer(executive, seconds=executive.seconds + 3.0)
    candidate = executive.detection(yaw=0.8)
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')


def test_continuous_completed_component_retains_identity_inside_boundary_band(executive):
    executive._verify_complete_site(_measured_report(executive))
    for offset in (19.0, 20.0, 21.0, 24.9, 25.0):
        candidate = _next_cluster_frame(executive, point=(30.0 + offset, 0.0, 5.0))
        assert executive._verify_is_completed_detection(candidate, 'fire')
    assert executive._completed_cluster_regions()[0]['radius'] == 25.0


def test_continuous_completed_bearing_keeps_identity_through_slow_parallax(executive):
    executive._verify_complete_site(_measured_report(executive))
    for degrees in (4.0, 8.0, 12.0, 16.0, 20.0):
        candidate = _next_cluster_frame(executive, yaw=math.radians(degrees))
        assert executive._completed_candidate_decision(candidate, 'fire') in (
            'completed', 'ambiguous')
        assert not executive._verify_candidate_is_eligible(candidate, 'fire')


def test_large_independent_position_jump_breaks_completed_continuity(executive):
    executive._verify_complete_site(_measured_report(executive))
    for index in range(3):
        candidate = _next_cluster_frame(executive, point=(130.0, 0.0, 5.0))
        assert executive._verify_candidate_is_eligible(candidate, 'fire') == (index == 2)


def _wall_observer(report, point):
    return PointStamped(header=deepcopy(report.smoke_planning_position.header),
                        point=Point(**dict(zip('xyz', point))))


@pytest.mark.parametrize('distance', [15.0, 20.0])
@pytest.mark.parametrize('bearing', [0.0, math.pi / 2, -math.pi / 3, math.pi])
def test_directional_routing_circle_is_entirely_behind_original_wall(
        executive, distance, bearing):
    observer = (11.0, -7.0, 60.0)
    normal = math.cos(bearing), math.sin(bearing)
    wall = (observer[0] + distance * normal[0], observer[1] + distance * normal[1], -10.0)
    report = _measured_report(executive, center=wall)
    original = deepcopy(report.smoke_planning_position)
    executive._verify_report_completion_context = {
        'wall_observer': _wall_observer(report, observer)}
    assert executive._verify_complete_site(report)
    entry = executive._completed_clusters[0]
    circle = executive._completed_cluster_regions()[0]
    center, radius = circle['center'].point, circle['radius']
    assert radius == 25.0
    assert (center.x, center.y, center.z) == pytest.approx(
        (wall[0] + radius * normal[0], wall[1] + radius * normal[1], wall[2]))
    assert entry['wall_point'] == original
    assert report.smoke_planning_position == original
    assert entry['front_normal'] == pytest.approx(normal)
    assert math.hypot(center.x - observer[0], center.y - observer[1]) == pytest.approx(
        distance + radius)
    assert circle['center'].header == original.header
    # Sample the core, buffer and full outer boundary in the map XY plane.
    for radial in (0.0, 20.0, radius):
        for step in range(64):
            angle = step * math.tau / 64
            point = center.x + radial * math.cos(angle), center.y + radial * math.sin(angle)
            depth = (point[0] - wall[0]) * normal[0] + (point[1] - wall[1]) * normal[1]
            assert depth >= -1e-12
    before_wall = wall[0] - 0.001 * normal[0], wall[1] - 0.001 * normal[1]
    assert math.hypot(before_wall[0] - center.x, before_wall[1] - center.y) > radius
    assert math.hypot(wall[0] - center.x, wall[1] - center.y) == pytest.approx(radius)


def test_retained_measurement_observer_wins_after_report_viewpoint_moves(executive):
    report = _measured_report(executive, center=(17.0, -4.0, -10.0))
    original = _wall_observer(report, (2.0, -4.0, 100.0))
    executive._verify_report_completion_context = {'wall_observer': original}
    _move_observer(executive, seconds=23.0, x=13.0, y=13.0, z=27.0, yaw=1.2)
    report.observer_planning_pose.header.stamp = stamp(23.0)
    report.observer_planning_pose.pose.position = Point(x=13.0, y=13.0, z=27.0)
    report.observer_planning_pose.pose.orientation = quaternion_from_yaw(1.2)
    assert executive._verify_complete_site(report)
    entry = executive._completed_clusters[0]
    assert entry['wall_observer'] == original
    assert entry['center'].point == Point(x=42.0, y=-4.0, z=-10.0)
    assert entry['center'].header.stamp == stamp(20.0)


def test_legacy_observer_fallback_uses_positions_instead_of_camera_yaw(executive):
    report = _measured_report(executive, center=(30.0, 0.0, 5.0))
    report.observer_planning_pose.pose.orientation = quaternion_from_yaw(math.pi / 2)
    assert executive._verify_complete_site(report)
    assert executive._completed_cluster_regions()[0]['center'].point == Point(x=55.0, z=5.0)


@pytest.mark.parametrize('invalid', [
    'missing', 'frame', 'nan_xy', 'nan_z', 'zero_stamp', 'future_stamp',
    'coincident_xy', 'almost_coincident_xy',
])
def test_invalid_retained_direction_never_invents_routing_zone_or_legacy_completion(
        executive, invalid):
    report = _measured_report(executive)
    observer = _wall_observer(report, (0.0, 0.0, 5.0))
    if invalid == 'missing':
        observer = None
    elif invalid == 'frame':
        observer.header.frame_id = 'another_map'
    elif invalid == 'nan_xy':
        observer.point.x = math.nan
    elif invalid == 'nan_z':
        observer.point.z = math.nan
    elif invalid == 'zero_stamp':
        observer.header.stamp = stamp(0.0)
    elif invalid == 'future_stamp':
        observer.header.stamp = stamp(30.0)
    elif invalid == 'coincident_xy':
        observer.point = Point(x=30.0, z=100.0)
    else:
        observer.point = Point(x=30.0 - 1e-10, z=5.0)
    executive._verify_report_completion_context = {'wall_observer': observer}
    assert not executive._verify_complete_site(report)
    assert report.smoke_location_valid  # The published finding and measured point remain valid.
    assert not executive._completed_cluster_regions()
    assert not executive._completed_sites
    assert executive._completed_cluster_failure_reason


def test_missing_retained_origin_and_invalid_report_pose_hold_without_yaw_guess(executive):
    report = _measured_report(executive)
    report.observer_planning_pose.header.frame_id = ''
    assert not executive._verify_complete_site(report)
    assert not executive._completed_cluster_regions()


def test_semantic_association_uses_wall_instead_of_shifted_routing_center(executive):
    executive._verify_complete_site(_measured_report(executive))
    assert executive._completed_cluster_regions()[0]['center'].point.x == 55.0
    front_source = _next_cluster_frame(executive, point=(10.0, 0.0, 5.0))
    assert executive._verify_is_completed_detection(front_source, 'fire')
    # This point is inside the shifted routing footprint, but lies 45 m from
    # the measured source anchor. Routing geometry must not veto its discovery.
    for index in range(3):
        other = _next_cluster_frame(executive, point=(75.0, 0.0, 5.0), track_id=42)
        assert executive._verify_candidate_is_eligible(other, 'fire') == (index == 2)


def test_merging_uses_original_wall_separation_instead_of_routing_centers(executive):
    executive._verify_complete_site(_measured_report(executive, center=(30.0, 0.0, 5.0)))
    executive._verify_complete_site(_measured_report(
        executive, center=(55.0, 0.0, 5.0), site_id=2))
    assert [entry['wall_point'].point.x for entry in executive._completed_clusters] == [30.0, 55.0]
    assert [entry['center'].point.x for entry in executive._completed_clusters] == [55.0, 80.0]


def test_routing_geometry_is_logged_once_when_new_circle_is_created(executive):
    logs = []
    executive._report = logs.append
    executive._verify_complete_site(_measured_report(executive))
    executive._verify_complete_site(_measured_report(executive, site_id=2))
    logs = [line for line in logs if 'completed smoke routing zone' in line]
    assert len(logs) == 1
    assert 'wall=(30.00, 0.00, 5.00)' in logs[0]
    assert 'center=(55.00, 0.00, 5.00)' in logs[0]
    assert 'radius=25.00' in logs[0]
    assert 'front_normal=(+1.000, +0.000)' in logs[0]


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_overlapping_nonmatches_do_not_become_novel_through_time_or_track_reset(
        executive, source):
    executive._verify_complete_site(_measured_report(executive))
    for index in range(12):
        candidate = _next_cluster_frame(
            executive, source=source, yaw=math.radians(22), track_id=index + 40)
        assert executive._completed_candidate_decision(candidate, source) == 'ambiguous'
        assert not executive._verify_candidate_is_eligible(candidate, source)
    assert executive._completed_followup_task() is not None


def test_candidate_bbox_intersection_is_used_instead_of_whole_camera_view(executive):
    source = 'smoke'
    executive._verify_complete_site(_measured_report(executive))
    message = executive.detection(source, yaw=math.radians(40))
    message.angular_bounds_valid = True
    message.yaw_min_rad, message.yaw_max_rad = math.radians(20), math.radians(45)
    message.vertical_min_rad, message.vertical_max_rad = -0.01, 0.01
    assert executive._completed_candidate_decision(message, source) == 'ambiguous'
    for _ in range(3):
        separate = _next_cluster_frame(executive, source=source, yaw=math.radians(60))
        executive._verify_candidate_is_eligible(separate, source)
    assert executive._verify_candidate_is_eligible(separate, source)


@pytest.mark.parametrize('vertical', [-math.radians(40), math.radians(40)])
def test_infinite_height_routing_cylinder_does_not_mask_distinct_vertical_source(
        executive, vertical):
    executive._verify_complete_site(_measured_report(executive))
    for index in range(3):
        candidate = _next_cluster_frame(executive, vertical=vertical)
        assert executive._verify_candidate_is_eligible(candidate, 'fire') == (index == 2)


def test_known_site_beyond_seventy_metres_has_no_identity_horizon(executive):
    executive._verify_complete_site(_measured_report(executive, center=(100.0, 0.0, 5.0)))
    _move_observer(executive, y=8.0)
    candidate = executive.detection(yaw=math.atan2(-8.0, 100.0), track_id=222)
    assert executive._completed_candidate_decision(candidate, 'fire') == 'ambiguous'
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')


def test_far_aligned_smoke_gets_bounded_alternate_view_investigation(executive):
    executive._verify_complete_site(_measured_report(
        executive, center=(60.0, 0.0, 5.0), fire=False))
    task = executive._completed_followup_task()
    assert task is not None
    assert task['origin'] == (0.0, 0.0, 5.0)
    _move_observer(executive, y=6.0)
    assert executive._completed_followup_result(task['key'], True, (0.0, 6.0, 5.0))
    assert executive._completed_followup_task() is None
    for index in range(3):
        executive.seconds += 0.1
        executive.odometry(executive.seconds - 0.01, y=6.0)
        executive.odometry(executive.seconds + 0.01, y=6.0)
        candidate = executive.detection('smoke', yaw=math.atan2(-6.0, 100.0))
        assert executive._verify_candidate_is_eligible(candidate, 'smoke') == (index == 2)
    executive._completed_followup_consume()
    assert not executive._verify_candidate_is_eligible(candidate, 'smoke')
    second = executive._completed_followup_task()
    assert second['origin'] == task['origin']
    assert second['observations'] == [(0.0, 6.0, 5.0)]
    assert executive._completed_followup_result(second['key'], True, (0.0, -6.0, 5.0))
    executive._completed_followup_consume()
    assert executive._completed_followup_task() is None
    for _ in range(5):
        executive._completed_candidate_decision(executive.detection('smoke'), 'smoke')
    assert executive._completed_followup_task() is None


def test_followup_requires_real_distinct_observer_displacement(executive):
    executive._verify_complete_site(_measured_report(executive, fire=False))
    task = executive._completed_followup_task()
    assert not executive._completed_followup_result(task['key'], True, (0.0, 0.5, 5.0))
    assert executive._completed_followup_task()['attempts'] == 1
    assert not executive._completed_followup_result(task['key'], True, (math.nan, 8.0, 5.0))
    assert executive._completed_followup_task() is None


def test_successful_followup_cannot_repeat_same_observation(executive):
    executive._verify_complete_site(_measured_report(executive, fire=False))
    key = executive._completed_followup_task()['key']
    assert executive._completed_followup_result(key, True, (0.0, 6.0, 5.0))
    executive._completed_followup_consume()
    assert not executive._completed_followup_result(key, True, (0.0, 6.0, 5.0))
    assert executive._completed_followup_task() is None


def test_clock_reset_clears_tasks_leases_and_requeue_tombstones(executive):
    executive._verify_complete_site(_measured_report(executive, fire=False))
    key = executive._completed_followup_task()['key']
    executive._completed_followup_result(key, True, (0.0, 6.0, 5.0))
    _move_observer(executive, seconds=10.0)
    assert executive._completed_followup_task() is None
    assert not executive._completed_followups
    assert not executive._completed_clusters


def test_followup_lease_expires_while_exploring_but_not_during_bound_inspection(
        executive, monkeypatch):
    import kv_executive.completed_sites as module
    now = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    executive._verify_complete_site(_measured_report(executive, fire=False))
    task = executive._completed_followup_task()
    executive._completed_followup_result(task['key'], True, (0.0, 6.0, 5.0))
    now[0] += 61.0
    executive._parent_state = 'Verify'
    assert executive._completed_followup_task() is None
    assert executive._completed_followups[task['key']]['lease']
    executive._parent_state = 'Exploration'
    assert executive._completed_followup_task() is not None
    assert not executive._completed_followups[task['key']]['lease']


def test_same_fresh_image_can_recover_association_after_pose_arrives(executive):
    executive._verify_complete_site(_measured_report(executive))
    executive._inspection_odometry_history.clear()
    candidate = executive.detection(yaw=1.0, track_id=200)
    assert executive._completed_candidate_decision(candidate, 'fire') == 'unknown'
    executive.odometry(19.9)
    executive.odometry(20.1)
    assert executive._completed_candidate_decision(candidate, 'fire') == 'pending'
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')


@pytest.mark.parametrize('offset,expected', [(0.0, 1), (20.0, 1), (20.01, None), (50.0, None)])
def test_report_identity_match_uses_original_wall_and_stable_id(executive, offset, expected):
    executive._verify_complete_site(_measured_report(executive))
    other = _measured_report(executive, center=(30.0 + offset, 0.0, 5.0), site_id=99)
    match = executive._completed_report_match(other)
    assert (match['site_id'] if match is not None else None) == expected
    if match is not None:
        match['wall_point'].point.x = 900.0
        assert executive._completed_clusters[0]['wall_point'].point.x == 30.0


def test_all_nearby_confirmed_components_are_saved_without_unrelated_side_plume(executive):
    report = _measured_report(executive)
    nearby = executive.detection('fire', yaw=math.radians(15.0), track_id=23)
    unrelated = executive.detection('fire', yaw=math.radians(65.0), track_id=24)
    executive._verify.raw_candidates = lambda kind: (
        [executive.fire, nearby, unrelated] if kind == 'fire' else [executive.smoke])
    executive._verify_complete_site(report)
    ids = [view['message'].track_id for view in executive._completed_clusters[0]['views']['fire']]
    assert ids == [1, 23]


def test_projection_diagnostics_are_throttled_and_include_reason(executive):
    logs = []
    executive._report = logs.append
    executive._verify_complete_site(_measured_report(executive))
    for _ in range(10):
        executive._completed_candidate_decision(executive.fire, 'fire')
    matching = [line for line in logs if 'source=fire' in line]
    assert len(matching) == 1
    assert 'site_id=1' in matching[0]
    assert 'reason=' in matching[0]


@pytest.mark.parametrize('point', [(6.0, 0.0, 5.0), (0.0, 0.0, 11.0)])
def test_followup_baseline_rejects_forward_only_and_vertical_only_motion(executive, point):
    executive._verify_complete_site(_measured_report(executive, fire=False))
    key = executive._completed_followup_task()['key']
    assert not executive._completed_followup_result(key, True, point)
    assert not executive._completed_followups[key]['lease']


def test_followup_leases_are_source_specific_and_both_sources_can_get_opportunity(executive):
    executive._verify_complete_site(_measured_report(executive))
    assert executive._completed_followup_task()['source'] == 'fire'
    smoke_task = next(task for task in executive._completed_followups.values()
                      if task['source'] == 'smoke')
    assert executive._completed_followup_result(smoke_task['key'], True, (0.0, 6.0, 5.0))
    _move_observer(executive, y=6.0)
    for _ in range(3):
        executive.seconds += 0.1
        executive.odometry(executive.seconds - 0.01, y=6.0)
        executive.odometry(executive.seconds + 0.01, y=6.0)
        candidate = executive.detection(yaw=math.atan2(-6.0, 100.0))
        assert executive._completed_candidate_decision(candidate, 'fire') == 'ambiguous'
    fire_tasks = [task for task in executive._completed_followups.values()
                  if task['source'] == 'fire']
    assert fire_tasks
    executive._completed_followup_consume()
    fire_task = fire_tasks[0]
    assert executive._completed_followup_result(fire_task['key'], True, (0.0, 6.0, 5.0))
    for index in range(3):
        executive.seconds += 0.1
        executive.odometry(executive.seconds - 0.01, y=6.0)
        executive.odometry(executive.seconds + 0.01, y=6.0)
        candidate = executive.detection(yaw=math.atan2(-6.0, 100.0))
        assert executive._verify_candidate_is_eligible(candidate, 'fire') == (index == 2)


def test_expired_lease_cannot_replay_cached_novel_image(executive, monkeypatch):
    import kv_executive.completed_sites as module
    now = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    executive._verify_complete_site(_measured_report(executive, fire=False))
    task = executive._completed_followup_task()
    executive._completed_followup_result(task['key'], True, (0.0, 6.0, 5.0))
    _move_observer(executive, y=6.0)
    for _ in range(3):
        executive.seconds += 0.1
        executive.odometry(executive.seconds - 0.01, y=6.0)
        executive.odometry(executive.seconds + 0.01, y=6.0)
        candidate = executive.detection('smoke', yaw=math.atan2(-6.0, 100.0))
        executive._verify_candidate_is_eligible(candidate, 'smoke')
    assert executive._verify_candidate_is_eligible(candidate, 'smoke')
    now[0] += 61.0
    assert not executive._verify_candidate_is_eligible(candidate, 'smoke')
    assert executive._completed_candidate_decision(candidate, 'smoke') == 'ambiguous'


def test_only_one_alternate_view_investigation_lease_can_be_active(executive):
    executive._verify_complete_site(_measured_report(executive))
    fire = executive._completed_followup_task()
    smoke = next(task for task in executive._completed_followups.values()
                 if task['source'] == 'smoke')
    assert fire['source'] == 'fire'
    assert executive._completed_followup_result(fire['key'], True, (0.0, 6.0, 5.0))
    assert not executive._completed_followup_result(smoke['key'], True, (0.0, 6.0, 5.0))
    assert not executive._completed_followup_result(fire['key'], True, (0.0, -6.0, 5.0))
    assert sum(task['lease'] for task in executive._completed_followups.values()) == 1
    assert executive._completed_followups[smoke['key']]['attempts'] == 0


def test_observer_parallax_does_not_requeue_exhausted_cluster_sector(executive):
    executive._verify_complete_site(_measured_report(
        executive, center=(15.0, 0.0, 5.0), fire=False))
    task = executive._completed_followup_task()
    executive._completed_followup_result(task['key'], False)
    executive._completed_followup_result(task['key'], False)
    for lateral in (6.0, 12.0, -12.0, -20.0):
        _move_observer(executive, seconds=executive.seconds + 1.0, y=lateral)
        candidate = executive.detection('smoke', yaw=math.atan2(-lateral, 15.0))
        assert executive._completed_candidate_decision(candidate, 'smoke') == 'ambiguous'
        assert executive._completed_followup_task() is None
    assert len(executive._completed_followups) == 1


def test_candidate_priority_prefers_separated_then_uninspected_angular_components(executive):
    executive._verify_complete_site(_measured_report(executive))
    task = executive._completed_followup_task()
    assert task['source'] == 'fire'
    _move_observer(executive, y=6.0)
    assert executive._completed_followup_result(task['key'], True, (0.0, 6.0, 5.0))
    for _ in range(3):
        executive.seconds += 0.1
        executive.odometry(executive.seconds - 0.01, y=6.0)
        executive.odometry(executive.seconds + 0.01, y=6.0)
        old = executive.detection(yaw=math.atan2(-6.0, 30.0), track_id=1)
        angular_new = executive.detection(yaw=math.radians(12.0), track_id=42)
        separate = executive.detection(yaw=math.radians(70.0), track_id=43)
        for message in (old, angular_new, separate):
            executive._completed_candidate_decision(message, 'fire')
    assert executive._completed_candidate_priority(separate, 'fire')[0] == 0
    assert executive._completed_candidate_priority(angular_new, 'fire')[0] == 1
    assert executive._completed_candidate_priority(old, 'fire')[0] == 2
    assert executive._completed_candidate_priority(separate, 'fire') \
        < executive._completed_candidate_priority(angular_new, 'fire') \
        < executive._completed_candidate_priority(old, 'fire')
    # Priority is advisory. It never suppresses an already eligible component.
    assert all(executive._verify_candidate_is_eligible(message, 'fire')
               for message in (old, angular_new, separate))


def test_first_site_candidate_priority_preserves_existing_selection(executive):
    assert executive._completed_candidate_priority(executive.detection(), 'fire') == (0, 0.0)


@pytest.mark.parametrize('map_yaw', [0.0, 0.7, -0.6])
def test_map_association_uses_image_time_transform_when_current_tf_is_not_available(
        executive, map_yaw):
    from rclpy.duration import Duration
    from rclpy.time import Time as RclTime
    from tf2_ros import Buffer
    executive.config['verify_planning_frame'] = 'map'
    executive.seconds = 21.0
    executive._inspection_odometry_history.clear()
    for acquired in (19.9, 20.1):
        executive.odometry(acquired, x=10.0, y=-4.0, z=5.0, yaw=0.3)
    executive.odometry(21.0, x=15.0, y=9.0, z=6.0, yaw=1.1)
    buffer = Buffer()
    camera = TransformStamped(
        header=Header(frame_id='kv_drone/base_link', stamp=stamp(20.0)),
        child_frame_id='thermal_camera', transform=Transform(
            translation=Vector3(x=0.25, y=-0.1, z=0.2),
            rotation=quaternion_from_yaw(0.2)))
    buffer.set_transform_static(camera, 'completed-site-test')
    for acquired in (19.9, 20.1):
        buffer.set_transform(TransformStamped(
            header=Header(frame_id='map', stamp=stamp(acquired)),
            child_frame_id='kv_drone/base_link', transform=Transform(
                translation=Vector3(x=100.0, y=30.0, z=15.0),
                rotation=quaternion_from_yaw(map_yaw))), 'completed-site-test')
    with pytest.raises(TransformException):
        buffer.lookup_transform('map', 'kv_drone/base_link', RclTime(seconds=21.0),
                                timeout=Duration(seconds=0.0))
    calls = []

    def lookup(target, source, acquired, timeout):
        calls.append((target, source, acquired.nanoseconds, timeout.nanoseconds))
        return buffer.lookup_transform(target, source, acquired, timeout=timeout)

    executive._tf_buffer = SimpleNamespace(lookup_transform=lookup)
    executive._completed_image_frame_transform = (
        MissionExecutive._completed_image_frame_transform.__get__(executive))

    def current_alignment(*_args):
        raise AssertionError('A camera image must not depend on the newest odometry TF')

    executive._completed_path_frame_transform = current_alignment
    message = executive.detection(
        header=Header(frame_id='thermal_camera', stamp=stamp(20.0)), yaw=0.1, vertical=0.2)
    origin, direction, observer = executive._completed_planning_ray(message, 'fire')
    expected_x = 100.0 + 0.25 * math.cos(map_yaw) + 0.1 * math.sin(map_yaw)
    expected_y = 30.0 + 0.25 * math.sin(map_yaw) - 0.1 * math.cos(map_yaw)
    assert (origin.x, origin.y, origin.z) == pytest.approx((expected_x, expected_y, 15.2))
    assert (observer.x, observer.y, observer.z) == pytest.approx((100.0, 30.0, 15.0))
    raw = (1.0, math.tan(0.1), math.tan(0.2))
    angle = map_yaw + 0.2
    expected = (math.cos(angle) * raw[0] - math.sin(angle) * raw[1],
                math.sin(angle) * raw[0] + math.cos(angle) * raw[1], raw[2])
    assert direction == pytest.approx(tuple(value / math.hypot(*raw) for value in expected))
    assert all(acquired == 20_000_000_000 and timeout == 0
               for _, _, acquired, timeout in calls)
    assert ('map', 'kv_drone/base_link', 20_000_000_000, 0) in calls


def test_missing_image_time_map_tf_cannot_borrow_a_newer_transform(executive):
    from tf2_ros import Buffer
    executive._verify_complete_site(_measured_report(executive))
    executive.config['verify_planning_frame'] = 'map'
    executive._completed_image_frame_transform = (
        MissionExecutive._completed_image_frame_transform.__get__(executive))
    buffer = Buffer()
    buffer.set_transform_static(TransformStamped(
        header=Header(frame_id='kv_drone/base_link', stamp=stamp(20.0)),
        child_frame_id='thermal_camera', transform=Transform(
            rotation=quaternion_from_yaw(0.0))), 'completed-site-test')
    buffer.set_transform(TransformStamped(
        header=Header(frame_id='map', stamp=stamp(21.0)),
        child_frame_id='kv_drone/base_link', transform=Transform(
            translation=Vector3(x=100.0), rotation=quaternion_from_yaw(0.0))),
        'completed-site-test')
    executive._tf_buffer = buffer
    candidate = executive.detection(yaw=1.0, track_id=88)
    assert executive._completed_planning_ray(candidate, 'fire') is None
    assert executive._completed_candidate_decision(candidate, 'fire') == 'unknown'
    assert not executive._verify_candidate_is_eligible(candidate, 'fire')
