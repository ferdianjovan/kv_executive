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
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.seconds * 1e9)))

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


@pytest.mark.parametrize('condition', ['missing_tf', 'missing_pose', 'invalid_pose', 'stale_image'])
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
