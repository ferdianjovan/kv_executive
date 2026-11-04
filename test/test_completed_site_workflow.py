"""Exercise completed-source filtering, alternate views and report commits together."""

from copy import deepcopy
import math
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import Point, Pose, Transform, TransformStamped, Vector3
from kv_executive.completed_sites import COMPLETED_SITE_DEFAULTS, CompletedSitesMixin
from kv_executive.mission_geometry import quaternion_from_yaw, yaw_from_quaternion
from kv_executive.verification import VERIFY_DEFAULTS, VerifyController
from kv_perception.msg import (
    FireDetection, FireDetectionArray, SmokeDetection, SmokeDetectionArray,
)
from nav_msgs.msg import Odometry
import pytest
from test_inspection_navigation import (
    _confirmed_report_wall, Harness as NavigationHarness, header,
)
from tf2_ros import TransformException


class Workflow(NavigationHarness, CompletedSitesMixin):
    """Keep real association and controller filtering around deterministic flight services."""

    def __init__(self, *, fire_found=True):
        super().__init__()
        self.config = {**VERIFY_DEFAULTS, **COMPLETED_SITE_DEFAULTS, **self.config,
                       'robot_base_frame': 'kv_drone/base_link'}
        self._initialize_completed_sites()
        self._goal_lock = threading.Lock()
        self._requested_mission_state = None
        self.transform_available = True
        self._tf_buffer = SimpleNamespace(lookup_transform=self._camera_transform)
        self._completed_path_frame_transform = self._path_frame_transform
        self._verify = VerifyController(self)
        self.move(45.0, 0.0)
        self.observe(fire_found=fire_found)
        _confirmed_report_wall(self, nearest=15.0)

    def _camera_transform(self, *_args, **_kwargs):
        if not self.transform_available:
            raise TransformException('Image-time camera transform not available yet')
        return TransformStamped(transform=Transform(rotation=quaternion_from_yaw(0.0)))

    def move(self, x, y, *, z=20.0, yaw=0.0):
        self.now += 0.1
        pose = Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                    orientation=quaternion_from_yaw(yaw))
        for acquired in (self.now - 0.01, self.now + 0.01):
            odometry = Odometry(header=header(acquired), pose=deepcopy(self.odometry.pose))
            odometry.pose.pose = deepcopy(pose)
            with self._data_lock:
                self._completed_odometry_locked(odometry)
        self.odometry.header = header(self.now)
        self.odometry.pose.pose = deepcopy(pose)
        self._global_position.header = header(self.now, 'gps')
        self._global_position_received = time.monotonic()

    def observe(self, *, yaw=0.0, fire_found=True, track_id=1, smoke_yaw=None, vertical=0.0):
        self.move(self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y,
                  z=self.odometry.pose.pose.position.z,
                  yaw=yaw_from_quaternion(self.odometry.pose.pose.orientation))
        self.smoke = SmokeDetection(
            header=header(self.now, 'rgb_camera'), detected=True,
            status=SmokeDetection.CONFIRMED, background_ready=True, coverage=0.2,
            yaw_correction_rad=float(yaw if smoke_yaw is None else smoke_yaw))
        self._verify.smoke_callback(self.smoke)
        self.fire = (FireDetection(
            header=header(self.now, 'thermal_camera'), detected=True,
            status=FireDetection.CONFIRMED, source_count=1, track_id=track_id,
            confirmation_frames=10, persistence_sec=3.0,
            image_width=640, image_height=480, area_px=100,
            bbox_x=310, bbox_y=230, bbox_width=20, bbox_height=20,
            center_x=320.0, center_y=240.0, yaw_correction_rad=float(yaw),
            vertical_correction_rad=float(vertical))
            if fire_found else None)
        if self.fire is not None:
            self._verify.fire_callback(self.fire)

    def _verify_prepare_report_view(self, retained_wall, *args, guard=None, **kwargs):
        if guard:
            guard()
        self._verify_report_view_after_ns = 0
        return self._inspection_report_smoke(retained_wall)

    def first_report(self):
        return self._verify_report_inspection({}, self._verify.stable_fire(), attempts=3)


def test_first_report_then_track_reset_does_not_generate_second_report_or_site():
    executive = Workflow()
    original = executive.first_report()
    assert original.site_id == 1 and original.fire_found
    assert len(executive._completed_cluster_regions()) == 1
    executive.observe(track_id=77)
    assert executive._verify.fresh_smoke() is None
    assert executive._verify.stable_fire() is None
    assert executive._verify.raw_fresh_smoke() is not None
    assert executive._verify.raw_fresh_fire() is not None
    blackboard = {'verify_report_wall': deepcopy(executive._verify_retained_report_wall)}
    assert executive._verify.center_fire(blackboard) == 'depart'
    result = blackboard['verify_completed_report']
    assert result.site_id == original.site_id and result.fire_found
    assert executive._site_sequence == 1
    assert len([event for event in executive.events if event[0] == 'report']) == 1
    assert len([event for event in executive.events if event[0] == 'legacy_report']) == 1
    task = executive._completed_followup_task()
    assert task is not None and task['site_id'] == original.site_id


def test_farther_collinear_smoke_gets_bounded_investigation_and_new_measured_site():
    executive = Workflow()
    first = executive.first_report()
    assert first.smoke_planning_position.point.x == pytest.approx(60.0)
    assert not first.fire_location_valid  # Smoke range never becomes a fire depth.
    executive.move(0.0, 0.0)
    executive.observe(track_id=77)
    assert executive._verify.fresh_smoke() is None
    assert executive._completed_candidate_decision(executive.smoke, 'smoke') == 'ambiguous'
    task = executive._completed_followup_task()
    assert task is not None
    assert task['source'] == 'fire'  # A known fire's paired smoke stays navigation-filtered.
    initial_origin = task['origin']
    executive.move(0.0, 6.0)
    assert executive._completed_followup_result(task['key'], True, (0.0, 6.0, 20.0))
    assert executive._completed_followups[task['key']]['origin'] == initial_origin
    assert executive._completed_followup_task() is None  # One live investigation lease.
    for _ in range(3):
        executive.observe(yaw=math.atan2(-6.0, 100.0), track_id=88)
    assert executive._verify.fresh_smoke() is None
    assert executive._inspection_report_smoke() is not None
    assert executive._verify.stable_fire() is not None
    # The report guard is tested after arrival, not with a fictitious 100 m
    # LiDAR return. The route around the old zone is outside this protocol test.
    executive.move(118.0, 0.0, yaw=math.pi)
    for _ in range(3):
        executive.observe(yaw=0.0, vertical=math.radians(30.0), track_id=88)
    wall = _confirmed_report_wall(executive, nearest=18.0)
    wall['nearest_point'].point.x = 100.0
    wall['axis'], wall['bearing'] = (-1.0, 0.0, 0.0), math.pi
    assert math.dist((118.0, 0.0, 20.0), (100.0, 0.0, 20.0)) == wall['nearest']
    second = executive.first_report()
    assert second.smoke_planning_position.point.x == pytest.approx(100.0)
    assert second.site_id == 2 and second.fire_found
    assert len(executive._completed_cluster_regions()) == 2
    assert not second.fire_location_valid
    executive._completed_followup_consume()
    assert not any(task['lease'] for task in executive._completed_followups.values())


def test_alternate_view_of_same_wall_cannot_publish_again_during_investigation_lease():
    executive = Workflow()
    first = executive.first_report()
    executive.move(0.0, 0.0)
    executive.observe(track_id=77)
    task = executive._completed_followup_task()
    executive.move(0.0, 6.0)
    assert executive._completed_followup_result(task['key'], True, (0.0, 6.0, 20.0))
    for _ in range(3):
        executive.observe(yaw=math.atan2(-6.0, 60.0), track_id=88)
    assert executive._verify.stable_fire() is not None
    duplicate = executive.first_report()
    assert duplicate.site_id == first.site_id
    assert not executive._verify_report_was_published
    assert len([event for event in executive.events if event[0] == 'report']) == 1
    assert len([event for event in executive.events if event[0] == 'legacy_report']) == 1
    assert executive._site_sequence == 1


def test_separate_direction_is_eligible_without_any_new_lidar_measurement():
    executive = Workflow()
    executive.first_report()
    before = len([event for event in executive.events if event[0] == 'query'])
    for _ in range(3):
        executive.observe(yaw=math.radians(70.0), track_id=88)
    assert executive._verify.fresh_smoke() is not None
    assert executive._verify.stable_fire() is not None
    assert len([event for event in executive.events if event[0] == 'query']) == before


def test_new_fire_upgrades_negative_site_with_raw_smoke_without_reopening_smoke_navigation():
    executive = Workflow(fire_found=False)
    first = executive.first_report()
    assert first.site_id == 1 and not first.fire_found
    executive.observe(track_id=99)
    assert executive._verify.fresh_smoke() is None
    assert executive._verify.stable_fire() is not None
    assert executive._inspection_report_smoke() is not None
    assert not executive._verify._completed_arrival({
        'verify_report_wall': deepcopy(executive._verify_retained_report_wall)})
    upgraded = executive.first_report()
    assert upgraded.site_id == first.site_id and upgraded.fire_found
    assert len(executive._completed_cluster_regions()) == 1
    assert executive._completed_clusters[0]['fire_found']
    assert executive._site_sequence == 1
    assert len([event for event in executive.events if event[0] == 'report']) == 2
    assert len([event for event in executive.events if event[0] == 'legacy_report']) == 1


def test_source_time_transform_retry_does_not_erase_cluster_on_new_controller_goal():
    executive = Workflow()
    executive.first_report()
    executive._verify.reset()
    executive.transform_available = False
    executive.observe(yaw=math.radians(70.0), track_id=99)
    assert executive._verify.stable_fire() is None
    assert len(executive._completed_clusters) == 1
    executive.transform_available = True
    for _ in range(3):
        executive.observe(yaw=math.radians(70.0), track_id=99)
    assert executive._verify.stable_fire() is not None
    executive.move(-30.0, 6.0)
    assert len(executive._completed_clusters) == 1
    executive.now -= 20.0
    with executive._data_lock:
        executive._completed_clock_locked()
    assert not executive._completed_clusters
    assert not executive._completed_followups


@pytest.mark.parametrize('yaw', [math.radians(90.0), math.radians(-90.0)])
def test_raw_report_smoke_cannot_substitute_an_unrelated_direction(yaw):
    executive = Workflow(fire_found=False)
    executive.first_report()
    executive.observe(yaw=yaw, fire_found=False)
    # Hold navigation filtering out independently so the report-only fallback
    # itself must reject wrong/behind-wall evidence.
    executive._verify.fresh_smoke = lambda: None
    assert executive._inspection_report_smoke() is None


@pytest.mark.parametrize('rotation', [math.pi / 2, -0.7])
def test_completed_geometry_and_candidate_projection_share_map_local_alignment(rotation):
    executive = Workflow()
    executive.conversion = Transform(
        translation=Vector3(x=100.0, y=-40.0, z=3.0),
        rotation=quaternion_from_yaw(rotation))
    wall = _confirmed_report_wall(executive, nearest=15.0)
    first = executive.first_report()
    assert first.smoke_planning_position == wall['nearest_point']
    assert first.smoke_local_position.point.x == pytest.approx(60.0)
    assert first.smoke_planning_position.point.x != first.smoke_local_position.point.x
    original_map_point = deepcopy(first.smoke_planning_position)
    executive.observe(track_id=99)
    assert executive._verify.fresh_smoke() is None
    assert executive._verify.stable_fire() is None
    assert executive._completed_report_match(first)['site_id'] == first.site_id
    for _ in range(3):
        executive.observe(yaw=math.radians(70.0), track_id=100)
    assert executive._verify.stable_fire() is not None
    assert executive._completed_clusters[0]['wall_point'] == original_map_point


def test_report_raw_smoke_selects_matching_component_and_requires_image_time_pose():
    executive = Workflow(fire_found=False)
    executive.first_report()
    executive.observe(fire_found=False)
    associated = deepcopy(executive.smoke)
    unrelated = deepcopy(associated)
    unrelated.yaw_correction_rad = math.radians(70.0)
    executive._verify.raw_candidates = lambda source: [unrelated, associated]
    executive._verify.fresh_smoke = lambda: None
    selected = executive._inspection_report_smoke()
    assert selected == associated and selected is not associated
    executive.transform_available = False
    assert executive._inspection_report_smoke() is None
    executive.transform_available = True
    assert executive._inspection_report_smoke() == associated
    executive.now += executive.config['verify_observation_timeout'] + 1.0
    assert executive._inspection_report_smoke() is None


def test_new_goal_report_view_uses_raw_rgb_camera_for_completed_negative_site_upgrade():
    executive = Workflow(fire_found=False)
    executive.first_report()
    executive._verify.reset()
    executive.observe(track_id=99)
    assert executive._verify.fresh_smoke() is None
    assert executive._verify._smoke_last_confirmed is None
    wall = executive._verify_retained_report_wall
    wall['camera_frame'] = 'thermal_camera'  # Wall was probed while tracking thermal fire.
    assert executive._verify_report_camera_frame(wall) == 'rgb_camera'


@pytest.mark.parametrize('array', [False, True])
def test_newest_image_waiting_for_pose_does_not_veto_fresh_accepted_new_site(array):
    executive = Workflow()
    executive.first_report()
    for _ in range(3):
        executive.observe(yaw=math.radians(70.0), fire_found=False)
    accepted = executive._verify.fresh_smoke()
    assert accepted is not None
    executive.now += 0.1
    newest = deepcopy(accepted)
    newest.header = header(executive.now, 'rgb_camera')
    # Deliberately deliver the camera frame before the following odometry sample.
    assert executive._completed_source_pose(round(executive.now * 1e9)) is None
    if array:
        executive._verify.candidates_callback(SmokeDetectionArray(
            header=deepcopy(newest.header), detections=[newest]), 'smoke')
    else:
        executive._verify.smoke_callback(newest)
    assert executive._completed_candidate_decision(newest, 'smoke') == 'unknown'
    assert executive._verify.raw_candidates('smoke')[0].header == newest.header
    assert executive._verify.fresh_smoke() == accepted
    assert not executive._verify_request_is_completed()


@pytest.mark.parametrize('array', [False, True])
def test_unknown_newest_image_without_fresh_prior_association_stays_withheld(array):
    executive = Workflow()
    executive.first_report()
    executive._verify.reset()
    executive.now += 0.1
    newest = SmokeDetection(
        header=header(executive.now, 'rgb_camera'), detected=True,
        status=SmokeDetection.CONFIRMED, background_ready=True,
        yaw_correction_rad=math.radians(70.0), coverage=0.2)
    if array:
        executive._verify.candidates_callback(SmokeDetectionArray(
            header=deepcopy(newest.header), detections=[newest]), 'smoke')
    else:
        executive._verify.smoke_callback(newest)
    assert executive._verify.fresh_smoke() is None
    assert executive._verify_request_is_completed()


def test_bounded_fire_opportunity_prefers_visible_uninspected_component_over_sticky_old_one():
    executive = Workflow()
    executive.first_report()
    task = executive._completed_followup_task()
    executive.move(45.0, 6.0)
    assert executive._completed_followup_result(task['key'], True, (45.0, 6.0, 20.0))
    old_bearing = math.atan2(-6.0, 15.0)
    for _ in range(3):
        executive.observe(yaw=old_bearing, track_id=1)
    assert executive._verify.stable_fire().track_id == 1
    for _ in range(3):
        executive.move(45.0, 6.0)
        old = deepcopy(executive.fire)
        old.header = header(executive.now, 'thermal_camera')
        novel = deepcopy(old)
        novel.track_id = 88
        novel.yaw_correction_rad = math.radians(12.0)
        executive._verify.candidates_callback(FireDetectionArray(
            header=deepcopy(old.header), detections=[old, novel]), 'fire')
    assert executive._verify.stable_fire().track_id == 88
    assert executive._completed_candidate_priority(novel, 'fire') \
        < executive._completed_candidate_priority(old, 'fire')


def test_expired_accepted_frame_cannot_override_newest_unknown_image():
    executive = Workflow()
    executive.first_report()
    for _ in range(3):
        executive.observe(yaw=math.radians(70.0), fire_found=False)
    accepted = executive._verify.fresh_smoke()
    assert accepted is not None
    executive.now += 0.1
    newest = deepcopy(accepted)
    newest.header = header(executive.now, 'rgb_camera')
    executive._verify.smoke_callback(newest)
    with executive._verify._lock:
        message, acquired, _ = executive._verify._observations['smoke']
        expired = time.monotonic() - executive.config['verify_observation_timeout'] - 0.1
        executive._verify._observations['smoke'] = (message, acquired, expired)
    assert executive._verify.fresh_smoke() is None
    assert executive._verify_request_is_completed()
