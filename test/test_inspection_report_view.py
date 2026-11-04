"""Bound final viewing to a fixed retained point and new forward smoke images."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Point, PointStamped, Pose, Transform, TransformStamped
import kv_executive.inspection_report_view as report_view
from kv_executive.inspection_report_view import InspectionReportViewMixin
from kv_executive.mission_geometry import quaternion_from_yaw
from kv_executive.verification import VerificationObservationLost
import pytest
from std_msgs.msg import Header
from tf2_ros import TransformException


def image(stamp, *, source='smoke'):
    header = Header(frame_id='rgb_camera' if source == 'smoke' else 'thermal_camera')
    header.stamp.sec = int(stamp)
    header.stamp.nanosec = round((stamp % 1.0) * 1e9)
    return SimpleNamespace(header=header, detected=True, status=3, CONFIRMED=3,
                           background_ready=True, regions=[4], coverage=0.01)


def retained(x=20.0, y=0.0, z=5.0):
    point = PointStamped(header=Header(frame_id='map'), point=Point(x=x, y=y, z=z))
    point.header.stamp.sec = 90
    return {'confirmed': True, 'confirmation_stamps': (89_000_000_000, 90_000_000_000),
            'nearest_point_valid': True, 'nearest_point': point, 'camera_frame': 'rgb_camera',
            'origin': Point(z=5.0), 'axis': (1.0, 0.0, 0.0), 'nearest': 20.0,
            'uncertainty': 0.2}


class Harness(InspectionReportViewMixin):
    def __init__(self):
        self.now = self.wall = 100.0
        self.config = {'verify_planning_frame': 'map', 'verify_report_view_timeout': 3.0,
                       'verify_report_view_yaw_tolerance': math.radians(30)}
        self.pose = Pose(position=Point(z=5.0), orientation=quaternion_from_yaw(0.0))
        self.camera = Transform(rotation=deepcopy(self.pose.orientation))
        self.camera.translation.z = self.pose.position.z
        self.conversion = Transform(rotation=quaternion_from_yaw(0.0))
        self.smoke = self.fire = None
        self.on_wait = lambda: None
        self.on_rotate = lambda: None
        self.ready = True
        self.prepare_success = True
        self.events = []
        self.missing_tf = False
        self._verify = SimpleNamespace(fresh_smoke=lambda: deepcopy(self.smoke),
                                       stable_fire=lambda: deepcopy(self.fire),
                                       _smoke_last_confirmed=None)
        self._tf_buffer = SimpleNamespace(lookup_transform=self.lookup)
        self._stop = SimpleNamespace(wait=self.wait)
        self._verify_wall_observation_context = {'settled_ns': 99_000_000_000}

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.now * 1e9)))

    def _inspection_detection_valid(self, message, source):
        if message is None:
            return False
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        return (message.detected and message.status == message.CONFIRMED
                and -0.25 <= self.now - stamp <= 2)

    def _check_running(self):
        return None

    def _report(self, message):
        self.events.append(('status', message))

    def _verify_control_pose(self):
        return None, deepcopy(self.pose)

    def _path_frame_transform(self, frame):
        assert frame == 'map'
        return deepcopy(self.conversion)

    def lookup(self, frame, camera, stamp, timeout):
        self.events.append(('tf', frame, camera, stamp.nanoseconds))
        if self.missing_tf:
            raise TransformException('camera transform missing')
        return TransformStamped(transform=deepcopy(self.camera))

    def wait(self, duration):
        self.now += duration
        self.wall += duration
        self.on_wait()
        return False

    def _verify_rotate(self, yaw, guard=None):
        if guard:
            guard()
        self.events.append(('rotate', yaw))
        self.pose.orientation = quaternion_from_yaw(yaw)
        self.camera.rotation = deepcopy(self.pose.orientation)
        self._verify_wall_observation_context = None
        self.on_rotate()
        self.wait(0.2)

    def _verify_prepare_wall_observation(self, guard=None, **kwargs):
        self.events.append(('prepare', kwargs))
        for _ in range(20):
            if guard:
                guard()
            self.wait(0.05)
        self._verify_wall_observation_context = {'settled_ns': round(self.now * 1e9)}
        return self.prepare_success

    def _verify_wall_observation_ready(self):
        return self.ready


@pytest.fixture
def vehicle(monkeypatch):
    value = Harness()
    monkeypatch.setattr(report_view.time, 'monotonic', lambda: value.wall)
    return value


def test_viewing_target_uses_current_pose_and_fixed_map_point_not_cached_range(vehicle):
    wall = retained(x=20, y=10)
    original = deepcopy(wall)
    vehicle.camera.translation.x, vehicle.camera.translation.y = 10.0, 10.0
    assert vehicle._verify_report_target_yaw(wall) == pytest.approx(0.0)
    vehicle.camera.translation.x, vehicle.camera.translation.y = 20.0, 0.0
    assert vehicle._verify_report_target_yaw(wall) == pytest.approx(math.pi / 2)
    assert wall == original
    assert all(event[-1] == 0 for event in vehicle.events if event[0] == 'tf')


def test_map_to_local_translation_does_not_add_the_drone_position_twice(vehicle):
    vehicle.conversion.translation.x = 100.0
    vehicle.conversion.translation.y = -40.0
    vehicle.camera.translation.x = 10.0
    assert vehicle._verify_report_target_yaw(retained(x=10, y=20)) == pytest.approx(math.pi / 2)


def test_camera_mount_yaw_is_compensated_when_facing_fixed_point(vehicle):
    vehicle.camera.rotation = quaternion_from_yaw(math.radians(30.0))
    assert vehicle._verify_report_target_yaw(retained()) == pytest.approx(math.radians(-30))


def test_one_yaw_then_settle_and_new_any_quadrant_smoke_without_wall_query(vehicle):
    wall = retained(x=0, y=20)
    original = deepcopy(wall)
    vehicle.on_rotate = lambda: setattr(
        vehicle, 'on_wait', lambda: setattr(vehicle, 'smoke', image(vehicle.now)))
    smoke = vehicle._verify_prepare_report_view(wall)
    assert [e[1] for e in vehicle.events if e[0] == 'rotate'] == pytest.approx([math.pi / 2])
    assert next(e[1] for e in vehicle.events if e[0] == 'prepare') == {
        'align_fire': False, 'wait_for_map': False}
    assert report_view._stamp_ns(smoke) > vehicle._verify_report_view_after_ns
    assert smoke.regions == [4]
    assert wall == original


def test_visible_smoke_does_not_force_exact_old_point_centering(vehicle):
    vehicle.smoke = image(vehicle.now)
    vehicle.on_wait = lambda: setattr(vehicle, 'smoke', image(vehicle.now))
    assert vehicle._verify_prepare_report_view(retained(x=20, y=5))
    assert not any(e[0] == 'rotate' for e in vehicle.events)


def test_visible_unrelated_smoke_does_not_prevent_facing_a_retained_point_behind(vehicle):
    vehicle.smoke = image(vehicle.now)
    vehicle.on_wait = lambda: setattr(vehicle, 'smoke', image(vehicle.now))
    assert vehicle._verify_prepare_report_view(retained(x=-20))
    assert [e[1] for e in vehicle.events if e[0] == 'rotate'] == pytest.approx([math.pi])


def test_visible_fire_retains_yaw_while_waiting_for_rgb_smoke(vehicle):
    vehicle.fire = image(vehicle.now, source='fire')

    def returning_smoke():
        if vehicle.now > 101.2:
            vehicle.smoke = image(vehicle.now)

    vehicle.on_wait = returning_smoke
    assert vehicle._verify_prepare_report_view(retained(x=-20), fire_message=vehicle.fire)
    assert not any(e[0] in {'rotate', 'tf'} for e in vehicle.events)


def test_old_still_fresh_smoke_cannot_substitute_for_a_post_settle_image(vehicle):
    vehicle.smoke = image(vehicle.now)
    with pytest.raises(VerificationObservationLost, match='Fresh forward smoke unavailable'):
        vehicle._verify_prepare_report_view(retained())
    assert vehicle.wall <= 103.06


def test_missing_smoke_cannot_be_inferred_from_retained_coordinates(vehicle):
    with pytest.raises(VerificationObservationLost, match='Fresh forward smoke unavailable'):
        vehicle._verify_prepare_report_view(retained())
    assert vehicle.wall <= 103.06


def test_missing_camera_tf_can_recover_through_a_fresh_visible_image(vehicle):
    vehicle.missing_tf = True
    vehicle.on_wait = lambda: setattr(vehicle, 'smoke', image(vehicle.now))
    assert vehicle._verify_prepare_report_view(retained(x=0, y=20))
    assert not any(e[0] == 'rotate' for e in vehicle.events)


@pytest.mark.parametrize('reason', ['settling', 'motion'])
def test_unsettled_report_never_accepts_even_visible_smoke(vehicle, reason):
    vehicle.on_wait = lambda: setattr(vehicle, 'smoke', image(vehicle.now))
    if reason == 'settling':
        vehicle.prepare_success = False
    else:
        vehicle.ready = False
    with pytest.raises(VerificationObservationLost, match='viewpoint'):
        vehicle._verify_prepare_report_view(retained())


def test_operator_or_original_deadline_guard_interrupts_the_bounded_view(vehicle):
    interrupted = RuntimeError('operator requested return')

    def guard():
        raise interrupted

    with pytest.raises(RuntimeError) as caught:
        vehicle._verify_prepare_report_view(retained(), guard=guard)
    assert caught.value is interrupted
    assert not any(e[0] == 'rotate' for e in vehicle.events)


@pytest.mark.parametrize('invalid', [
    'unconfirmed', 'replay', 'frame', 'nan', 'missing', 'wrong_stamp', 'uncertainty', 'nearest',
])
def test_invalid_retained_point_cannot_authorize_a_report_view(vehicle, invalid):
    wall = retained()
    if invalid == 'unconfirmed':
        wall['confirmed'] = False
    elif invalid == 'replay':
        wall['confirmation_stamps'] = (90, 90)
    elif invalid == 'frame':
        wall['nearest_point'].header.frame_id = 'other_map'
    elif invalid == 'nan':
        wall['nearest_point'].point.x = math.nan
    elif invalid == 'wrong_stamp':
        wall['nearest_point'].header.stamp.sec = 89
    elif invalid == 'uncertainty':
        wall['uncertainty'] = 2.1
    elif invalid == 'nearest':
        wall['nearest'] = 0.0
    else:
        wall['nearest_point_valid'] = False
    with pytest.raises(VerificationObservationLost, match='confirmed retained'):
        vehicle._verify_prepare_report_view(wall)
