"""Historical camera bearings, sticky source selection, and bounded fire alignment."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Pose, Quaternion, Transform, TransformStamped
import kv_executive.inspection_heading as heading_module
from kv_executive.inspection_heading import InspectionHeadingMixin
from kv_executive.mission_geometry import (
    angular_distance, quaternion_from_yaw, yaw_from_quaternion,
)
import pytest
from std_msgs.msg import Header
from tf2_ros import TransformException


def image(stamp=100.0, *, source='fire', yaw=0.0, vertical=0.0):
    header = Header(frame_id='thermal_camera' if source == 'fire' else 'rgb_camera')
    header.stamp.sec = int(stamp)
    header.stamp.nanosec = round((stamp % 1.0) * 1e9)
    confirmed = 2 if source == 'fire' else 3
    return SimpleNamespace(header=header, detected=True, status=confirmed, CONFIRMED=confirmed,
                           background_ready=True, yaw_correction_rad=yaw,
                           vertical_correction_rad=vertical)


class Harness(InspectionHeadingMixin):
    def __init__(self):
        self.now = self.wall = 100.0
        self.config = {
            'verify_planning_frame': 'map', 'verify_observation_timeout': 2.0,
            'verify_future_tolerance': 0.25, 'verify_reacquire_yaw_step': math.radians(5.0),
            'verify_heading_rate': math.radians(12.0), 'verify_fire_heading_grace': 2.0,
            'verify_heading_clock_timeout': 5.0,
            'verify_fire_heading_tolerance': math.radians(2.0),
            'verify_fire_heading_align_timeout': 15.0,
            'robot_base_frame': 'body',
        }
        self.actual = Pose(orientation=quaternion_from_yaw(0.0))
        self.conversion = Transform(rotation=quaternion_from_yaw(0.0))
        self.camera = Transform(rotation=quaternion_from_yaw(0.0))
        self.body = Transform(rotation=quaternion_from_yaw(0.0))
        self.mount_yaw = 0.0
        self.fire_world_target = None
        self.freeze = False
        self.fire = self.smoke = None
        self.events = []
        self.missing_tf = False
        self.on_transform = lambda: None
        self.on_rotate = lambda: None
        self._verify = SimpleNamespace(stable_fire=lambda: deepcopy(self.fire),
                                       fresh_smoke=lambda: deepcopy(self.smoke))
        self._tf_buffer = SimpleNamespace(lookup_transform=self.lookup_transform)
        self._stop = SimpleNamespace(wait=self.wait)
        self._verify_wall_observation_context = {'valid': True}

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.now * 1e9)))

    def _inspection_detection_valid(self, message, source):
        if message is None:
            return False
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        return (message.detected and message.status == message.CONFIRMED
                and (source != 'smoke' or message.background_ready)
                and -0.25 <= self.now - stamp <= 2.0)

    def lookup_transform(self, frame, camera, stamp, timeout):
        self.events.append(('tf', frame, camera, stamp.nanoseconds))
        self.on_transform()
        if self.missing_tf:
            raise TransformException('historical camera transform missing')
        return TransformStamped(transform=deepcopy(self.body if camera == 'body' else self.camera))

    def wait(self, duration):
        self.wall += duration
        if self.freeze:
            return
        self.now += duration
        if self.fire is not None:
            body_yaw = yaw_from_quaternion(self.actual.orientation)
            camera_yaw = body_yaw + self.mount_yaw
            self.body.rotation = quaternion_from_yaw(body_yaw)
            self.camera.rotation = quaternion_from_yaw(camera_yaw)
            target = self.fire_world_target
            if target is None:
                target = self._heading_state()['fire']['bearing']
            self.fire = image(self.now, yaw=angular_distance(target, camera_yaw))

    def _path_frame_transform(self, frame):
        assert frame == 'map'
        return deepcopy(self.conversion)

    def _verify_control_pose(self):
        return None, deepcopy(self.actual)

    def _report(self, text):
        self.events.append(('status', text))

    def _verify_rotate(self, yaw, guard=None, *, yaw_tolerance=None):
        assert self._verify_wall_observation_context is None
        self.events.append(('rotate', yaw, yaw_tolerance))
        self.on_rotate()
        if guard:
            guard()
        self.actual.orientation = quaternion_from_yaw(yaw)


@pytest.fixture
def node(monkeypatch):
    value = Harness()
    monkeypatch.setattr(heading_module.time, 'monotonic', lambda: value.wall)
    return value


def test_image_bearing_uses_historical_camera_mount_roll_and_local_alignment(node):
    # Camera roll moves image vertical error into horizontal bearing. Its
    # acquisition-time yaw and map/local rotation both differ from actual yaw.
    half_yaw = 0.7 / 2
    half_roll = math.pi / 4
    node.camera.rotation = Quaternion(
        x=math.sin(half_roll) * math.cos(half_yaw),
        y=math.sin(half_roll) * math.sin(half_yaw),
        z=math.cos(half_roll) * math.sin(half_yaw),
        w=math.cos(half_roll) * math.cos(half_yaw))
    node.camera.translation.x = 20.0
    node.conversion.rotation = quaternion_from_yaw(0.4)
    node.conversion.translation.y = 50.0
    node.actual.orientation = quaternion_from_yaw(-1.0)
    node.now = 101.0
    assert node._verify_image_bearing(image(yaw=0.2, vertical=0.3)) == pytest.approx(0.8)
    assert node.events[0] == ('tf', 'map', 'thermal_camera', 100_000_000_000)


def test_missing_historical_tf_holds_heading_without_reinterpreting_old_image(node):
    node.fire = image(yaw=0.5)
    node.actual.orientation = quaternion_from_yaw(1.0)
    node.missing_tf = True
    provider = node._verify_live_source_heading(0.7)
    assert provider(Pose()) == 0.7
    node.missing_tf = False
    assert provider(Pose()) == 0.7
    assert node._verify_live_source_heading(0.8)(Pose()) == 0.8
    assert len([e for e in node.events if e[0] == 'tf']) == 1
    node.wall += 0.3
    assert provider(Pose()) == pytest.approx(1.0 - math.radians(5))
    assert len([e for e in node.events if e[0] == 'tf' and e[2] != 'body']) == 2


def test_pitched_camera_uses_vertical_angle_when_projecting_horizontal_bearing(node):
    node.camera.rotation = Quaternion(y=math.sin(math.pi / 4), w=math.cos(math.pi / 4))
    assert node._verify_image_bearing(image(yaw=0.3, vertical=0.2)) == pytest.approx(
        math.atan2(math.tan(0.3), math.tan(0.2)))


def test_slow_failed_tf_lookup_still_has_a_retry_pause_after_it_returns(node):
    node.fire = image(yaw=0.3)
    node.missing_tf = True
    node.on_transform = lambda: setattr(node, 'wall', node.wall + 0.5)
    provider = node._verify_live_source_heading(0.7)
    assert provider(Pose()) == 0.7
    assert provider(Pose()) == 0.7
    assert len([e for e in node.events if e[0] == 'tf']) == 1
    node.wall += 0.3
    assert provider(Pose()) == 0.7
    assert len([e for e in node.events if e[0] == 'tf']) == 2


def test_missing_map_alignment_tf_is_unavailable_but_control_faults_still_propagate(node):
    def missing(frame):
        try:
            raise TransformException('map/body transform missing')
        except TransformException as error:
            raise RuntimeError('Cannot align local frame') from error

    node._path_frame_transform = missing
    assert node._verify_image_bearing(image(yaw=0.3)) is None

    def stale_telemetry(frame):
        raise RuntimeError('Stale odometry')

    node._path_frame_transform = stale_telemetry
    with pytest.raises(RuntimeError, match='Stale odometry'):
        node._verify_image_bearing(image(yaw=0.3))


def test_reordered_image_cannot_replace_a_newer_consumed_absolute_bearing(node):
    node.fire = image(yaw=0.3)
    node._heading_observation('fire', node.fire)
    node.now += 0.5
    newer = image(node.now, yaw=-0.3)
    node._heading_observation('fire', newer)
    assert node._heading_observation('fire', node.fire) is None
    assert node._heading_state()['fire']['bearing'] == pytest.approx(-0.3)
    assert len([e for e in node.events if e[0] == 'tf' and e[2] != 'body']) == 2


def test_fire_priority_survives_a_brief_gap_then_allows_new_smoke_guidance(node):
    node.fire = image(yaw=-0.4)
    node.smoke = image(source='smoke', yaw=0.4)
    provider = node._verify_live_source_heading(0.0)
    assert provider(Pose()) == pytest.approx(-math.radians(5))
    node.fire = None
    node.now = node.wall = 101.0
    node.smoke = image(node.now, source='smoke', yaw=0.4)
    assert provider(Pose()) == pytest.approx(-math.radians(5))
    assert not any(e[0] == 'tf' and e[2] == 'rgb_camera' for e in node.events)
    node.now = node.wall = 102.1
    node.smoke = image(node.now, source='smoke', yaw=0.4)
    assert provider(Pose()) == pytest.approx(math.radians(5))


def test_frozen_source_clock_cannot_extend_fire_memory_beyond_wall_timeout(node):
    node.fire = image(yaw=0.4)
    provider = node._verify_live_source_heading(0.0)
    first = provider(Pose())
    node.fire = None
    node.wall += 4.9
    assert node._heading_fire_memory() is not None
    node.wall += 0.2
    assert node._heading_fire_memory() is None
    node.actual.orientation = quaternion_from_yaw(first)
    assert provider(Pose()) == first


def test_progressing_slow_clock_keeps_the_full_source_time_fire_grace(node):
    node.fire = image(yaw=-0.4)
    provider = node._verify_live_source_heading(0.0)
    first = provider(Pose())
    node.fire = None
    node.now += 1.0
    node.wall += 10.0
    node.smoke = image(node.now, source='smoke', yaw=0.4)
    assert node._heading_fire_memory() is not None
    assert provider(Pose()) == pytest.approx(first)
    assert not any(e[0] == 'tf' and e[2] == 'rgb_camera' for e in node.events)


def test_cached_image_bearing_survives_wall_age_when_the_slow_source_clock_progresses(node):
    node.fire = image(yaw=0.4)
    provider = node._verify_live_source_heading(0.0)
    first = provider(Pose())
    node.actual.orientation = quaternion_from_yaw(first)
    node.now += 1.0
    node.wall += 10.0
    assert provider(Pose()) > first
    assert len([e for e in node.events if e[0] == 'tf' and e[2] != 'body']) == 1


def test_consumption_survives_source_switches_and_new_legs_without_angle_replay(node):
    node.config['verify_fire_heading_grace'] = 0.0
    node.smoke = image(source='smoke', yaw=0.3)
    provider = node._verify_live_source_heading(0.0)
    provider(Pose())
    node.now += 0.1
    node.fire = image(node.now, yaw=-0.3)
    provider(Pose())
    node.now += 0.1
    node.fire = None
    node.actual.orientation = quaternion_from_yaw(0.3)
    assert node._verify_live_source_heading(0.3)(Pose()) == pytest.approx(0.3)
    rgb_queries = [e for e in node.events if e[0] == 'tf' and e[2] == 'rgb_camera']
    assert len(rgb_queries) == 1


def test_rate_limited_tracking_can_turn_beyond_five_degrees_without_new_relative_error(node):
    node.fire = image(yaw=math.radians(40))
    provider = node._verify_live_source_heading(0.0)
    future = Pose(orientation=quaternion_from_yaw(-2.0))
    first = provider(future)
    assert first == pytest.approx(math.radians(5))
    node.actual.orientation = quaternion_from_yaw(first)
    node.now += 0.25
    second = provider(future)
    assert second == pytest.approx(math.radians(8))
    node.actual.orientation = quaternion_from_yaw(second)
    node.now += 0.25
    assert provider(future) == pytest.approx(math.radians(11))
    assert len([e for e in node.events if e[0] == 'tf' and e[2] != 'body']) == 1


def test_image_time_bearing_does_not_double_count_rotation_since_acquisition(node):
    node.fire = image(yaw=0.2)
    node.actual.orientation = quaternion_from_yaw(0.2)
    provider = node._verify_live_source_heading(0.0)
    assert provider(Pose()) == pytest.approx(0.2)


def test_heading_wrap_uses_the_short_rotation(node):
    node.camera.rotation = quaternion_from_yaw(math.radians(179))
    node.body.rotation = quaternion_from_yaw(math.radians(179))
    node.actual.orientation = quaternion_from_yaw(math.radians(-179))
    node.fire = image()
    assert node._verify_live_source_heading(0.0)(Pose()) == pytest.approx(math.radians(179))


def test_fire_alignment_checks_full_bearing_and_passes_tighter_yaw_tolerance(node):
    node.fire = image(yaw=math.radians(8))
    assert not node._verify_fire_heading_ready()
    assert not node.events  # Readiness never waits for TF.
    assert node._verify_align_fire_heading()
    rotations = [e for e in node.events if e[0] == 'rotate']
    assert rotations == [('rotate', pytest.approx(math.radians(8)),
                          pytest.approx(math.radians(2)))]
    assert node._verify_fire_heading_ready()


def test_fire_alignment_compensates_camera_yaw_mount_and_confirms_new_centered_image(node):
    node.mount_yaw = 0.3
    node.camera.rotation = quaternion_from_yaw(node.mount_yaw)
    node.fire = image(yaw=0.1)
    assert node._verify_align_fire_heading()
    assert [e[1] for e in node.events if e[0] == 'rotate'] == pytest.approx([0.1])
    assert node._heading_state()['fire']['bearing'] == pytest.approx(0.4)
    assert abs(node.fire.yaw_correction_rad) < node.config['verify_fire_heading_tolerance']
    assert node.fire.header.stamp.nanosec > 0  # A new image confirms the turn.


def test_tilted_image_error_above_two_degrees_executes_a_smaller_absolute_body_turn(node):
    pitch, error, vertical = math.radians(30), math.radians(2.5), math.radians(40)
    node.camera.rotation = Quaternion(y=math.sin(pitch / 2), w=math.cos(pitch / 2))
    node.fire = image(yaw=error, vertical=vertical)
    target = math.atan2(math.tan(error), math.cos(pitch) + math.sin(pitch) * math.tan(vertical))
    assert 0.0 < target < node.config['verify_fire_heading_tolerance'] < error
    # The fixture levels the camera with the stopped body for its next image.
    assert node._verify_align_fire_heading()
    rotations = [e for e in node.events if e[0] == 'rotate']
    assert rotations[0][1] == pytest.approx(target)
    assert rotations[0][2] == pytest.approx(target / 2)
    assert abs(node.fire.yaw_correction_rad) < node.config['verify_fire_heading_tolerance']


def test_rolled_camera_centers_below_image_fire_without_turning_to_the_ray_azimuth(node):
    roll = math.radians(15)
    cosine, sine = math.cos(roll), math.sin(roll)
    node.camera.rotation = Quaternion(x=math.sin(roll / 2), w=math.cos(roll / 2))
    node.fire = image(yaw=0.1, vertical=-0.2)
    # Fixed physical direction, obtained from the original camera image.
    ray = (1.0, cosine * math.tan(0.1) - sine * math.tan(-0.2),
           sine * math.tan(0.1) + cosine * math.tan(-0.2))
    ray_azimuth = math.atan2(ray[1], ray[0])
    low, high = 0.0, 0.3
    for _ in range(60):
        middle = (low + high) / 2
        projected_left = cosine * (-math.sin(middle) * ray[0]
                                   + math.cos(middle) * ray[1]) + sine * ray[2]
        if projected_left > 0.0:
            low = middle
        else:
            high = middle
    expected_turn = (low + high) / 2

    def actual_new_image(duration):
        node.now += duration
        node.wall += duration
        yaw = yaw_from_quaternion(node.actual.orientation)
        node.body.rotation = quaternion_from_yaw(yaw)
        node.camera.rotation = Quaternion(
            x=math.sin(roll / 2) * math.cos(yaw / 2),
            y=math.sin(roll / 2) * math.sin(yaw / 2),
            z=math.cos(roll / 2) * math.sin(yaw / 2),
            w=math.cos(roll / 2) * math.cos(yaw / 2))
        body_x = math.cos(yaw) * ray[0] + math.sin(yaw) * ray[1]
        body_y = -math.sin(yaw) * ray[0] + math.cos(yaw) * ray[1]
        camera_y = cosine * body_y + sine * ray[2]
        camera_z = -sine * body_y + cosine * ray[2]
        assert body_x > 0.0
        node.fire = image(node.now, yaw=math.atan2(camera_y, body_x),
                          vertical=math.atan2(camera_z, body_x))

    node._stop = SimpleNamespace(wait=actual_new_image)
    assert node._verify_align_fire_heading()
    turns = [e[1] for e in node.events if e[0] == 'rotate']
    assert turns == pytest.approx([expected_turn])
    assert expected_turn > 0.0
    assert abs(expected_turn - ray_azimuth) > math.radians(2)
    assert abs(node.fire.yaw_correction_rad) < 1e-9
    assert node._heading_state()['fire']['bearing'] == pytest.approx(ray_azimuth)


@pytest.mark.parametrize('roll', [math.pi / 2, math.radians(75)])
def test_yaw_unreachable_image_centering_is_unavailable_without_inventing_a_turn(node, roll):
    node.camera.rotation = Quaternion(x=math.sin(roll / 2), w=math.cos(roll / 2))
    node.fire = image(yaw=0.5, vertical=-0.1)
    assert node._verify_image_bearing(node.fire) is not None  # Navigation ray is still meaningful.
    assert node._verify_live_source_heading(0.7)(Pose()) == 0.7
    assert not node._verify_align_fire_heading()
    assert not any(e[0] == 'rotate' for e in node.events)
    assert any('cannot center this image ray' in e[1] for e in node.events if e[0] == 'status')


def test_tracking_never_counters_a_body_that_has_already_reached_the_absolute_target(node):
    node.fire = image(yaw=0.3)
    provider = node._verify_live_source_heading(0.0)
    provider(Pose())
    node.actual.orientation = quaternion_from_yaw(0.3)
    node.now += 0.1
    assert provider(Pose()) == pytest.approx(0.3)


def test_source_handoff_is_logged_even_inside_the_minor_update_throttle(node):
    node.config['verify_fire_heading_grace'] = 0.0
    node.fire = image(yaw=-0.4)
    provider = node._verify_live_source_heading(0.0)
    provider(Pose())
    node.fire = None
    node.now = node.wall = 100.1
    node.smoke = image(node.now, source='smoke', yaw=0.4)
    provider(Pose())
    logs = [e[1] for e in node.events if e[0] == 'status' and 'heading source=' in e[1]]
    assert len(logs) == 2
    assert 'source=fire' in logs[0] and 'source=smoke' in logs[1]


def test_same_source_direction_reversal_is_logged_without_waiting_half_a_second(node):
    node.fire = image(yaw=0.4)
    provider = node._verify_live_source_heading(0.0)
    provider(Pose())
    node.now = node.wall = 100.1
    node.fire = image(node.now, yaw=-0.4)
    provider(Pose())
    node.now = node.wall = 100.2
    assert provider(Pose()) < 0.0
    logs = [e[1] for e in node.events if e[0] == 'status' and 'heading source=' in e[1]]
    assert len(logs) == 2
    assert 'capped correction=-' in logs[1]


def test_no_fire_alignment_does_not_rotate_or_require_tf(node):
    assert node._verify_align_fire_heading()
    assert node._verify_fire_heading_ready()
    assert node.events == []


def test_fire_alignment_with_missing_historical_tf_is_unavailable(node):
    node.fire = image(yaw=0.4)
    node.missing_tf = True
    assert not node._verify_align_fire_heading()
    assert not any(e[0] == 'rotate' for e in node.events)
    assert node._verify_wall_observation_context == {'valid': True}


def test_fire_alignment_has_one_bounded_wall_clock_window(node):
    node.fire = image(yaw=0.4)
    node.on_rotate = lambda: setattr(node, 'wall', node.wall + 15.0)
    assert not node._verify_align_fire_heading()
    assert len([e for e in node.events if e[0] == 'rotate']) == 1


def test_tf_wait_cannot_make_alignment_succeed_after_its_deadline(node):
    node.fire = image()
    node.on_transform = lambda: setattr(node, 'wall', node.wall + 15.0)
    assert not node._verify_align_fire_heading()
    assert not any(e[0] == 'rotate' for e in node.events)


def test_alignment_follows_new_fire_image_instead_of_reusing_a_relative_correction(node):
    node.fire = image(yaw=0.15)

    def changed():
        if len([e for e in node.events if e[0] == 'rotate']) == 1:
            node.fire_world_target = 0.25

    node.on_rotate = changed
    assert node._verify_align_fire_heading()
    assert [e[1] for e in node.events if e[0] == 'rotate'] == pytest.approx([0.15, 0.25])


def test_image_aging_during_transform_cannot_replace_current_heading(node):
    node.fire = image(yaw=0.4)
    node.on_transform = lambda: setattr(node, 'now', node.now + 3.0)
    assert node._verify_live_source_heading(0.7)(Pose()) == 0.7


def test_unconfirmed_fire_cannot_override_confirmed_smoke(node):
    node.fire = image(yaw=-0.4)
    node.fire.status = 1
    node.smoke = image(source='smoke', yaw=0.4)
    assert node._verify_live_source_heading(0.0)(Pose()) == pytest.approx(math.radians(5))
    assert node.events[0][2] == 'rgb_camera'
