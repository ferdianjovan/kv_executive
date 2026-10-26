"""Verify navigation, guard recovery, and fire report validation."""

from copy import deepcopy
import math
import threading
from types import SimpleNamespace

from geometry_msgs.msg import Pose, PoseStamped, Transform
from kv_executive.mission_geometry import quaternion_from_yaw, transform_pose
import kv_executive.mission_navigation as navigation
from kv_executive.mission_navigation import VerifyNavigationMixin
from kv_executive.verification import VerificationObservationLost
from nav_msgs.msg import Odometry, Path
import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


def pose(x=0.0, y=0.0, z=6.0, yaw=0.0):
    value = Pose()
    value.position.x = float(x)
    value.position.y = float(y)
    value.position.z = float(z)
    value.orientation = quaternion_from_yaw(yaw)
    return value


def header(now=100.0, frame='px4_odom'):
    result = Header(frame_id=frame)
    result.stamp.sec = int(now)
    result.stamp.nanosec = round((now % 1.0) * 1e9)
    return result


class Harness(VerifyNavigationMixin):
    def __init__(self):
        self.now = 100.0
        self.config = {
            'verify_motion_speed': 0.5, 'verify_minimum_altitude': 2.0,
            'verify_planning_frame': 'map', 'planning_timeout': 3.0,
            'position_tolerance': 0.3, 'setpoint_rate': 20.0,
            'path_turn_step_duration': 5.0, 'global_position_timeout': 2.0,
            'verify_future_tolerance': 0.25, 'verify_observation_timeout': 2.0,
            'telemetry_timeout': 2.0, 'local_frame': 'px4_odom',
            'fire_exclusion_radius': 15.0,
            'verify_yaw_tolerance': math.radians(5.0),
        }
        self.state = SimpleNamespace(mode='AUTO.LOITER', armed=True)
        self.odometry = Odometry(header=header())
        self.odometry.pose.pose = pose()
        self.events = []
        self._data_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._motion_guard = None
        self._recovering = False
        self._owns_flight = True
        self._target_planning_client = object()
        self.conversion = Transform()
        self.conversion.rotation.w = 1.0
        self.path_vertices = [pose(), pose(1.0)]
        self.response_success = True
        self.response_frame = 'map'
        self.on_plan = lambda: None
        self.on_motion = lambda: None
        self.on_warmup = lambda: None
        self.recovery_fails = False
        self._global_position = NavSatFix(
            header=header(frame='gps'), status=NavSatStatus(status=NavSatStatus.STATUS_FIX),
            latitude=51.5, longitude=-0.12, altitude=50.0,
        )
        self._global_position_received = self.now
        self._fire_sites = []
        self._site_sequence = 0
        self.current_fire = None
        self._verify = SimpleNamespace(stable_fire=lambda: self.current_fire)
        self._fire_report_publisher = SimpleNamespace(
            publish=lambda message: self.events.append(('report', deepcopy(message))))

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(self.now * 1e9), to_msg=lambda: header(self.now).stamp))

    def _check_running(self):
        return None

    def _snapshot(self, expected_mode=None, require_armed=False):
        from kv_executive.mission_executive import MissionError
        if expected_mode and self.state.mode != expected_mode:
            raise MissionError('Pilot changed flight mode')
        return deepcopy(self.state), deepcopy(self.odometry)

    def _path_frame_transform(self, frame):
        assert frame == 'map'
        return deepcopy(self.conversion)

    def _call(self, client, request, description, timeout=None):
        assert client is self._target_planning_client
        self.events.append(('request', deepcopy(request)))
        self.on_plan()
        path = Path(header=header(frame=self.response_frame))
        for vertex in self.path_vertices:
            path.poses.append(PoseStamped(header=deepcopy(path.header), pose=deepcopy(vertex)))
        return SimpleNamespace(success=self.response_success, path=path, message='test')

    def _enter_offboard(self, current, allowed_modes):
        assert allowed_modes == ('AUTO.LOITER',)
        self.events.append(('offboard',))
        self.on_warmup()
        self.state.mode = 'OFFBOARD'
        if self._motion_guard:
            self._motion_guard()

    def _ramp(self, start, target, duration, hold_duration=None):
        if self._motion_guard:
            self._motion_guard()
        self.events.append(('ramp', deepcopy(start), deepcopy(target), duration, hold_duration))
        self.on_motion()
        self._snapshot('OFFBOARD', require_armed=True)
        if self._motion_guard:
            self._motion_guard()
        self.odometry.pose.pose = deepcopy(target)

    def _set_target(self, target):
        self.events.append(('target', deepcopy(target)))

    def _wait_at_pose(self, target, hold_duration=None):
        self.events.append(('hold', hold_duration))
        self.on_motion()
        if self._motion_guard:
            self._motion_guard()

    def _loiter(self):
        assert self.state.mode == 'OFFBOARD'
        self.events.append(('loiter',))
        self.state.mode = 'AUTO.LOITER'

    def _recover_to_loiter(self):
        assert self._recovering
        assert self._motion_guard is None
        self.events.append(('recovery',))
        if self.recovery_fails:
            return '; AUTO.LOITER recovery failed'
        if self.state.mode == 'OFFBOARD':
            self.state.mode = 'AUTO.LOITER'
        return '; AUTO.LOITER confirmed'

    def _report(self, message):
        self.events.append(('status', message))


@pytest.fixture
def executive(monkeypatch):
    harness = Harness()
    monkeypatch.setattr(navigation.time, 'monotonic', lambda: harness.now)
    return harness


def test_target_converts_between_map_and_px4_and_retains_planned_corner(executive):
    executive.conversion.translation.x = 10.0
    executive.conversion.translation.y = 20.0
    executive.conversion.translation.z = 3.0
    executive.conversion.rotation = quaternion_from_yaw(math.pi / 2)
    executive.path_vertices = [pose(0, 0, 3), pose(1, 1, 3), pose(5, 0, 3)]
    executive.odometry.pose.pose = pose(10, 20, 6)
    target = pose(10, 25, 6, yaw=0.4)
    executive._verify_plan_to(target)
    request = next(e[1] for e in executive.events if e[0] == 'request')
    assert request.target.header.frame_id == 'map'
    assert request.target.pose.position.x == pytest.approx(5.0)
    assert request.target.pose.position.y == pytest.approx(0.0, abs=1e-7)
    corner = transform_pose(executive.path_vertices[1], executive.conversion)
    ramps = [e for e in executive.events if e[0] == 'ramp']
    assert any(math.dist(
        (e[2].position.x, e[2].position.y), (corner.position.x, corner.position.y)) < 1e-6
        for e in ramps)
    assert executive.odometry.pose.pose.position.y == pytest.approx(25.0)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


@pytest.mark.parametrize('invalid', [
    'failure', 'frame', 'empty', 'quaternion', 'endpoint', 'start', 'low',
])
def test_invalid_plans_never_enter_offboard(executive, invalid):
    from kv_executive.mission_executive import MissionError
    if invalid == 'failure':
        executive.response_success = False
    elif invalid == 'frame':
        executive.response_frame = 'other'
    elif invalid == 'empty':
        executive.path_vertices = []
    elif invalid == 'quaternion':
        executive.path_vertices[1].orientation.w = 0.0
    elif invalid == 'endpoint':
        executive.path_vertices[-1].position.x = 0.5
    elif invalid == 'start':
        executive.path_vertices[0].position.y = 1.0
    elif invalid == 'low':
        executive.path_vertices.insert(1, pose(0.5, 0.0, 1.0))
    with pytest.raises(MissionError):
        executive._verify_plan_to(pose(1))
    assert not any(e[0] == 'offboard' for e in executive.events)


def test_start_drift_during_offboard_warmup_stops_before_translation(executive):
    from kv_executive.mission_executive import MissionError
    executive.on_warmup = lambda: setattr(executive.odometry.pose.pose.position, 'y', 1.0)
    with pytest.raises(MissionError, match='warmup'):
        executive._verify_plan_to(pose(1))
    assert not any(e[0] == 'ramp' for e in executive.events)


def test_perception_loss_during_path_recovers_loiter_then_propagates(executive):
    visible = True

    def guard():
        if not visible:
            raise VerificationObservationLost('fire lost')

    def lose():
        nonlocal visible
        visible = False

    executive.on_motion = lose
    with pytest.raises(VerificationObservationLost):
        executive._verify_plan_to(pose(1), guard=guard)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None
    assert not executive._recovering
    assert len([e for e in executive.events if e[0] == 'recovery']) == 1


def test_loiter_failure_on_observation_loss_aborts_instead_of_reacquiring(executive):
    from kv_executive.mission_executive import MissionError
    executive.state.mode = 'OFFBOARD'
    executive.recovery_fails = True

    def guard():
        raise VerificationObservationLost('smoke lost')

    with pytest.raises(MissionError, match='LOITER recovery failed'):
        executive._verify_hold(3, guard=guard)
    assert not executive._recovering
    assert executive._motion_guard is None


@pytest.mark.parametrize('operation', ['rotate', 'hold', 'plan'])
def test_helpers_never_take_control_from_pilot_mode(executive, operation):
    from kv_executive.mission_executive import MissionError
    executive.state.mode = 'POSCTL'
    with pytest.raises(MissionError, match='cannot take control'):
        if operation == 'rotate':
            executive._verify_rotate(0.5)
        elif operation == 'hold':
            executive._verify_hold(3.0)
        else:
            executive._verify_plan_to(pose(1))
    assert executive.state.mode == 'POSCTL'
    assert not any(e[0] == 'offboard' for e in executive.events)


def test_rotate_changes_only_yaw_and_hold_has_requested_duration(executive):
    executive._verify_rotate(math.pi / 3, hold_duration=3.0)
    ramp = next(e for e in executive.events if e[0] == 'ramp')
    assert ramp[1].position == ramp[2].position
    assert ramp[2].orientation.x == ramp[2].orientation.y == 0.0
    assert ramp[4] == 3.0
    executive._verify_hold(3.0)
    assert ('hold', 3.0) in executive.events
    assert executive.state.mode == 'AUTO.LOITER'


def fire(executive, **changes):
    result = SimpleNamespace(
        header=header(executive.now, 'thermal'), detected=True, status=2, CONFIRMED=2,
        source_count=2, track_id=7,
        yaw_correction_rad=0.0, vertical_correction_rad=0.0,
    )
    for name, value in changes.items():
        setattr(result, name, value)
    executive.current_fire = result
    return result


def test_report_contains_local_map_and_real_global_observation_and_count(executive):
    executive.conversion.translation.x = 10.0
    executive.odometry.pose.pose.position.x = 11.0
    report = executive._verify_record_fire(fire(executive), 0.4)
    assert report.label == 'Fire source found'
    assert report.local_pose.header.frame_id == 'px4_odom'
    assert report.local_pose.pose.position.x == 11.0
    assert report.planning_pose.header.frame_id == 'map'
    assert report.planning_pose.pose.position.x == 1.0
    assert report.global_position.latitude == 51.5
    assert report.source_count == 2
    assert report.track_id == 7
    assert report.exclusion_radius == 15.0
    assert report.site_id == 1
    assert len(executive._fire_sites) == 1
    assert executive._fire_sites[0] is not report


@pytest.mark.parametrize('invalid', [
    'missing', 'no_fix', 'stale_receipt', 'stale_source', 'future', 'nan', 'latitude',
    'longitude', 'fire_source', 'odometry_source', 'zero_count',
])
def test_report_rejects_invalid_gps_or_stale_observations(executive, invalid):
    from kv_executive.mission_executive import MissionError
    observation = fire(executive)
    if invalid == 'missing':
        executive._global_position = None
    elif invalid == 'no_fix':
        executive._global_position.status.status = NavSatStatus.STATUS_NO_FIX
    elif invalid == 'stale_receipt':
        executive._global_position_received -= 3.0
    elif invalid == 'stale_source':
        executive._global_position.header.stamp.sec -= 3
    elif invalid == 'future':
        executive._global_position.header.stamp.sec += 1
    elif invalid == 'nan':
        executive._global_position.latitude = math.nan
    elif invalid == 'latitude':
        executive._global_position.latitude = 91.0
    elif invalid == 'longitude':
        executive._global_position.longitude = -181.0
    elif invalid == 'fire_source':
        observation.header.stamp.sec -= 3
    elif invalid == 'odometry_source':
        executive.odometry.header.stamp.sec -= 3
    elif invalid == 'zero_count':
        observation.source_count = 0
    with pytest.raises(MissionError):
        executive._verify_record_fire(observation, 0.0)
    assert not executive._fire_sites
    assert not any(e[0] == 'report' for e in executive.events)


@pytest.mark.parametrize('change', ['lost', 'track', 'frame', 'uncentered'])
def test_report_rechecks_live_fire_after_transform_lookup(executive, change):
    observation = fire(executive)
    current = deepcopy(observation)
    if change == 'lost':
        current = None
    elif change == 'track':
        current.track_id += 1
    elif change == 'frame':
        current.header.frame_id = 'other_camera'
    else:
        current.vertical_correction_rad = 0.2

    def transform(frame):
        executive.current_fire = current
        return deepcopy(executive.conversion)

    executive._path_frame_transform = transform
    with pytest.raises(VerificationObservationLost, match='changed or lost centering'):
        executive._verify_record_fire(observation, 0.0)
    assert not executive._fire_sites
    assert not any(event[0] == 'report' for event in executive.events)
