"""Verify navigation, guard recovery, and fire report validation."""

from copy import deepcopy
import math
import threading
from types import SimpleNamespace

from geometry_msgs.msg import Point, Pose, PoseStamped, Transform
from kv_executive.mission_geometry import (
    quaternion_from_yaw, transform_pose, yaw_from_quaternion,
)
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
        self.on_wait = lambda: None
        self._stop = SimpleNamespace(wait=self.wait)
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

    def wait(self, duration):
        self.now += duration
        self.events.append(('wait', duration, self.state.mode))
        self.on_wait()
        return False

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

    def _wait_at_pose(self, target, hold_duration=None, **kwargs):
        self.events.append(('hold', hold_duration))
        self.on_motion()
        if self._motion_guard:
            self._motion_guard()
        if kwargs.get('pose_validator') is not None:
            kwargs['pose_validator'](self.odometry.pose.pose)

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


@pytest.mark.parametrize('direction', [-1.0, 1.0])
def test_height_guided_path_checks_height_convergence_before_loiter(executive, direction):
    target = pose(1.0, z=6.0 + direction * 0.5)
    executive.path_vertices = [pose(), deepcopy(target)]
    original_wait = executive._wait_at_pose

    def wait_height(target_pose, **kwargs):
        assert executive.state.mode == 'OFFBOARD'
        assert kwargs['height_tolerance'] == 0.05
        executive.events.append(('height_convergence', deepcopy(target_pose)))
        original_wait(target_pose, **kwargs)

    executive._wait_at_pose = wait_height
    assert executive._verify_plan_to(target, height_tolerance=0.05)
    events = [event[0] for event in executive.events]
    assert events.index('height_convergence') < events.index('loiter')


def test_height_convergence_rechecks_corridor_and_recovers_if_it_becomes_unsafe(executive):
    target = pose(1.0, z=5.5)
    executive.path_vertices = [pose(), deepcopy(target)]
    waiting = [False]
    original_wait = executive._wait_at_pose

    def wait_height(target_pose, **kwargs):
        waiting[0] = True
        original_wait(target_pose, **kwargs)

    executive._wait_at_pose = wait_height
    with pytest.raises(VerificationObservationLost, match='Height convergence corridor'):
        executive._verify_plan_to(
            target, height_tolerance=0.05, path_validator=lambda path: not waiting[0])
    assert executive.state.mode == 'AUTO.LOITER'
    assert any(event[0] == 'recovery' for event in executive.events)


def test_height_convergence_retains_the_live_heading_selected_during_flight(executive):
    target = pose(1.0, z=5.5)
    executive.path_vertices = [pose(), deepcopy(target)]
    heading = [0.0]
    original_wait = executive._wait_at_pose

    def fire_moves_in_image():
        heading[0] = 0.4

    def wait_height(target_pose, **kwargs):
        assert yaw_from_quaternion(target_pose.orientation) == pytest.approx(0.4)
        assert yaw_from_quaternion(executive.odometry.pose.pose.orientation) == pytest.approx(0.4)
        original_wait(target_pose, **kwargs)

    executive.on_motion = fire_moves_in_image
    executive._wait_at_pose = wait_height
    assert executive._verify_plan_to(
        target, height_tolerance=0.05, heading_provider=lambda current: heading[0])


@pytest.mark.parametrize('tolerance', [0.0, -0.1, math.inf, math.nan, True])
def test_invalid_height_convergence_setting_never_starts_flight(executive, tolerance):
    from kv_executive.mission_executive import MissionError
    with pytest.raises(MissionError, match='Height convergence tolerance'):
        executive._verify_plan_to(pose(1.0), height_tolerance=tolerance)
    assert not any(event[0] in {'request', 'offboard'} for event in executive.events)


def test_inspection_planner_hook_preserves_transform_and_flight_validation(executive):
    from kv_executive.srv import PlanInspectionPath
    custom_client = object()
    executive.conversion.translation.x = 10.0
    executive.odometry.pose.pose = pose(10.0)
    executive.path_vertices = [pose(), pose(1.0)]
    original_call = executive._call
    built = []

    def build(target, map_from_local):
        built.append((deepcopy(target), deepcopy(map_from_local)))
        request = PlanInspectionPath.Request(target=target, minimum_agl=5.0)
        return request

    def call(client, request, description, timeout=None):
        assert client is custom_client
        assert isinstance(request, PlanInspectionPath.Request)
        return original_call(executive._target_planning_client, request, description, timeout)

    executive._call = call
    validations = []
    assert executive._verify_plan_to(
        pose(11.0), planning_client=custom_client, request_builder=build,
        path_validator=lambda path: validations.append(deepcopy(path)) or True)
    assert built[0][0].pose.position.x == 1.0
    assert built[0][1].translation.x == -10.0
    assert len(validations) >= 2
    assert executive.state.mode == 'AUTO.LOITER'


@pytest.mark.parametrize('code', [2, 3])
def test_typed_inspection_request_or_transform_error_is_not_optional(executive, code):
    from kv_executive.mission_executive import MissionError
    executive._call = lambda *args, **kwargs: SimpleNamespace(
        success=False, error_code=code, message='native transform or request failed')
    with pytest.raises(MissionError, match='native transform or request failed'):
        executive._verify_plan_to(pose(1.0), allow_plan_failure=True)
    assert not any(event[0] == 'offboard' for event in executive.events)


@pytest.mark.parametrize('allow_plan_failure', [False, True])
def test_invalid_inspection_start_is_distinct_and_never_enters_offboard(
        executive, allow_plan_failure):
    from kv_executive.verification import VerificationStartUnavailable
    executive._call = lambda *args, **kwargs: SimpleNamespace(
        success=False, error_code=4, message='current inflated body contains unknown cells')

    with pytest.raises(VerificationStartUnavailable, match='current inflated body'):
        executive._verify_plan_to(pose(1.0), allow_plan_failure=allow_plan_failure)

    assert not any(event[0] in {'offboard', 'ramp'} for event in executive.events)
    assert executive.state.mode == 'AUTO.LOITER'


def test_invalid_inspection_goal_can_select_another_candidate_without_flight(executive):
    executive._call = lambda *args, **kwargs: SimpleNamespace(
        success=False, error_code=5, message='candidate goal contains occupied cells')

    assert executive._verify_plan_to(pose(1.0), allow_plan_failure=True) is False

    assert not any(event[0] in {'offboard', 'ramp'} for event in executive.events)
    assert any('candidate goal contains occupied cells' in event[1]
               for event in executive.events if event[0] == 'status')


def test_exhausted_clearance_recovery_is_distinct_from_invalid_body_clearance(executive):
    from kv_executive.verification import (
        VerificationRecoveryUnavailable, VerificationStartUnavailable,
    )
    executive._call = lambda *args, **kwargs: SimpleNamespace(
        success=False, error_code=6, message='all bounded climb and side escapes blocked')

    with pytest.raises(VerificationRecoveryUnavailable, match='bounded climb') as caught:
        executive._verify_plan_to(pose(1.0), allow_plan_failure=True)

    assert not isinstance(caught.value, VerificationStartUnavailable)
    assert not any(event[0] in {'offboard', 'ramp'} for event in executive.events)


def test_path_refreshes_image_heading_during_translation_without_a_fire_point(executive):
    executive.path_vertices = [pose(), pose(1.0), pose(1.0, 1.0)]
    headings = []

    def heading(pose):
        value = 0.1 if pose.position.x < 0.5 else -0.2
        headings.append(value)
        return value

    assert executive._verify_plan_to(pose(1.0, 1.0), heading_provider=heading)

    ramps = [event for event in executive.events if event[0] == 'ramp']
    assert any(yaw_from_quaternion(event[2].orientation) == pytest.approx(0.1) for event in ramps)
    assert any(yaw_from_quaternion(event[2].orientation) == pytest.approx(-0.2) for event in ramps)
    assert len(headings) > 5
    assert executive.state.mode == 'AUTO.LOITER'


@pytest.mark.parametrize('failure_stage', ['arrival', 'stopping'])
def test_actual_arrival_clearance_is_required_before_and_after_loiter(
        executive, failure_stage):
    executive.odometry.pose.pose = pose(z=6.0)
    executive.path_vertices = [pose(z=6.0), pose(z=5.2)]
    ramp = executive._ramp
    loiter = executive._loiter
    singleton_checks = []

    def undershoot(start, target, duration, hold_duration=None):
        ramp(start, target, duration, hold_duration=hold_duration)
        if failure_stage == 'arrival' and target.position.z == 5.2:
            executive.odometry.pose.pose.position.z = 4.95

    def stop_then_lose_clearance():
        loiter()
        if failure_stage == 'stopping':
            executive.odometry.pose.pose.position.z = 4.95

    def safe_actual_height(path):
        if len(path.poses) == 1:
            singleton_checks.append(path.poses[0].pose.position.z)
        return all(vertex.pose.position.z >= 5.0 for vertex in path.poses)

    executive._ramp = undershoot
    executive._loiter = stop_then_lose_clearance

    with pytest.raises(VerificationObservationLost, match='arrival'):
        executive._verify_plan_to(pose(z=5.2), path_validator=safe_actual_height)

    assert singleton_checks[-1] == 4.95
    assert abs(5.2 - executive.odometry.pose.pose.position.z) < (
        executive.config['position_tolerance'])
    assert bool([event for event in executive.events if event[0] == 'loiter']) == (
        failure_stage == 'stopping')
    assert any(event[0] == 'recovery' for event in executive.events)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


def test_upward_warmup_drift_does_not_command_descent_to_inspection_plan_start(executive):
    from kv_executive.srv import PlanInspectionPath
    executive.odometry.pose.pose = pose(z=3.0)
    executive.path_vertices = [pose(z=3.0), pose(z=5.0)]
    executive.on_warmup = lambda: setattr(executive.odometry.pose.pose.position, 'z', 3.1)
    checked = []

    def validate(path):
        heights = [vertex.pose.position.z for vertex in path.poses]
        checked.append(heights)
        return all(end >= start for start, end in zip(heights, heights[1:]))

    def build(target, conversion):
        return PlanInspectionPath.Request(target=target, allow_initial_climb=True)

    assert executive._verify_plan_to(
        pose(z=5.0), request_builder=build, path_validator=validate)
    assert checked[0] == [3.0, 5.0]
    assert checked[1] == [3.1, 5.0]
    for event in executive.events:
        if event[0] == 'ramp':
            assert event[2].position.z >= event[1].position.z


def test_periodic_climb_validation_uses_the_next_target_after_actual_pose_clears_floor(executive):
    from kv_executive.srv import PlanInspectionPath
    executive.odometry.pose.pose = pose(z=4.99)
    executive.path_vertices = [pose(z=4.99), pose(z=5.1), pose(1.0, z=5.1)]
    ramp = executive._ramp
    checked = []

    def arrive_slightly_ahead_of_the_old_command(start, target, duration, hold_duration=None):
        ramp(start, target, duration, hold_duration=hold_duration)
        executive.odometry.pose.pose.position.z = max(5.02, target.position.z)
        executive.now += 1.1  # Force the real periodic corridor validation.

    def validate(path):
        heights = [vertex.pose.position.z for vertex in path.poses]
        checked.append(heights)
        # Initial planning/warmup can include the explicitly permitted climb
        # from 4.99 m. Once actual pose passes the 5 m floor, a stale previous target
        # below it must not be inserted back into the certified remaining path.
        return len(checked) <= 2 or min(heights) >= 5.02 - 1e-9

    executive._ramp = arrive_slightly_ahead_of_the_old_command

    assert executive._verify_plan_to(
        pose(1.0, z=5.1), path_validator=validate,
        request_builder=lambda target, _: PlanInspectionPath.Request(
            target=target, allow_initial_climb=True))

    assert checked[2][0] == pytest.approx(5.02)
    assert checked[2][1] == pytest.approx(5.034)
    assert all(min(heights) >= 5.02 - 1e-9 for heights in checked[2:])
    assert any(event[0] == 'ramp' and event[2] == pose(z=5.1) for event in executive.events)
    assert executive.odometry.pose.pose == pose(1.0, z=5.1)
    assert executive.state.mode == 'AUTO.LOITER'


@pytest.mark.parametrize('invalid', [
    'failure', 'frame', 'empty', 'quaternion', 'endpoint', 'start', 'low',
])
@pytest.mark.parametrize('allow_plan_failure', [False, True])
def test_invalid_plans_never_enter_offboard(executive, invalid, allow_plan_failure):
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
    if invalid == 'failure' and allow_plan_failure:
        assert executive._verify_plan_to(pose(1), allow_plan_failure=True) is False
    else:
        with pytest.raises(MissionError):
            executive._verify_plan_to(pose(1), allow_plan_failure=allow_plan_failure)
    assert not any(e[0] == 'offboard' for e in executive.events)


def test_optional_rejected_candidate_reports_reason_without_starting_flight(executive):
    executive.response_success = False

    assert executive._verify_plan_to(pose(1), allow_plan_failure=True) is False

    assert ('status', 'Verify: candidate unavailable: test') in executive.events
    assert not any(event[0] in {'offboard', 'ramp'} for event in executive.events)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


@pytest.mark.parametrize('allow_plan_failure', [False, True])
def test_completed_plan_returns_true_after_arrival_and_loiter(executive, allow_plan_failure):
    target = pose(1)

    assert executive._verify_plan_to(target, allow_plan_failure=allow_plan_failure) is True

    assert executive.odometry.pose.pose == target
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive.events[-1] == ('loiter',)


@pytest.mark.parametrize('failure', [
    'service', 'transform', 'flight', 'cancel', 'abort', 'return',
])
def test_optional_candidates_do_not_suppress_operation_or_stop_failures(executive, failure):
    from kv_executive.mission_executive import (
        MissionAbortRequested, MissionCanceled, MissionError, MissionStateRequested,
    )
    error = {
        'service': MissionError('planning service timed out'),
        'transform': MissionError('transform unavailable'),
        'flight': MissionError('flight telemetry lost'),
        'cancel': MissionCanceled('operator cancellation'),
        'abort': MissionAbortRequested('battery alert'),
        'return': MissionStateRequested('Return'),
    }[failure]

    def fail(*args, **kwargs):
        raise error

    if failure == 'transform':
        executive._path_frame_transform = fail
    elif failure == 'service':
        executive._call = fail
    else:
        executive.on_motion = fail

    with pytest.raises(type(error)) as stopped:
        executive._verify_plan_to(pose(1), allow_plan_failure=True)

    assert stopped.value is error
    assert executive._motion_guard is None
    assert not any(event[0] == 'status' for event in executive.events)


def test_start_drift_during_offboard_warmup_stops_before_translation(executive):
    from kv_executive.mission_executive import MissionError
    executive.on_warmup = lambda: setattr(executive.odometry.pose.pose.position, 'y', 1.0)
    with pytest.raises(MissionError, match='warmup'):
        executive._verify_plan_to(pose(1))
    assert not any(e[0] == 'ramp' for e in executive.events)


def test_inspection_path_faces_target_at_every_translation_setpoint(executive):
    executive.path_vertices = [pose(), pose(0, 1), pose(1, 0)]
    target = Point(x=10.0, y=3.0, z=0.0)
    assert executive._verify_plan_to(pose(1), look_at_point=target) is True
    ramps = [event for event in executive.events if event[0] == 'ramp']
    assert ramps
    for event in ramps:
        endpoint = event[2]
        expected = math.atan2(target.y - endpoint.position.y, target.x - endpoint.position.x)
        assert yaw_from_quaternion(endpoint.orientation) == pytest.approx(expected)
        assert endpoint.orientation.x == endpoint.orientation.y == 0.0


@pytest.mark.parametrize('reject_after_warmup', [False, True])
def test_inspection_corridor_rejection_stops_before_translation(executive, reject_after_warmup):
    calls = []

    def validate(path):
        assert path.header.frame_id == 'px4_odom'
        assert path.poses
        calls.append(path)
        return reject_after_warmup and len(calls) == 1

    assert executive._verify_plan_to(pose(1), path_validator=validate) is False
    assert len(calls) == (2 if reject_after_warmup else 1)
    assert not any(event[0] == 'ramp' for event in executive.events)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


@pytest.mark.parametrize('phase', ['planning', 'warmup'])
def test_inspection_validates_actual_position_connector_before_translation(executive, phase):
    calls = []

    def drift():
        # This remains within start-position tolerance, but the connector from
        # the actual pose to the original path crosses an unsafe side region.
        executive.odometry.pose.pose.position.y = 0.2

    if phase == 'planning':
        executive.on_plan = drift
    else:
        executive.on_warmup = drift

    def validate(path):
        calls.append(deepcopy(path))
        return all(stamped.pose.position.y <= 0.1 for stamped in path.poses)

    assert executive._verify_plan_to(pose(1), path_validator=validate) is False

    assert len(calls) == (1 if phase == 'planning' else 2)
    assert calls[-1].poses[0].pose.position.y == pytest.approx(0.2)
    assert calls[-1].poses[1].pose.position.y == 0.0
    assert calls[-1].poses[-1].pose.position.x == 1.0
    assert not any(event[0] == 'ramp' for event in executive.events)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


def test_inspection_revalidates_remaining_corridor_during_flight(executive):
    calls = []

    def validate(path):
        calls.append(path)
        return len(calls) < 3

    executive.on_motion = lambda: setattr(executive, 'now', executive.now + 1.1)
    with pytest.raises(VerificationObservationLost, match='corridor'):
        executive._verify_plan_to(pose(1), path_validator=validate)
    assert len(calls) == 3
    assert calls[-1].poses[0].pose.position.x < 1.0
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None
    assert any(event[0] == 'recovery' for event in executive.events)


def test_inspection_periodic_validation_includes_actual_and_next_commanded_target(executive):
    calls = []
    ramp = executive._ramp

    def drift_after_first_translation(*args, **kwargs):
        ramp(*args, **kwargs)
        if args[1].position.x > 0.0:
            executive.odometry.pose.pose.position.y = 0.2
            executive.now += 1.1

    def validate(path):
        calls.append(deepcopy(path))
        return all(stamped.pose.position.y <= 0.1 for stamped in path.poses)

    executive._ramp = drift_after_first_translation
    with pytest.raises(VerificationObservationLost, match='corridor'):
        executive._verify_plan_to(pose(1), path_validator=validate)

    assert len(calls) == 3
    actual, commanded = calls[-1].poses[:2]
    assert actual.pose.position.y == pytest.approx(0.2)
    assert commanded.pose.position.y == 0.0
    assert commanded.pose.position.x > actual.pose.position.x
    assert calls[-1].poses[-1].pose.position.x == 1.0
    translating = [event for event in executive.events
                   if event[0] == 'ramp' and event[2].position.x > 0.0]
    assert len(translating) == 1
    assert executive.state.mode == 'AUTO.LOITER'
    assert any(event[0] == 'recovery' for event in executive.events)
    assert executive._motion_guard is None


@pytest.mark.parametrize('allow_plan_failure', [False, True])
def test_perception_loss_during_path_recovers_loiter_then_propagates(
        executive, allow_plan_failure):
    visible = True

    def guard():
        if not visible:
            raise VerificationObservationLost('fire lost')

    def lose():
        nonlocal visible
        visible = False

    executive.on_motion = lose
    with pytest.raises(VerificationObservationLost):
        executive._verify_plan_to(pose(1), guard=guard, allow_plan_failure=allow_plan_failure)
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
    started = executive.now
    executive._verify_hold(3.0)
    assert executive.now - started == pytest.approx(3.0)
    assert sum(event[1] for event in executive.events if event[0] == 'wait') == pytest.approx(3.0)
    assert executive.state.mode == 'AUTO.LOITER'


def test_verify_hold_observes_stationary_loiter_without_mode_or_setpoint_churn(executive):
    executive.odometry.pose.pose = pose(12.0, -4.0, 8.0, yaw=0.7)
    original = deepcopy(executive.odometry.pose.pose)
    started = executive.now

    executive._verify_hold(5.0)

    assert executive.now - started == pytest.approx(5.0)
    assert executive.odometry.pose.pose == original
    assert all(event[0] == 'wait' for event in executive.events)
    assert all(event[2] == 'AUTO.LOITER' for event in executive.events)
    assert max(event[1] for event in executive.events) <= 0.05
    assert executive._motion_guard is None


def test_verify_hold_counts_full_duration_only_after_loiter_acknowledgment(executive):
    executive.state.mode = 'OFFBOARD'
    loiter = executive._loiter
    started = executive.now

    def delayed_loiter():
        executive.now += 2.0
        loiter()

    executive._loiter = delayed_loiter
    executive._verify_hold(5.0)

    assert executive.now - started == pytest.approx(7.0)
    waits = [event for event in executive.events if event[0] == 'wait']
    assert sum(event[1] for event in waits) == pytest.approx(5.0)
    assert all(event[2] == 'AUTO.LOITER' for event in waits)
    assert executive.events[0] == ('loiter',)
    assert not any(event[0] in {'offboard', 'ramp', 'target'} for event in executive.events)
    assert executive.state.mode == 'AUTO.LOITER'


def test_verify_hold_perception_guard_interrupts_without_entering_offboard(executive):
    started = executive.now

    def guard():
        if executive.now - started >= 0.2:
            raise VerificationObservationLost('Persistent fire found')

    with pytest.raises(VerificationObservationLost, match='fire found'):
        executive._verify_hold(5.0, guard=guard)

    assert 0.2 <= executive.now - started < 0.3
    assert not any(event[0] in {'offboard', 'ramp', 'target'} for event in executive.events)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None
    assert not executive._recovering


def test_verify_hold_cancel_interrupts_the_passive_wait(executive):
    from kv_executive.mission_executive import MissionCanceled
    started = executive.now

    def check_running():
        if executive.now - started >= 0.15:
            raise MissionCanceled('Mission canceled by client')

    executive._check_running = check_running
    with pytest.raises(MissionCanceled, match='canceled by client'):
        executive._verify_hold(5.0)

    assert executive.now - started < 0.25
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


def test_verify_hold_detects_pilot_mode_change_without_taking_control_back(executive):
    from kv_executive.mission_executive import MissionError
    started = executive.now

    def pilot_changes_mode():
        if executive.now - started >= 0.15:
            executive.state.mode = 'POSCTL'

    executive.on_wait = pilot_changes_mode
    with pytest.raises(MissionError, match='Pilot changed flight mode'):
        executive._verify_hold(5.0)

    assert executive.state.mode == 'POSCTL'
    assert not any(event[0] in {'offboard', 'loiter', 'ramp'} for event in executive.events)
    assert executive._motion_guard is None


def test_verify_hold_keeps_checking_telemetry_freshness(executive):
    from kv_executive.mission_executive import MissionError
    snapshot = executive._snapshot
    started = executive.now

    def telemetry_expires(*args, **kwargs):
        if executive.now - started >= 0.15:
            raise MissionError('Stale local odometry')
        return snapshot(*args, **kwargs)

    executive._snapshot = telemetry_expires
    with pytest.raises(MissionError, match='Stale local odometry'):
        executive._verify_hold(5.0)

    assert executive.now - started < 0.25
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


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


def test_prepared_fire_report_has_no_publication_or_exclusion_until_commit(executive):
    report = executive._verify_record_fire(fire(executive), 0.4, publish=False)

    assert report.site_id == 0
    assert executive._site_sequence == 0
    assert not executive._fire_sites
    assert not any(event[0] == 'report' for event in executive.events)

    executive._verify_publish_fire_report(report)
    assert report.site_id == 1
    assert len(executive._fire_sites) == 1
    assert len([event for event in executive.events if event[0] == 'report']) == 1


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


def test_committed_fire_path_limit_counts_detour_not_endpoint_distance(executive):
    executive.path_vertices = [pose(), pose(0.0, 8.0), pose(1.0)]
    assert executive._verify_plan_to(pose(1.0), maximum_path_length=12.0) is False
    assert not any(e[0] == 'offboard' for e in executive.events)


def test_committed_fire_motion_timeout_recovers_to_loiter(executive):
    def stall():
        executive.now += 61.0
    executive.on_motion = stall
    with pytest.raises(VerificationObservationLost, match='movement time limit'):
        executive._verify_plan_to(pose(1.0), maximum_path_length=12.0, motion_timeout=60.0)
    assert executive.state.mode == 'AUTO.LOITER'
    assert executive._motion_guard is None


def test_committed_fire_timeout_starts_after_offboard_preparation(executive):
    executive.on_warmup = lambda: setattr(executive, 'now', executive.now + 70.0)
    assert executive._verify_plan_to(pose(1.0), maximum_path_length=12.0, motion_timeout=60.0)
    assert executive.state.mode == 'AUTO.LOITER'
