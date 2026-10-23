"""Mission behavior with real YASMIN/messages and no ROS node or FCU."""

from copy import deepcopy
import math
import threading
from types import SimpleNamespace

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped
from kv_executive.action import ExecuteMission
import kv_executive.mission_executive as executive_module
from kv_executive.mission_executive import (
    build_state_machine, MissionCanceled, MissionError, MissionExecutive,
)
from kv_executive.mission_geometry import (
    angular_distance, quaternion_from_yaw, yaw_from_quaternion,
)
from kv_executive.srv import PlanPath
from mavros_msgs.msg import State as VehicleState
from mavros_msgs.srv import CommandBool, ParamPull, SetMode
from nav_msgs.msg import Odometry, Path
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.action import GoalResponse
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from yasmin import Blackboard


def make_pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return Pose(
        position=Point(x=float(x), y=float(y), z=float(z)),
        orientation=quaternion_from_yaw(yaw),
    )


def xyz(pose):
    return (pose.position.x, pose.position.y, pose.position.z)


def arc_length(path):
    return math.fsum(
        math.dist(xyz(start.pose), xyz(end.pose))
        for start, end in zip(path.poses, path.poses[1:])
    )


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class ClockStop:
    def __init__(self, clock):
        self.clock = clock
        self.stopped = False
        self.on_wait = lambda: None

    def is_set(self):
        return self.stopped

    def wait(self, duration):
        self.clock.now += duration
        self.on_wait()
        return self.stopped


class Harness:
    """Bind production operations to deterministic telemetry and record commands."""

    _wait_until = MissionExecutive._wait_until
    _call = MissionExecutive._call
    _change_mode = MissionExecutive._change_mode
    _change_mode_command = MissionExecutive._change_mode_command
    _read_takeoff_altitude = MissionExecutive._read_takeoff_altitude
    _takeoff = MissionExecutive._takeoff
    _sweep = MissionExecutive._sweep
    _path_frame_transform = MissionExecutive._path_frame_transform
    _fly_to = MissionExecutive._fly_to
    _execute_path = MissionExecutive._execute_path
    _check_running = MissionExecutive._check_running
    _goal_callback = MissionExecutive._goal_callback
    _recover_to_loiter = MissionExecutive._recover_to_loiter
    _execute_mission_callback = MissionExecutive._execute_mission_callback

    def __init__(self, clock):
        self.config = {
            'service_timeout': 0.5,
            'mode_timeout': 0.3,
            'planning_timeout': 0.5,
            'takeoff_timeout': 1.0,
            'takeoff_altitude_parameter': 'MIS_TAKEOFF_ALT',
            'takeoff_altitude_tolerance': 0.3,
            'sweep_step_duration': 12.0,
            'sweep_hold_duration': 3.0,
            'path_turn_step_duration': 5.0,
            'hold_duration': 0.5,
            'yaw_tolerance': math.radians(5.0),
            'waypoint_timeout': 60.0,
            'path_speed': 1.0,
            'position_tolerance': 0.3,
            'local_frame': 'px4_odom',
            'robot_base_frame': 'kv_drone/base_link',
            'transform_timeout': 0.5,
            'setpoint_rate': 20.0,
            'offboard_warmup': 1.0,
        }
        self.events = []
        self._stop = ClockStop(clock)
        self._data_lock = threading.Lock()
        self._mission_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._recovering = False
        self._finalizing = False
        self._command_inflight = False
        self._last_error = ''
        self._state_machine = None
        self._target = None
        self._recovery_hold = False
        self._owns_flight = False
        self.context = object()
        self.latest_path = None
        self.sub_paths = []
        self.state = VehicleState(connected=True, armed=True, mode='AUTO.LOITER')
        self._vehicle_state = self.state
        self.odometry = Odometry(header=Header(frame_id='px4_odom'))
        self.odometry.header.stamp.sec = 123
        self.odometry.pose.pose = make_pose(1, 2, 3)
        self._pull_client = object()
        self._parameter_client = object()
        self._mode_client = object()
        self._arming_client = object()
        self._planning_client = object()

    def _report(self, message, error=False):
        self.events.append(('report', message, error))

    def _telemetry_ready(self):
        return True

    def _snapshot(self, expected_mode=None, require_armed=False):
        self.events.append((
            'telemetry', self.state.mode, self.state.armed,
            self.odometry.pose.pose.position.z,
        ))
        if require_armed and not self.state.armed:
            raise MissionError('Vehicle disarmed during mission')
        if expected_mode is not None and self.state.mode != expected_mode:
            raise MissionError('Unexpected flight mode')
        return deepcopy(self.state), deepcopy(self.odometry)

    def _enter_offboard(self, hold_pose, allowed_modes=('AUTO.TAKEOFF', 'AUTO.LOITER')):
        assert self.state.mode in allowed_modes
        self.events.append(('offboard', deepcopy(hold_pose)))
        self.state.mode = 'OFFBOARD'

    def _set_target(self, pose):
        self._target = deepcopy(pose)
        self.events.append(('target', deepcopy(pose)))

    def _ramp(self, start, target, duration, hold_duration=None):
        self.events.append(('ramp', deepcopy(start), deepcopy(target), duration))
        if hold_duration is not None:
            self.events.append(('dwell', deepcopy(target), hold_duration))
        self.odometry.pose.pose = deepcopy(target)

    def _wait_at_pose(self, target, hold_duration=None):
        self.events.append(('converged', deepcopy(target)))

    def _loiter(self):
        self.events.append(('loiter',))
        self.state.mode = 'AUTO.LOITER'


class TelemetryHarness(Harness):
    """Exercise real telemetry checks without creating a ROS node."""

    _telemetry_error_locked = MissionExecutive._telemetry_error_locked
    _telemetry_ready = MissionExecutive._telemetry_ready
    _snapshot = MissionExecutive._snapshot

    def __init__(self, clock):
        super().__init__(clock)
        self.config.update(
            state_timeout=10.0, telemetry_timeout=2.0,
            local_odometry_topic='/test/local_odometry',
        )
        self._odometry = self.odometry
        self._state_received = clock.now
        self._odometry_received = clock.now


class MotionHarness(Harness):
    """Run the production ramp and convergence waits with ideal pose tracking."""

    _wait_at_pose = MissionExecutive._wait_at_pose

    def _set_target(self, pose):
        super()._set_target(pose)
        self.odometry.pose.pose = deepcopy(pose)

    def _ramp(self, start, target, duration, hold_duration=None):
        started = self._stop.clock.now
        self.events.append(('ramp_started', started, deepcopy(target)))
        MissionExecutive._ramp(self, start, target, duration, hold_duration=hold_duration)
        self.events.append(('ramp_finished', self._stop.clock.now, deepcopy(target)))


@pytest.fixture
def harness(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(
        executive_module, 'time', SimpleNamespace(monotonic=clock.monotonic),
    )
    monkeypatch.setattr(executive_module.rclpy, 'ok', lambda context: True)
    return Harness(clock)


@pytest.fixture
def motion_harness(harness):
    return MotionHarness(harness._stop.clock)


@pytest.fixture
def telemetry_harness(monkeypatch):
    clock = Clock()
    clock.now = 20.0
    monkeypatch.setattr(
        executive_module, 'time', SimpleNamespace(monotonic=clock.monotonic),
    )
    return TelemetryHarness(clock)


@pytest.mark.parametrize('state_age', [2.1, 5.0, 9.9])
def test_state_gaps_longer_than_odometry_timeout_remain_usable(
    telemetry_harness, state_age,
):
    node = telemetry_harness
    node._state_received = node._stop.clock.now - state_age

    assert node._telemetry_ready()
    state, odometry = node._snapshot(expected_mode='AUTO.LOITER', require_armed=True)
    assert state == node._vehicle_state
    assert odometry == node._odometry
    assert state is not node._vehicle_state
    assert odometry is not node._odometry


def test_stale_state_aborts_despite_fresh_local_odometry(telemetry_harness):
    node = telemetry_harness
    node._state_received = node._stop.clock.now - 10.1

    assert not node._telemetry_ready()
    with pytest.raises(MissionError) as failure:
        node._snapshot()
    assert str(failure.value) == (
        'Stale FCU state on /mavros/state: last received 10.10 s ago '
        '(state_timeout=10.00 s)'
    )


def test_stale_local_odometry_aborts_despite_fresh_state(telemetry_harness):
    node = telemetry_harness
    node._odometry_received = node._stop.clock.now - 2.1

    assert not node._telemetry_ready()
    with pytest.raises(MissionError) as failure:
        node._snapshot()
    assert str(failure.value) == (
        'Stale local odometry on /test/local_odometry: last received 2.10 s ago '
        '(telemetry_timeout=2.00 s)'
    )


def test_disconnected_fcu_aborts_immediately_with_fresh_telemetry(telemetry_harness):
    node = telemetry_harness
    node._vehicle_state.connected = False

    assert not node._telemetry_ready()
    with pytest.raises(MissionError, match='FCU disconnected: /mavros/state'):
        node._snapshot()


@pytest.mark.parametrize('missing_attribute,topic', [
    ('_vehicle_state', '/mavros/state'),
    ('_odometry', '/test/local_odometry'),
])
def test_missing_telemetry_identifies_the_missing_topic(
    telemetry_harness, missing_attribute, topic,
):
    node = telemetry_harness
    setattr(node, missing_attribute, None)

    assert not node._telemetry_ready()
    with pytest.raises(MissionError) as failure:
        node._snapshot()
    assert 'Missing' in str(failure.value)
    assert topic in str(failure.value)


@pytest.mark.parametrize('changed_attribute,value,options,message', [
    ('armed', False, {'require_armed': True}, 'Vehicle disarmed during mission'),
    ('mode', 'POSCTL', {'expected_mode': 'OFFBOARD'},
     'Expected OFFBOARD, vehicle is in POSCTL'),
])
def test_fresh_telemetry_still_requires_expected_flight_state(
    telemetry_harness, changed_attribute, value, options, message,
):
    node = telemetry_harness
    setattr(node._vehicle_state, changed_attribute, value)

    assert node._telemetry_ready()
    with pytest.raises(MissionError, match=message):
        node._snapshot(**options)


def test_snapshot_copies_the_state_that_passed_telemetry_validation(telemetry_harness):
    node = telemetry_harness

    class DisconnectOnRelease:
        def __init__(self):
            self.lock = threading.Lock()

        def __enter__(self):
            self.lock.acquire()

        def __exit__(self, *args):
            self.lock.release()
            node._vehicle_state.connected = False

    node._data_lock = DisconnectOnRelease()
    state, _ = node._snapshot()

    assert state.connected
    assert not node._vehicle_state.connected


class Goal:
    def __init__(self, skip_takeoff=False):
        self.request = ExecuteMission.Goal(skip_takeoff=skip_takeoff)
        self.is_cancel_requested = False
        self.is_active = True
        self.outcome = None

    def succeed(self):
        self.outcome = 'succeeded'
        self.is_active = False

    def canceled(self):
        assert self.is_cancel_requested
        self.outcome = 'canceled'
        self.is_active = False

    def abort(self):
        self.outcome = 'aborted'
        self.is_active = False


def run_goal(harness, goal=None):
    goal = Goal() if goal is None else goal
    assert harness._goal_callback(goal.request) == GoalResponse.ACCEPT
    harness._goal_handle = goal
    return harness._execute_mission_callback(goal)


class SequenceExecutive:
    def __init__(self, fail_at=None):
        self.fail_at = fail_at
        self.executed = []
        self.reports = []
        self._last_error = ''

    def _report(self, message, error=False):
        self.reports.append((message, error))

    def _check_running(self):
        pass

    def run(self, name, blackboard):
        self.executed.append(name)
        if name == self.fail_at:
            raise MissionError('controlled failure')
        blackboard[name] = True

    def _takeoff(self, blackboard):
        self.run('TAKEOFF', blackboard)

    def _sweep(self, blackboard):
        self.run('SWEEP', blackboard)

    def _execute_path(self, blackboard):
        self.run('EXECUTE_PATH', blackboard)


def test_actual_yasmin_machine_runs_three_states_in_order():
    fake = SequenceExecutive()
    blackboard = Blackboard()
    outcome = build_state_machine(fake)(blackboard)
    assert outcome == 'succeeded'
    assert fake.executed == ['TAKEOFF', 'SWEEP', 'EXECUTE_PATH']
    assert all(blackboard[name] for name in fake.executed)
    assert fake.reports[-1] == ('EXECUTE_PATH: done', False)


def test_airborne_yasmin_machine_skips_takeoff():
    fake = SequenceExecutive()
    outcome = build_state_machine(fake, skip_takeoff=True)(Blackboard())
    assert outcome == 'succeeded'
    assert fake.executed == ['SWEEP', 'EXECUTE_PATH']


def test_yasmin_cancel_stops_before_next_state():
    fake = SequenceExecutive()

    def canceled_sweep(blackboard):
        fake.executed.append('SWEEP')
        raise MissionCanceled('Mission canceled')

    fake._sweep = canceled_sweep
    assert build_state_machine(fake)(Blackboard()) == 'canceled'
    assert fake.executed == ['TAKEOFF', 'SWEEP']
    assert not any(text == 'SWEEP: done' for text, _ in fake.reports)


@pytest.mark.parametrize('failed_index', range(3))
def test_actual_yasmin_machine_aborts_without_executing_later_states(failed_index):
    names = ['TAKEOFF', 'SWEEP', 'EXECUTE_PATH']
    fake = SequenceExecutive(fail_at=names[failed_index])
    assert build_state_machine(fake)(Blackboard()) == 'aborted'
    assert fake.executed == names[:failed_index + 1]
    assert fake._last_error == f'{names[failed_index]}: controlled failure'
    assert fake.reports[-1][1] is True


def configure_takeoff_services(harness, *, arming_success=True, integer=False):
    harness.state.armed = False
    harness.state.mode = 'MANUAL'
    harness.odometry.pose.pose = make_pose(1, 2, 10)
    progress = {'mode_requested': None, 'arm_requested': False}
    height = 4.0 if integer else 4.5

    def call(client, request, description, timeout=None):
        harness.events.append(('service', description, deepcopy(request)))
        if client is harness._pull_client:
            assert isinstance(request, ParamPull.Request)
            return ParamPull.Response(success=True, param_received=100)
        if client is harness._parameter_client:
            assert request.names == ['MIS_TAKEOFF_ALT']
            value = ParameterValue(
                type=(ParameterType.PARAMETER_INTEGER if integer
                      else ParameterType.PARAMETER_DOUBLE),
                integer_value=4, double_value=height,
            )
            return GetParameters.Response(values=[value])
        if client is harness._mode_client:
            progress['mode_requested'] = request.custom_mode
            return SetMode.Response(mode_sent=True)
        if client is harness._arming_client:
            assert request.value is True
            assert harness.state.mode == 'AUTO.TAKEOFF'
            progress['arm_requested'] = arming_success
            return CommandBool.Response(
                success=arming_success, result=0 if arming_success else 2,
            )
        raise AssertionError(f'Unexpected service: {description}')

    def advance_telemetry():
        now = harness._stop.clock.now
        if progress['mode_requested'] and now >= 0.05:
            harness.state.mode = progress['mode_requested']
        if progress['arm_requested'] and now >= 0.1:
            harness.state.armed = True
        if harness.state.armed and now >= 0.15:
            harness.odometry.pose.pose.position.z = 10 + height - 0.31
        if harness.state.armed and now >= 0.2:
            harness.odometry.pose.pose.position.z = 10 + height - 0.29

    harness._call = call
    harness._stop.on_wait = advance_telemetry
    return height


@pytest.mark.parametrize('integer', [False, True])
def test_takeoff_reads_px4_height_selects_mode_before_arming_and_waits(harness, integer):
    height = configure_takeoff_services(harness, integer=integer)
    blackboard = Blackboard()
    harness._takeoff(blackboard)

    service_names = [event[1] for event in harness.events if event[0] == 'service']
    assert service_names == [
        'Import PX4 parameters', 'Read MIS_TAKEOFF_ALT',
        'Set mode AUTO.TAKEOFF', 'Arm vehicle',
    ]
    assert blackboard['takeoff_altitude'] == height
    assert harness.state.armed is True
    assert harness._stop.clock.now >= 0.2
    readings = [event[3] for event in harness.events if event[0] == 'telemetry']
    assert 10 + height - 0.31 in readings
    assert readings[-1] >= 10 + height - 0.3
    assert not any(event[0] == 'offboard' for event in harness.events)


def test_rejected_arming_aborts_takeoff_before_altitude_wait(harness):
    configure_takeoff_services(harness, arming_success=False)
    with pytest.raises(MissionError, match='rejected arming'):
        harness._takeoff(Blackboard())
    assert harness.state.armed is False
    assert harness.odometry.pose.pose.position.z == 10.0
    assert harness._stop.clock.now < 0.1
    assert not any(event[0] in {'offboard', 'ramp'} for event in harness.events)


def test_cancel_during_arming_observes_armed_ack_before_recovery(harness):
    configure_takeoff_services(harness)
    goal = Goal()
    harness._goal_handle = goal
    call = harness._call

    def cancel_after_arm_dispatch(client, *args, **kwargs):
        response = call(client, *args, **kwargs)
        if client is harness._arming_client:
            goal.is_cancel_requested = True
        return response

    harness._call = cancel_after_arm_dispatch
    with pytest.raises(MissionCanceled):
        harness._takeoff(Blackboard())
    assert harness.state.armed
    assert harness.state.mode == 'AUTO.TAKEOFF'
    assert not harness._command_inflight
    assert harness.odometry.pose.pose.position.z < 14.0


def test_operator_arming_during_parameter_fetch_stops_takeoff_before_mode_change(harness):
    configure_takeoff_services(harness)
    original_call = harness._call

    def operator_arms(client, *args, **kwargs):
        response = original_call(client, *args, **kwargs)
        if client is harness._parameter_client:
            harness.state.armed = True
        return response

    harness._call = operator_arms
    with pytest.raises(MissionError, match='armed while preparing'):
        harness._takeoff(Blackboard())
    assert harness._owns_flight is False
    assert [event[1] for event in harness.events if event[0] == 'service'] == [
        'Import PX4 parameters', 'Read MIS_TAKEOFF_ALT',
    ]


def test_takeoff_aborts_if_fcu_leaves_takeoff_mode_before_climb(harness):
    configure_takeoff_services(harness)
    update = harness._stop.on_wait

    def leave_takeoff():
        update()
        if harness.state.armed:
            harness.state.mode = 'POSCTL'

    harness._stop.on_wait = leave_takeoff
    with pytest.raises(MissionError, match='Unexpected mode during TAKEOFF: POSCTL'):
        harness._takeoff(Blackboard())
    assert harness.odometry.pose.pose.position.z == 10.0
    assert not any(event[0] == 'offboard' for event in harness.events)


def test_sweep_proves_six_positive_sixty_degree_steps_and_final_convergence(harness):
    original = make_pose(7, -2, 5, math.radians(170))
    harness.odometry.pose.pose = deepcopy(original)
    blackboard = Blackboard()
    harness._sweep(blackboard)

    moves = [event for event in harness.events if event[0] == 'ramp']
    assert len(moves) == 6
    increments = []
    for _, start, target, duration in moves:
        assert xyz(start) == xyz(target) == xyz(original)
        assert duration == 12.0
        increments.append(angular_distance(
            yaw_from_quaternion(target.orientation),
            yaw_from_quaternion(start.orientation),
        ))
    assert increments == pytest.approx([math.pi / 3] * 6)
    assert math.fsum(increments) == pytest.approx(math.tau)
    assert yaw_from_quaternion(moves[0][2].orientation) < 0.0  # Crossed +pi.
    assert abs(angular_distance(
        yaw_from_quaternion(moves[-1][2].orientation), math.radians(170),
    )) < 1e-12
    assert blackboard['sweep_start_orientation'] == original.orientation
    assert [event[2] for event in harness.events if event[0] == 'dwell'] == [3.0] * 6
    assert harness.events[-2][0] == 'converged'
    assert harness.events[-2][1] == original
    assert harness.events[-1] == ('loiter',)


def test_sweep_spends_twelve_seconds_rotating_and_three_seconds_stable_per_step(
    motion_harness,
):
    node = motion_harness
    node._sweep(Blackboard())

    started = [event[1] for event in node.events if event[0] == 'ramp_started']
    finished = [event[1] for event in node.events if event[0] == 'ramp_finished']
    assert len(started) == len(finished) == 6
    assert all(15.0 <= end - begin < 15.1 for begin, end in zip(started, finished))
    assert 90.5 <= node._stop.clock.now < 91.0
    assert node.state.mode == 'AUTO.LOITER'


@pytest.mark.parametrize('drift', ['position', 'yaw'])
def test_sweep_dwell_restarts_after_measured_pose_drift(motion_harness, drift):
    node = motion_harness
    node.state.mode = 'OFFBOARD'
    target = deepcopy(node.odometry.pose.pose)
    recovered_at = []

    def disturb_then_recover():
        now = node._stop.clock.now
        node.odometry.pose.pose = deepcopy(target)
        if 1.0 <= now < 1.5:
            if drift == 'position':
                node.odometry.pose.pose.position.x += node.config['position_tolerance'] * 2
            else:
                yaw = yaw_from_quaternion(target.orientation)
                node.odometry.pose.pose.orientation = quaternion_from_yaw(
                    yaw + node.config['yaw_tolerance'] * 2,
                )
        elif now >= 1.5 and not recovered_at:
            recovered_at.append(now)

    node._stop.on_wait = disturb_then_recover
    node._wait_at_pose(target, hold_duration=node.config['sweep_hold_duration'])

    assert recovered_at
    assert 3.0 <= node._stop.clock.now - recovered_at[0] < 3.1
    assert node._stop.clock.now >= 4.5


def test_cancel_during_sweep_dwell_prevents_the_next_rotation(motion_harness):
    node = motion_harness
    goal = Goal(skip_takeoff=True)
    node._goal_handle = goal
    cancel_at = node.config['sweep_step_duration'] + 0.5

    def cancel_during_dwell():
        if node._stop.clock.now >= cancel_at:
            goal.is_cancel_requested = True

    node._stop.on_wait = cancel_during_dwell
    with pytest.raises(MissionCanceled, match='canceled by client'):
        node._sweep(Blackboard())

    assert 0.0 <= node._stop.clock.now - cancel_at < 0.1
    assert len([event for event in node.events if event[0] == 'ramp_started']) == 1
    assert not any(event[0] == 'ramp_finished' for event in node.events)
    assert not any(event[0] == 'loiter' for event in node.events)
    heading = yaw_from_quaternion(node.odometry.pose.pose.orientation)
    assert heading == pytest.approx(math.tau / 6)


def planned_corner_path():
    path = Path(header=Header(frame_id='map'))
    path.poses = [
        PoseStamped(
            header=Header(frame_id='map'),
            pose=make_pose(x, y, 5, math.pi / 2),
        )
        for x, y in [(10, 20), (10, 21), (7, 21), (7, 26)]
    ]
    return path


def test_execute_path_splits_equal_arcs_and_converts_waypoints_without_cutting_corners(harness):
    path = planned_corner_path()
    response = PlanPath.Response(success=True, message='planned', path=path)

    def call(client, request, description, timeout=None):
        assert client is harness._planning_client
        assert isinstance(request, PlanPath.Request)
        assert timeout == harness.config['planning_timeout']
        return response

    lookups = []

    def lookup(frame, body, stamp, timeout):
        lookups.append((frame, body, stamp.nanoseconds, timeout.nanoseconds))
        result = TransformStamped()
        result.transform.translation.x = 10.0
        result.transform.translation.y = 20.0
        result.transform.translation.z = 5.0
        result.transform.rotation = quaternion_from_yaw(math.pi / 2)
        return result

    harness._call = call
    harness._tf_buffer = SimpleNamespace(lookup_transform=lookup)
    blackboard = Blackboard()
    harness._execute_path(blackboard)

    assert lookups == [('map', 'kv_drone/base_link', 123_000_000_000, 500_000_000)]
    assert len(harness.sub_paths) == 3
    assert [arc_length(leg) for leg in harness.sub_paths] == pytest.approx([3.0] * 3)
    assert blackboard['sub_paths'] == harness.sub_paths
    assert harness.latest_path == path
    assert harness.latest_path is not path
    moves = [event for event in harness.events if event[0] == 'ramp']
    expected = [
        ((1, 2, 3), 0.0),
        ((2, 2, 3), 0.0),
        ((2, 2, 3), math.pi / 2),
        ((2, 4, 3), math.pi / 2),
        ((2, 5, 3), math.pi / 2),
        ((2, 5, 3), 0.0),
        ((4, 5, 3), 0.0),
        ((7, 5, 3), 0.0),
    ]
    assert len(moves) == len(expected)
    for move, (position, yaw) in zip(moves, expected):
        assert xyz(move[2]) == pytest.approx(position)
        assert yaw_from_quaternion(move[2].orientation) == pytest.approx(yaw)
    assert [event[3] for event in moves] == pytest.approx([0, 1, 7.5, 2, 1, 7.5, 2, 3])
    assert harness.events[-1] == ('loiter',)
    done = [event for event in harness.events if event[0] == 'report']
    assert [event[1] for event in done] == [
        'EXECUTE_PATH: sub-path 1/3 done', 'EXECUTE_PATH: sub-path 2/3 done',
        'EXECUTE_PATH: sub-path 3/3 done',
    ]


@pytest.mark.parametrize('dx,dy', [(3, 4), (-3, 4), (-3, -4), (3, -4)])
@pytest.mark.parametrize('planner_yaw', [-2.4, 0.8])
def test_fly_to_faces_xy_direction_before_translation(harness, dx, dy, planner_yaw):
    start = make_pose(2, 3, 4)
    target = make_pose(2 + dx, 3 + dy, 10, planner_yaw)
    original_target = deepcopy(target)
    expected_yaw = math.atan2(dy, dx)

    reached = harness._fly_to(start, target)

    moves = [event for event in harness.events if event[0] == 'ramp']
    assert len(moves) == 2
    _, turn_start, turn_target, turn_duration = moves[0]
    assert turn_start == start
    assert xyz(turn_target) == xyz(start)
    assert yaw_from_quaternion(turn_target.orientation) == pytest.approx(expected_yaw)
    assert turn_duration == pytest.approx(
        abs(expected_yaw) * harness.config['path_turn_step_duration'] / (math.tau / 6),
    )
    _, translation_start, translation_target, duration = moves[1]
    assert translation_start == turn_target
    assert xyz(translation_target) == xyz(target)
    assert translation_target.orientation == turn_target.orientation
    assert translation_target.orientation.x == 0.0
    assert translation_target.orientation.y == 0.0
    assert duration == pytest.approx(math.sqrt(dx * dx + dy * dy + 36))
    assert reached == translation_target
    assert target == original_target
    assert start == make_pose(2, 3, 4)


@pytest.mark.parametrize('dx,dy,dz', [(0, 0, 0), (0, 0, 5), (5e-7, 5e-7, 2)])
def test_fly_to_preserves_heading_without_horizontal_direction(harness, dx, dy, dz):
    start = make_pose(2, 3, 4, 1.2)
    target = make_pose(2 + dx, 3 + dy, 4 + dz, -0.7)

    reached = harness._fly_to(start, target)

    moves = [event for event in harness.events if event[0] == 'ramp']
    assert len(moves) == 1
    assert moves[0][1] == start
    assert xyz(reached) == xyz(target)
    assert yaw_from_quaternion(reached.orientation) == pytest.approx(1.2)
    assert moves[0][2] == reached
    assert moves[0][3] == pytest.approx(math.sqrt(dx * dx + dy * dy + dz * dz))


def test_fly_to_turns_shortest_angle_across_yaw_wrap(harness):
    start = make_pose(yaw=math.radians(170))
    direction = math.radians(-170)
    target = make_pose(math.cos(direction), math.sin(direction), yaw=0)

    reached = harness._fly_to(start, target)

    moves = [event for event in harness.events if event[0] == 'ramp']
    assert len(moves) == 2
    assert xyz(moves[0][1]) == xyz(moves[0][2]) == xyz(start)
    assert moves[0][3] == pytest.approx(harness.config['path_turn_step_duration'] / 3)
    assert yaw_from_quaternion(reached.orientation) == pytest.approx(direction)


@pytest.mark.parametrize('invalid', ['planner_failed', 'empty', 'no_frame', 'bad_quaternion'])
def test_bad_plan_never_switches_to_offboard_or_issues_motion(harness, invalid):
    path = planned_corner_path()
    success = invalid != 'planner_failed'
    if invalid == 'empty':
        path.poses = []
    elif invalid == 'no_frame':
        path.header.frame_id = ''
    elif invalid == 'bad_quaternion':
        path.poses[-1].pose.orientation = Quaternion(w=0.0)
    response = PlanPath.Response(success=success, message='No valid route', path=path)
    harness._call = lambda *args, **kwargs: response

    with pytest.raises((MissionError, ValueError)):
        harness._execute_path(Blackboard())
    assert not any(
        event[0] in {'offboard', 'target', 'ramp', 'loiter'} for event in harness.events
    )


@pytest.mark.parametrize('change_during_planning', [False, True])
def test_operator_mode_change_prevents_path_execution(harness, change_during_planning):
    def call(*args, **kwargs):
        harness.state.mode = 'POSCTL'
        return PlanPath.Response(success=True, path=planned_corner_path())

    harness._call = call
    if not change_during_planning:
        harness.state.mode = 'POSCTL'

    with pytest.raises(MissionError, match='Unexpected flight mode'):
        harness._execute_path(Blackboard())
    assert not any(event[0] in {'offboard', 'target', 'ramp'} for event in harness.events)


@pytest.mark.parametrize('drift_during_warmup', [False, True])
def test_path_start_drift_requires_replanning(harness, drift_during_warmup):
    path = Path(header=Header(frame_id='px4_odom'), poses=[
        PoseStamped(pose=make_pose(1, 2, 3)),
        PoseStamped(pose=make_pose(2, 2, 3)),
    ])
    harness._call = lambda *args, **kwargs: PlanPath.Response(success=True, path=path)
    enter_offboard = harness._enter_offboard

    def drift_on_entry(*args, **kwargs):
        enter_offboard(*args, **kwargs)
        harness.odometry.pose.pose.position.x += 0.4

    harness._enter_offboard = drift_on_entry
    if not drift_during_warmup:
        harness.odometry.pose.pose.position.x += 0.4

    with pytest.raises(MissionError, match='replan required'):
        harness._execute_path(Blackboard())
    assert not any(event[0] in {'target', 'ramp'} for event in harness.events)
    assert any(event[0] == 'offboard' for event in harness.events) == drift_during_warmup


def test_path_start_alignment_uses_speed_ramp(harness):
    path = Path(header=Header(frame_id='px4_odom'), poses=[
        PoseStamped(pose=make_pose(1.2, 2, 3)),
        PoseStamped(pose=make_pose(2, 2, 3)),
    ])
    harness._call = lambda *args, **kwargs: PlanPath.Response(success=True, path=path)
    harness._execute_path(Blackboard())
    initial = next(event for event in harness.events if event[0] == 'ramp')
    assert xyz(initial[1]) == pytest.approx((1, 2, 3))
    assert xyz(initial[2]) == pytest.approx((1.2, 2, 3))
    assert initial[3] == pytest.approx(0.2 / harness.config['path_speed'])


def test_mode_sent_response_still_waits_for_fcu_telemetry_acknowledgment(harness):
    harness._call = lambda *args, **kwargs: SetMode.Response(mode_sent=True)
    observed_before_ack = []

    def acknowledge():
        observed_before_ack.append(harness.state.mode)
        if harness._stop.clock.now >= 0.1:
            harness.state.mode = 'OFFBOARD'

    harness._stop.on_wait = acknowledge
    harness._change_mode('OFFBOARD')
    assert harness._stop.clock.now >= 0.1
    assert observed_before_ack[0] == 'AUTO.LOITER'
    assert harness.state.mode == 'OFFBOARD'


def test_mode_request_without_telemetry_acknowledgment_times_out(harness):
    harness._call = lambda *args, **kwargs: SetMode.Response(mode_sent=True)
    with pytest.raises(MissionError, match='FCU mode OFFBOARD'):
        harness._change_mode('OFFBOARD')
    assert harness.state.mode == 'AUTO.LOITER'


def test_rejected_mode_request_fails_without_waiting(harness):
    harness._call = lambda *args, **kwargs: SetMode.Response(mode_sent=False)
    with pytest.raises(MissionError, match='rejected mode request'):
        harness._change_mode('OFFBOARD')
    assert harness._stop.clock.now == 0.0


class PendingFuture:
    def __init__(self):
        self.canceled = False

    def done(self):
        return False

    def cancel(self):
        self.canceled = True

    def result(self):
        raise AssertionError('A pending future result must not be consumed')


def test_service_deadline_covers_discovery_and_long_running_future(harness):
    future = PendingFuture()
    requests = []

    def call_async(request):
        requests.append(request)
        return future

    client = SimpleNamespace(
        service_is_ready=lambda: harness._stop.clock.now >= 0.1,
        call_async=call_async,
    )
    request = Trigger.Request()
    with pytest.raises(MissionError, match='Timed out: test request'):
        harness._call(client, request, 'test request', timeout=0.15)
    assert requests == [request]
    assert future.canceled is True
    assert harness._stop.clock.now == pytest.approx(0.15)


def test_unavailable_service_times_out_without_dispatching_request(harness):
    def unexpected_call(request):
        raise AssertionError('Unavailable service must not receive a request')

    client = SimpleNamespace(service_is_ready=lambda: False, call_async=unexpected_call)
    with pytest.raises(MissionError, match='test request service'):
        harness._call(client, Trigger.Request(), 'test request', timeout=0.1)
    assert harness._stop.clock.now == pytest.approx(0.1)


def test_concurrent_mission_goal_is_rejected_without_touching_active_mission(harness):
    active_path = planned_corner_path()
    harness.latest_path = active_path
    harness._mission_lock.acquire()
    try:
        assert harness._goal_callback(ExecuteMission.Goal()) == GoalResponse.REJECT
        assert harness.latest_path is active_path
        assert harness.events == []
        assert harness._mission_lock.locked()
    finally:
        harness._mission_lock.release()


def test_offboard_waits_for_continuous_setpoint_warmup(harness):
    harness._published_count = 0
    harness._stream_started = 0.0
    harness._last_publish = 0.0
    requested = []
    harness._change_mode = requested.append

    def publish():
        now = harness._stop.clock.now
        if harness._published_count == 0:
            harness._stream_started = now
        harness._published_count += 1
        harness._last_publish = now

    harness._stop.on_wait = publish
    MissionExecutive._enter_offboard(harness, harness.odometry.pose.pose)
    assert requested == ['OFFBOARD']
    assert harness._stop.clock.now >= 1.05
    assert harness._published_count >= 15


def test_offboard_does_not_start_after_setpoint_stream_stops(harness):
    harness._published_count = 0
    harness._stream_started = 0.0
    harness._last_publish = 0.0
    requested = []
    harness._change_mode = requested.append

    def stopped_stream():
        if harness._published_count == 0:
            harness._published_count = 30
            harness._stream_started = harness._stop.clock.now
            harness._last_publish = harness._stop.clock.now

    harness._stop.on_wait = stopped_stream
    with pytest.raises(MissionError, match='OFFBOARD setpoint warmup'):
        MissionExecutive._enter_offboard(harness, harness.odometry.pose.pose)
    assert requested == []


@pytest.mark.parametrize('change_during_warmup', [False, True])
def test_offboard_entry_preserves_operator_mode_selection(harness, change_during_warmup):
    harness._published_count = 0
    harness._stream_started = 0.0
    harness._last_publish = 0.0
    requested = []
    harness._change_mode = requested.append

    def operator_takes_control():
        harness.state.mode = 'POSCTL'

    harness._stop.on_wait = operator_takes_control
    if not change_during_warmup:
        harness.state.mode = 'POSCTL'
    with pytest.raises(MissionError, match='OFFBOARD entry interrupted'):
        MissionExecutive._enter_offboard(harness, harness.odometry.pose.pose)
    assert requested == []
    assert harness.state.mode == 'POSCTL'


@pytest.mark.parametrize('loiter_acknowledged', [True, False])
def test_aborted_mission_refreshes_hold_until_loiter_is_acknowledged(
    harness, monkeypatch, loiter_acknowledged,
):
    monkeypatch.setattr(executive_module.rclpy, 'ok', lambda context: True)
    harness.state.mode = 'OFFBOARD'
    harness.odometry.pose.pose = make_pose(4, 5, 6, 0.7)
    harness._target = make_pose(100, 100, 100)
    errors = []
    harness.get_logger = lambda: SimpleNamespace(error=errors.append)

    def owned_takeoff(blackboard):
        harness._owns_flight = True

    def fail_sweep(blackboard):
        raise MissionError('sweep telemetry failed')

    def unexpected_path(blackboard):
        raise AssertionError('Path must not run after aborted sweep')

    harness._takeoff = owned_takeoff
    harness._sweep = fail_sweep
    harness._execute_path = unexpected_path
    if not loiter_acknowledged:
        def reject_loiter():
            raise MissionError('mode refused')

        harness._loiter = reject_loiter

    goal = Goal()
    result = run_goal(harness, goal)
    assert result.success is False
    assert 'sweep telemetry failed' in result.message
    assert goal.outcome == 'aborted'
    assert harness._goal_handle is None
    holds = [event[1] for event in harness.events if event[0] == 'target']
    assert holds[-1] == harness.odometry.pose.pose
    assert not harness._mission_lock.locked()
    assert harness._state_machine is None
    if loiter_acknowledged:
        assert harness.state.mode == 'AUTO.LOITER'
        assert harness._target is None
        assert harness._recovery_hold is False
        assert errors == []
    else:
        assert harness.state.mode == 'OFFBOARD'
        assert harness._target == harness.odometry.pose.pose
        assert harness._recovery_hold is True
        assert 'retaining the local hold setpoint' in result.message
        assert any(event[0] == 'report' and event[2] for event in harness.events)


def test_recovery_preserves_operator_takeover_observed_in_fresh_snapshot(harness):
    harness._owns_flight = True
    harness.state.mode = 'OFFBOARD'
    harness._snapshot = lambda **kwargs: (
        VehicleState(connected=True, armed=True, mode='POSCTL'),
        deepcopy(harness.odometry),
    )

    assert harness._recover_to_loiter() == ''

    assert harness.events == []
    assert harness._target is None


@pytest.mark.parametrize('armed,mode', [(False, 'OFFBOARD'), (True, 'POSCTL')])
def test_recovery_rechecks_vehicle_after_snapshot_failure(harness, armed, mode):
    harness._owns_flight = True
    harness.state.mode = 'OFFBOARD'

    def changed_vehicle(**kwargs):
        harness.state.armed = armed
        harness.state.mode = mode
        raise MissionError('Telemetry unavailable')

    harness._snapshot = changed_vehicle

    assert harness._recover_to_loiter() == ''

    assert harness.events == []
    assert harness._target is None


@pytest.mark.parametrize('late_state', [
    VehicleState(connected=True, armed=True, mode='AUTO.LOITER'),
    VehicleState(connected=True, armed=False, mode='OFFBOARD'),
])
def test_recovery_hold_stops_after_late_loiter_confirmation_or_disarming(harness, late_state):
    hold = make_pose(4, 5, 6)
    harness._target = hold
    harness._recovery_hold = True

    MissionExecutive._state_callback(
        harness, VehicleState(connected=True, armed=True, mode='OFFBOARD'),
    )
    assert harness._target is hold
    assert harness._recovery_hold is True

    harness._stop.clock.now = 0.25
    MissionExecutive._state_callback(harness, late_state)
    assert harness._vehicle_state is late_state
    assert harness._state_received == 0.25
    assert harness._target is None
    assert harness._recovery_hold is False


@pytest.mark.parametrize('recovering', [False, True])
def test_cancel_interrupts_waits_but_allows_recovery(harness, recovering):
    goal = Goal()
    goal.is_cancel_requested = True
    harness._goal_handle = goal
    harness._recovering = recovering
    if recovering:
        harness._check_running()
    else:
        with pytest.raises(MissionCanceled):
            harness._wait_until(lambda: False, 10.0, 'a pending operation')
        assert harness._stop.clock.now == 0.0
    assert not harness._stop.is_set()


def test_cancel_preserves_hover_and_releases_goal_slot(harness):
    goal = Goal()

    def climb(blackboard):
        harness._owns_flight = True
        harness.state.mode = 'OFFBOARD'
        harness._target = make_pose(100, 100, 100)
        goal.is_cancel_requested = True

    harness._takeoff = climb
    result = run_goal(harness, goal)
    assert goal.outcome == 'canceled'
    assert not result.success
    assert 'canceled' in result.message
    assert harness.state.mode == 'AUTO.LOITER'
    assert harness._target is None
    assert harness._goal_handle is None
    assert not harness._mission_lock.locked()
    assert not harness._stop.is_set()
    assert not any(event[0] == 'ramp' for event in harness.events)


@pytest.mark.parametrize('armed,mode', [(False, 'AUTO.LOITER'), (True, 'POSCTL')])
def test_invalid_airborne_goal_aborts_without_changing_flight_mode(harness, armed, mode):
    harness.state.armed = armed
    harness.state.mode = mode
    goal = Goal(skip_takeoff=True)
    result = run_goal(harness, goal)
    assert not result.success
    assert goal.outcome == 'aborted'
    assert harness.state.mode == mode
    assert not harness._mission_lock.locked()
    assert not any(event[0] in {'offboard', 'loiter', 'ramp'} for event in harness.events)
