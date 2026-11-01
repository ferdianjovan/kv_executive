"""Mission behavior with real YASMIN/messages and no ROS node or FCU."""

from copy import deepcopy
import math
import threading
from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TransformStamped
from kv_executive.action import ExecuteMission
import kv_executive.mission_executive as executive_module
from kv_executive.mission_executive import (
    build_state_machine, ExplorationStateMachine, MissionAbortRequested,
    MissionCanceled, MissionError,
    MissionExecutive, MissionStateRequested,
)
from kv_executive.mission_geometry import (
    angular_distance, quaternion_from_yaw, yaw_from_quaternion,
)
from kv_executive.msg import BatteryReturnEstimate, MissionStateRequest
from kv_executive.srv import PlanPath
from kv_executive.verification import VERIFY_DEFAULTS, VerifyController
from mavros_msgs.msg import State as VehicleState
from mavros_msgs.srv import CommandBool, ParamPull, SetMode
from nav_msgs.msg import Odometry, Path
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.action import GoalResponse
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from unique_identifier_msgs.msg import UUID
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
    _check_motion_observation = MissionExecutive._check_motion_observation
    _verify_parent_entry = MissionExecutive._verify_parent_entry
    _exploration_parent_entry = MissionExecutive._exploration_parent_entry
    _verify_reset_source_localization = MissionExecutive._verify_reset_source_localization
    _verify_request_is_completed = MissionExecutive._verify_request_is_completed
    _verify_defer_viewpoint = MissionExecutive._verify_defer_viewpoint
    _verify_request_is_deferred = MissionExecutive._verify_request_is_deferred
    _goal_callback = MissionExecutive._goal_callback
    _goal_accepted_callback = MissionExecutive._goal_accepted_callback
    request_mission_state = MissionExecutive.request_mission_state
    _mission_state_request_callback = MissionExecutive._mission_state_request_callback
    _goal_state_request_callback = MissionExecutive._goal_state_request_callback
    _battery_alert_callback = MissionExecutive._battery_alert_callback
    _recover_to_loiter = MissionExecutive._recover_to_loiter
    _execute_mission_callback = MissionExecutive._execute_mission_callback

    def __init__(self, clock):
        self.config = {
            **VERIFY_DEFAULTS,
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
            'local_odometry_frame_aliases': ['map'],
            'robot_base_frame': 'kv_drone/base_link',
            'transform_timeout': 0.5,
            'setpoint_rate': 20.0,
            'offboard_warmup': 1.0,
            'battery_alert_timeout': 5.0,
            'telemetry_timeout': 2.0,
        }
        self.events = []
        self._stop = ClockStop(clock)
        self._data_lock = threading.Lock()
        self._odometry_aliases_reported = set()
        self._mission_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._mission_started_ns = 122_000_000_000
        self._requested_mission_state = None
        self._parent_state = None
        self._verify_retry_after = 0.0
        self._airborne_ready = True
        self._motion_guard = None
        self._phase_sequence = 0
        self._phase_publisher = SimpleNamespace(
            publish=lambda phase: self.events.append(('phase', phase)))
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
        self._verify = VerifyController(self, error_type=MissionError)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=123_000_000_000, to_msg=lambda: TimeMessage(sec=123)))

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


def test_stock_mavros_map_odometry_keeps_local_coordinates_and_input(telemetry_harness):
    node = telemetry_harness
    incoming = deepcopy(node._odometry)
    incoming.header.frame_id = 'map'
    incoming.pose.pose = make_pose(21, -34, 8, math.radians(73))
    incoming.pose.covariance[0] = 0.25
    incoming.twist.twist.linear.x = 1.5
    original = deepcopy(incoming)

    MissionExecutive._odometry_callback(node, incoming)
    _, accepted = node._snapshot(expected_mode='AUTO.LOITER', require_armed=True)

    assert accepted.header.frame_id == 'px4_odom'
    assert accepted.pose == original.pose
    assert accepted.twist == original.twist
    assert accepted.child_frame_id == original.child_frame_id
    assert accepted.header.stamp == original.header.stamp
    assert incoming == original
    assert node._odometry is not incoming


@pytest.mark.parametrize('frame,aliases', [
    ('other_odom', ['map']), ('/map', ['map']), ('map', []),
])
def test_unconfigured_odometry_frames_still_abort(telemetry_harness, frame, aliases):
    node = telemetry_harness
    node.config['local_odometry_frame_aliases'] = aliases
    incoming = deepcopy(node._odometry)
    incoming.header.frame_id = frame

    MissionExecutive._odometry_callback(node, incoming)

    with pytest.raises(MissionError, match='Local odometry frame is'):
        node._snapshot()
    assert incoming.header.frame_id == frame


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
        self.goal_id = UUID(uuid=[1] * 16)

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
    def __init__(self, fail_at=None, cancel_after=2):
        self.fail_at = fail_at
        self.cancel_after = cancel_after
        self.cancel_requested = False
        self.executed = []
        self.reports = []
        self._last_error = ''
        self._airborne_ready = False
        self._verify = SimpleNamespace(
            approach_smoke=lambda bb: 'reacquire', reacquire=lambda bb: 'explore',
            center_fire=lambda bb: 'explore', search_fire=lambda bb: 'approach',
        )

    def _exploration_parent_entry(self, blackboard):
        self._check_running()

    def _report(self, message, error=False):
        self.reports.append((message, error))
        if message == 'EXECUTE_PATH: done':
            completed = sum(text == message for text, _ in self.reports)
            if completed == self.cancel_after:
                self.cancel_requested = True

    def _check_running(self):
        if self.cancel_requested:
            raise MissionCanceled('Mission canceled at cycle boundary')

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


def test_actual_yasmin_machine_nests_exploration_and_repeats_without_takeoff():
    fake = SequenceExecutive()
    blackboard = Blackboard()
    machine = build_state_machine(fake)
    assert machine.get_start_state() == 'Exploration'
    assert set(machine.get_states()) == {'Exploration', 'Verify'}
    assert machine.get_transitions()['Exploration']['verify_requested'] == 'Verify'
    assert machine.get_transitions()['Verify']['explore'] == 'Exploration'
    exploration = machine.get_states()['Exploration']['state']
    assert isinstance(exploration, ExplorationStateMachine)
    assert exploration.get_start_state() == 'ENTER_EXPLORATION'
    assert exploration.get_transitions()['ENTER_EXPLORATION']['succeeded'] == 'TAKEOFF'
    assert exploration.get_transitions()['EXECUTE_PATH']['succeeded'] == 'SWEEP'
    assert machine(blackboard) == 'canceled'
    assert fake.executed == [
        'TAKEOFF', 'SWEEP', 'EXECUTE_PATH', 'SWEEP', 'EXECUTE_PATH',
    ]
    assert all(blackboard[name] for name in fake.executed)
    assert sum(text == 'EXECUTE_PATH: done' for text, _ in fake.reports) == 2
    assert not any(error for _, error in fake.reports)


def test_airborne_yasmin_machine_repeats_without_takeoff():
    fake = SequenceExecutive()
    machine = build_state_machine(fake, skip_takeoff=True)
    exploration = machine.get_states()['Exploration']['state']
    assert exploration.get_start_state() == 'ENTER_EXPLORATION'
    assert exploration.get_transitions()['ENTER_EXPLORATION']['succeeded'] == 'SWEEP'
    assert 'TAKEOFF' not in exploration.get_states()
    assert machine(Blackboard()) == 'canceled'
    assert fake.executed == ['SWEEP', 'EXECUTE_PATH'] * 2


def test_cancel_in_a_later_cycle_stops_before_another_sweep():
    fake = SequenceExecutive(cancel_after=10)

    def canceled_path(blackboard):
        fake.run('EXECUTE_PATH', blackboard)
        if fake.executed.count('EXECUTE_PATH') == 3:
            raise MissionCanceled('Mission canceled during later path')

    fake._execute_path = canceled_path
    assert build_state_machine(fake)(Blackboard()) == 'canceled'
    assert fake.executed == ['TAKEOFF'] + ['SWEEP', 'EXECUTE_PATH'] * 3
    assert sum(text == 'EXECUTE_PATH: done' for text, _ in fake.reports) == 2


@pytest.mark.parametrize('state,outcome', [
    ('Return', 'return_requested'), ('Verify', 'verify_requested'),
])
def test_requested_higher_state_propagates_out_of_exploration(state, outcome):
    fake = SequenceExecutive()

    def request_state(blackboard):
        fake.run('SWEEP', blackboard)
        raise MissionStateRequested(state)

    fake._sweep = request_state
    assert ExplorationStateMachine(fake)(Blackboard()) == outcome
    assert fake.executed == ['TAKEOFF', 'SWEEP']
    assert fake._last_error == ''
    assert not any(error for _, error in fake.reports)


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


@pytest.mark.parametrize('stage', ['TAKEOFF', 'SWEEP', 'EXECUTE_PATH'])
def test_monitor_abort_exits_machine_without_entering_verify(stage):
    fake = SequenceExecutive(fail_at=None)
    original_run = fake.run

    def abort_stage(name, blackboard):
        original_run(name, blackboard)
        if name == stage:
            raise MissionAbortRequested('Monitoring requested abort')

    fake.run = abort_stage

    assert build_state_machine(fake)(Blackboard()) == 'aborted'
    assert fake.executed[-1] == stage
    assert 'Monitoring requested abort' in fake._last_error


def configure_takeoff_services(harness, *, arming_success=True, integer=False):
    harness._airborne_ready = False
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


@pytest.mark.parametrize('state', ['Return', 'Verify'])
def test_transition_during_arming_waits_for_safe_handoff(harness, state):
    height = configure_takeoff_services(harness)
    harness._goal_handle = Goal()
    call = harness._call

    def request_after_arm_dispatch(client, *args, **kwargs):
        response = call(client, *args, **kwargs)
        if client is harness._arming_client:
            assert harness.request_mission_state(state)
        return response

    harness._call = request_after_arm_dispatch
    with pytest.raises(MissionStateRequested) as request:
        harness._takeoff(Blackboard())
    assert request.value.state == state
    assert harness.state.armed
    assert harness.state.mode == 'AUTO.TAKEOFF'
    assert not harness._command_inflight
    if state == 'Verify':
        assert harness._airborne_ready
        assert harness.odometry.pose.pose.position.z >= 10 + height - 0.3
    else:
        assert not harness._airborne_ready
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


@pytest.mark.parametrize('odometry_frame', ['px4_odom', 'map'])
def test_execute_path_splits_equal_arcs_and_converts_waypoints_without_cutting_corners(
    harness, odometry_frame,
):
    incoming = deepcopy(harness.odometry)
    incoming.header.frame_id = odometry_frame
    MissionExecutive._odometry_callback(harness, incoming)
    harness.odometry = harness._odometry
    assert harness.odometry.header.frame_id == 'px4_odom'
    harness.events.clear()
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


def test_transition_during_mode_command_waits_for_telemetry_acknowledgment(harness):
    harness._goal_handle = Goal()

    def request_transition(*args, **kwargs):
        assert harness._command_inflight
        assert harness.request_mission_state('Return')
        return SetMode.Response(mode_sent=True)

    def acknowledge():
        if harness._stop.clock.now >= 0.1:
            harness.state.mode = 'OFFBOARD'

    harness._call = request_transition
    harness._stop.on_wait = acknowledge

    harness._change_mode('OFFBOARD')
    assert harness._stop.clock.now >= 0.1
    assert harness.state.mode == 'OFFBOARD'
    assert not harness._command_inflight
    assert harness._requested_mission_state == 'Return'
    with pytest.raises(MissionStateRequested):
        harness._check_running()


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


@pytest.mark.parametrize('state,outcome', [
    ('Return', 'return_requested'), ('Verify', 'verify_requested'),
])
def test_state_request_interrupts_pending_wait(harness, state, outcome):
    harness._goal_handle = Goal()
    assert harness.request_mission_state(state)

    with pytest.raises(MissionStateRequested) as request:
        harness._wait_until(lambda: False, 10.0, 'a pending operation')
    assert request.value.state == state
    assert request.value.outcome == outcome
    assert harness._stop.clock.now == 0.0
    assert not harness._stop.is_set()


def test_abort_request_interrupts_wait_without_entering_another_parent(harness):
    harness._goal_handle = Goal()
    assert harness.request_mission_state('Abort')

    with pytest.raises(MissionAbortRequested):
        harness._wait_until(lambda: False, 10.0, 'pending motion')

    assert harness._stop.clock.now == 0.0
    assert harness._parent_state is None


def battery_alert(harness, available=True):
    current = PoseStamped(
        header=Header(stamp=TimeMessage(sec=123), frame_id='px4_odom'),
        pose=deepcopy(harness.odometry.pose.pose),
    )
    home = deepcopy(current)
    home.header.stamp.sec = 1  # Home is a reference and need not be a fresh observation.
    return BatteryReturnEstimate(
        header=deepcopy(current.header), current_pose=current, home_pose=home,
        remaining_percent=8.0, buffered_required_percent=10.0,
        return_margin_percent=-2.0, home_distance_m=0.0,
        mean_percent_per_meter=0.1 if available else math.nan,
        return_required_percent=0.0 if available else math.nan,
        sample_count=1 if available else 0,
        estimate_available=available, return_alert=True,
    )


@pytest.mark.parametrize('available', [True, False])
def test_fresh_return_or_unknown_rate_reserve_alert_requests_abort(harness, available):
    harness._goal_handle = Goal()

    harness._battery_alert_callback(battery_alert(harness, available))

    assert harness._requested_mission_state == 'Abort'
    assert any(event[0] == 'report' and 'BATTERY:' in event[1] for event in harness.events)


@pytest.mark.parametrize('invalid', [
    'not_alert', 'before_goal', 'same_goal_tick', 'stale', 'future', 'stale_pose',
    'future_pose', 'uninitialized_pose', 'message_frame', 'pose_frame', 'home_frame',
    'invalid_pose', 'invalid_home', 'invalid_orientation', 'negative_remaining',
    'overfull_remaining', 'invalid_remaining', 'negative_required', 'invalid_required',
    'negative_distance', 'invalid_distance', 'inconsistent_margin', 'positive_margin',
    'no_samples', 'invalid_rate', 'negative_rate', 'invalid_return', 'negative_return',
])
def test_incoherent_or_old_battery_alert_does_not_abort(harness, invalid):
    harness._goal_handle = Goal()
    message = battery_alert(harness)
    if invalid == 'not_alert':
        message.return_alert = False
    elif invalid == 'before_goal':
        message.header.stamp.sec = 121
    elif invalid == 'same_goal_tick':
        message.header.stamp.sec = 122
    elif invalid == 'stale':
        harness._mission_started_ns = 1
        message.header.stamp.sec = 117
    elif invalid == 'future':
        message.header.stamp.sec = 124
    elif invalid in {'stale_pose', 'future_pose', 'uninitialized_pose'}:
        message.current_pose.header.stamp.sec = {
            'stale_pose': 120, 'future_pose': 124, 'uninitialized_pose': 0,
        }[invalid]
    elif invalid in {'message_frame', 'pose_frame', 'home_frame'}:
        {
            'message_frame': message.header,
            'pose_frame': message.current_pose.header,
            'home_frame': message.home_pose.header,
        }[invalid].frame_id = 'other_odom'
    elif invalid in {'invalid_pose', 'invalid_home'}:
        pose = message.current_pose if invalid == 'invalid_pose' else message.home_pose
        pose.pose.position.x = math.nan
    elif invalid == 'invalid_orientation':
        message.current_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
    elif invalid == 'negative_remaining':
        message.remaining_percent = -1.0
    elif invalid == 'overfull_remaining':
        message.remaining_percent = 101.0
    elif invalid == 'invalid_remaining':
        message.remaining_percent = math.nan
    elif invalid == 'negative_required':
        message.buffered_required_percent = -1.0
    elif invalid == 'invalid_required':
        message.buffered_required_percent = math.inf
    elif invalid == 'negative_distance':
        message.home_distance_m = -1.0
    elif invalid == 'invalid_distance':
        message.home_distance_m = math.nan
    elif invalid == 'inconsistent_margin':
        message.return_margin_percent = -5.0
    elif invalid == 'positive_margin':
        message.remaining_percent = 20.0
        message.return_margin_percent = 10.0
    elif invalid == 'no_samples':
        message.sample_count = 0
    elif invalid == 'invalid_rate':
        message.mean_percent_per_meter = math.nan
    elif invalid == 'negative_rate':
        message.mean_percent_per_meter = -0.1
    elif invalid == 'invalid_return':
        message.return_required_percent = math.inf
    elif invalid == 'negative_return':
        message.return_required_percent = -1.0

    harness._battery_alert_callback(message)

    assert harness._requested_mission_state is None


def test_delayed_battery_callback_cannot_abort_a_successor_goal(harness):
    harness._goal_handle = Goal()
    request = harness.request_mission_state

    def successor_started_before_request(state, goal_id=None):
        harness._goal_handle = Goal()
        harness._goal_handle.goal_id = UUID(uuid=[2] * 16)
        return request(state, goal_id=goal_id)

    harness.request_mission_state = successor_started_before_request
    harness._battery_alert_callback(battery_alert(harness))

    assert harness._requested_mission_state is None


@pytest.mark.parametrize('deferred_attribute', ['_command_inflight', '_recovering'])
def test_abort_waits_for_inflight_command_or_recovery(harness, deferred_attribute):
    harness._goal_handle = Goal()
    harness._requested_mission_state = 'Abort'
    setattr(harness, deferred_attribute, True)
    harness._check_running()

    setattr(harness, deferred_attribute, False)
    with pytest.raises(MissionAbortRequested):
        harness._check_running()


def test_abort_during_arming_waits_for_ack_but_does_not_continue_climb(harness):
    configure_takeoff_services(harness)
    harness._goal_handle = Goal()
    call = harness._call

    def abort_after_arm_dispatch(client, *args, **kwargs):
        response = call(client, *args, **kwargs)
        if client is harness._arming_client:
            assert harness.request_mission_state('Abort')
        return response

    harness._call = abort_after_arm_dispatch
    with pytest.raises(MissionAbortRequested):
        harness._takeoff(Blackboard())

    assert harness.state.armed
    assert not harness._command_inflight
    assert harness.odometry.pose.pose.position.z < 14.0
    assert not harness._airborne_ready


@pytest.mark.parametrize('deferred_attribute', ['_command_inflight', '_recovering'])
def test_state_request_is_deferred_during_command_and_recovery(
    harness, deferred_attribute,
):
    harness._goal_handle = Goal()
    harness._requested_mission_state = 'Return'
    setattr(harness, deferred_attribute, True)
    harness._check_running()
    assert harness._requested_mission_state == 'Return'

    setattr(harness, deferred_attribute, False)
    with pytest.raises(MissionStateRequested) as request:
        harness._check_running()
    assert request.value.state == 'Return'


@pytest.mark.parametrize('deferred_attribute', ['_command_inflight', '_recovering'])
def test_client_cancel_has_priority_over_pending_state_request(
    harness, deferred_attribute,
):
    goal = Goal()
    goal.is_cancel_requested = True
    harness._goal_handle = goal
    harness._requested_mission_state = 'Return'
    setattr(harness, deferred_attribute, True)
    harness._check_running()

    setattr(harness, deferred_attribute, False)
    with pytest.raises(MissionCanceled):
        harness._check_running()


@pytest.mark.parametrize('invalid', ['', 'Exploration', 'return', 'Verification', None])
def test_state_request_rejects_unsupported_states(harness, invalid):
    harness._goal_handle = Goal()
    assert harness.request_mission_state(invalid) is False
    assert harness._requested_mission_state is None


@pytest.mark.parametrize('unavailable', ['idle', 'inactive', 'finalizing', 'recovering'])
def test_state_request_rejects_when_goal_cannot_transition(harness, unavailable):
    if unavailable != 'idle':
        harness._goal_handle = Goal()
    if unavailable == 'inactive':
        harness._goal_handle.is_active = False
    elif unavailable == 'finalizing':
        harness._finalizing = True
    elif unavailable == 'recovering':
        harness._recovering = True

    assert harness.request_mission_state('Return') is False
    assert harness._requested_mission_state is None


def test_return_request_takes_priority_over_verify(harness):
    harness._goal_handle = Goal()
    assert harness.request_mission_state('Verify')
    assert harness._requested_mission_state == 'Verify'
    assert harness.request_mission_state('Return')
    assert harness._requested_mission_state == 'Return'
    harness.request_mission_state('Verify')
    assert harness._requested_mission_state == 'Return'


@pytest.mark.parametrize('first', ['Return', 'Verify'])
def test_abort_cannot_be_downgraded_to_another_state_request(harness, first):
    harness._goal_handle = Goal()
    assert harness.request_mission_state(first)
    assert harness.request_mission_state('Abort')
    assert harness._requested_mission_state == 'Abort'
    harness.request_mission_state(first)
    assert harness._requested_mission_state == 'Abort'


def test_verify_entry_consumes_request_stops_motion_and_publishes_phase(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._owns_flight = True
    harness.state.mode = 'OFFBOARD'
    assert harness.request_mission_state('Verify')

    harness._verify_parent_entry('Verify')

    assert harness._parent_state == 'Verify'
    assert harness._requested_mission_state is None
    assert harness.state.mode == 'AUTO.LOITER'
    phase = next(event[1] for event in harness.events if event[0] == 'phase')
    assert phase.state == 'Verify'
    assert not harness.request_mission_state('Verify')
    harness._check_running()
    assert harness._goal_handle.is_active


@pytest.mark.parametrize('state,exception', [
    ('Return', MissionStateRequested), ('Abort', MissionAbortRequested),
])
def test_stop_request_supersedes_verify_before_parent_entry(harness, state, exception):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness.request_mission_state('Verify')
    harness.request_mission_state(state)

    with pytest.raises(exception):
        harness._verify_parent_entry('Verify')

    assert harness._requested_mission_state == state
    assert not any(event[0] == 'phase' for event in harness.events)


def test_verify_entry_requires_confirmed_loiter_before_publishing_phase(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._owns_flight = True
    harness.state.mode = 'OFFBOARD'
    harness.request_mission_state('Verify')

    def reject_loiter():
        raise MissionError('mode refused')

    harness._loiter = reject_loiter
    with pytest.raises(MissionError):
        harness._verify_parent_entry('Verify')

    assert harness.state.mode == 'OFFBOARD'
    assert harness._target == harness.odometry.pose.pose
    assert not any(event[0] == 'phase' for event in harness.events)
    assert not harness._recovering


def test_pending_return_interrupts_exploration_parent_entry(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness.request_mission_state('Verify')
    harness.request_mission_state('Return')

    with pytest.raises(MissionStateRequested) as requested:
        harness._verify_parent_entry('Exploration')

    assert requested.value.state == 'Return'
    assert harness._requested_mission_state == 'Return'
    assert not any(event[0] == 'phase' for event in harness.events)


def test_parent_phase_requires_initialized_ros_clock_before_takeoff(harness):
    harness._goal_handle = Goal()
    harness._airborne_ready = False
    harness.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: TimeMessage()))

    with pytest.raises(MissionError, match='initialized mission ROS clock'):
        harness._verify_parent_entry('Exploration')

    assert not any(event[0] in {'phase', 'service'} for event in harness.events)


def test_exploration_reentry_resets_verify_progress_without_taking_off(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Verify'
    harness._phase_sequence = 2
    blackboard = Blackboard()
    blackboard['verify_entered'] = True
    blackboard['verify_approaches'] = 8
    blackboard['verify_smoke_approach_completed'] = True
    blackboard['verify_smoke_approach_after_ns'] = 99_000_000_000
    blackboard['verify_reference_pose'] = make_pose(100, 100, 100)
    blackboard['verify_smoke_bearing'] = 0.7
    blackboard['verify_arrival_after_ns'] = 100_000_000_000
    blackboard['verify_viewpoint_required'] = True
    blackboard['verify_smoke_walls'] = [{'nearest': 20.0}]
    blackboard['verify_smoke_wall_probe'] = {'nearest': 20.0}
    blackboard['verify_wall_latest'] = {'nearest': 20.0}
    blackboard['verify_wall_last_stamp_ns'] = 42
    blackboard['verify_wall_near_count'] = 2
    blackboard['verify_wall_recheck_deadline'] = 123.0
    blackboard['verify_height_hint'] = 'old target'
    blackboard['verify_height_hint_consumed'] = 42
    blackboard['verify_descent_deferred'] = True

    harness._exploration_parent_entry(blackboard)
    harness._takeoff(blackboard)

    assert not blackboard['verify_entered']
    assert blackboard['verify_approaches'] == 0
    assert not blackboard['verify_smoke_approach_completed']
    assert blackboard['verify_smoke_approach_after_ns'] == 0
    assert blackboard['verify_reference_pose'] is None
    assert blackboard['verify_smoke_bearing'] is None
    assert blackboard['verify_arrival_after_ns'] == 0
    assert not blackboard['verify_viewpoint_required']
    assert blackboard['verify_smoke_walls'] == []
    assert blackboard['verify_smoke_wall_probe'] is None
    assert blackboard['verify_wall_latest'] is None
    assert blackboard['verify_wall_last_stamp_ns'] == 0
    assert blackboard['verify_wall_near_count'] == 0
    assert blackboard['verify_wall_recheck_deadline'] is None
    assert blackboard['verify_height_hint'] is None
    assert blackboard['verify_height_hint_consumed'] == 0
    assert not blackboard['verify_descent_deferred']
    assert harness._verify_retry_after == 0.0
    phase = next(event[1] for event in harness.events if event[0] == 'phase')
    assert phase.state == 'Exploration'
    assert phase.sequence == 3
    assert not any(event[0] == 'service' for event in harness.events)


def test_failed_verify_defers_repeat_detection_only_until_retry_deadline(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Verify'
    board = Blackboard()
    board['verify_exploration_required'] = True
    board['verify_approaches'] = 3

    harness._exploration_parent_entry(board)

    assert harness._parent_state == 'Exploration'
    deadline = harness.config['verify_retry_cooldown']
    assert harness._verify_retry_after == deadline
    assert not board['verify_exploration_required']
    assert board['verify_approaches'] == 0
    assert harness.request_mission_state('Verify')
    harness._check_running()
    assert harness._requested_mission_state == 'Verify'
    harness._stop.clock.now = deadline
    with pytest.raises(MissionStateRequested) as request:
        harness._check_running()
    assert request.value.state == 'Verify'
    assert harness._verify_retry_after == 0.0
    assert not any(event[0] == 'ramp' for event in harness.events)


def test_inconclusive_verify_defers_source_before_exploration_resets_guidance(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Verify'
    board = Blackboard()
    board['verify_exploration_required'] = True
    board['verify_smoke_bearing'] = 0.7
    deferred = []

    def remember_viewpoint():
        deferred.append((harness._parent_state, board['verify_smoke_bearing']))
        return True

    harness._verify_defer_viewpoint = remember_viewpoint
    harness._exploration_parent_entry(board)

    assert deferred == [('Verify', 0.7)]
    assert harness._parent_state == 'Exploration'
    assert board['verify_smoke_bearing'] is None


def test_normal_exploration_entry_does_not_defer_uninspected_source(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Verify'
    harness._verify_defer_viewpoint = lambda: pytest.fail('No inconclusive search occurred')

    harness._exploration_parent_entry(Blackboard())

    assert harness._parent_state == 'Exploration'


def test_pending_automatic_verify_is_rechecked_after_viewpoint_becomes_deferred(
        harness, monkeypatch):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Exploration'
    deferred = [False]
    monkeypatch.setattr(
        executive_module.CompletedSitesMixin, '_verify_request_is_deferred',
        lambda _self, _goal_id: deferred[0])
    assert harness.request_mission_state('Verify', goal_id=harness._goal_handle.goal_id)

    deferred[0] = True
    harness._check_running()

    assert harness._requested_mission_state is None
    assert not harness._requested_verify_automatic
    assert harness._parent_state == 'Exploration'


def test_pending_manual_verify_never_uses_automatic_dispatch_veto(harness, monkeypatch):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Exploration'
    assert harness.request_mission_state('Verify')

    def unexpected_veto(*_args):
        pytest.fail('Manual Verify must bypass the automatic source association')

    monkeypatch.setattr(
        executive_module.CompletedSitesMixin, '_verify_request_is_deferred', unexpected_veto)

    with pytest.raises(MissionStateRequested) as request:
        harness._check_running()

    assert request.value.state == 'Verify'
    assert harness._requested_mission_state == 'Verify'
    assert not harness._requested_verify_automatic


@pytest.mark.parametrize('replacement', ['new_goal', 'manual'])
def test_dispatch_recheck_cannot_clear_a_replacement_verify_request(
        harness, monkeypatch, replacement):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Exploration'
    assert harness.request_mission_state('Verify', goal_id=harness._goal_handle.goal_id)

    def source_association(_self, goal_id):
        assert tuple(goal_id.uuid) == (1,) * 16
        if replacement == 'new_goal':
            harness._goal_handle = Goal(skip_takeoff=True)
            harness._goal_handle.goal_id = UUID(uuid=[2] * 16)
        else:
            harness._requested_verify_automatic = False
        return True

    monkeypatch.setattr(
        executive_module.CompletedSitesMixin, '_verify_request_is_deferred', source_association)

    with pytest.raises(MissionStateRequested) as request:
        harness._check_running()

    assert request.value.state == 'Verify'
    assert harness._requested_mission_state == 'Verify'
    assert harness._requested_verify_automatic == (replacement == 'new_goal')


@pytest.mark.parametrize('state', ['Abort', 'Return'])
def test_dispatch_source_association_cannot_erase_a_new_higher_priority_request(
        harness, monkeypatch, state):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Exploration'
    assert harness.request_mission_state('Verify', goal_id=harness._goal_handle.goal_id)

    def source_association(_self, _goal_id):
        assert harness.request_mission_state(state)
        return True

    monkeypatch.setattr(
        executive_module.CompletedSitesMixin, '_verify_request_is_deferred', source_association)
    exception = MissionAbortRequested if state == 'Abort' else MissionStateRequested

    with pytest.raises(exception):
        harness._check_running()

    assert harness._requested_mission_state == state
    assert not harness._requested_verify_automatic


def test_retry_deadline_interrupts_first_sweep_turn_without_waiting_for_path(motion_harness):
    harness = motion_harness
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Verify'
    board = Blackboard()
    board['verify_exploration_required'] = True
    harness._exploration_parent_entry(board)
    deadline = harness._verify_retry_after

    def detect_during_sweep():
        if harness._stop.clock.now >= 0.5:
            assert harness.request_mission_state('Verify')

    harness._stop.on_wait = detect_during_sweep

    with pytest.raises(MissionStateRequested) as request:
        harness._sweep(Blackboard())

    assert request.value.state == 'Verify'
    assert deadline <= harness._stop.clock.now <= deadline + 0.1
    assert harness._requested_mission_state == 'Verify'
    assert harness._verify_retry_after == 0.0
    assert len([event for event in harness.events if event[0] == 'ramp_started']) == 1
    assert not any(event[0] == 'ramp_finished' for event in harness.events)
    assert not any(event[0] == 'service' for event in harness.events)


def test_finishing_a_short_path_does_not_clear_retry_delay_early(harness):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Exploration'
    deadline = harness.config['verify_retry_cooldown']
    harness._verify_retry_after = deadline
    assert harness.request_mission_state('Verify')
    path = Path(header=Header(frame_id='px4_odom'), poses=[
        PoseStamped(pose=make_pose(1, 2, 3)),
        PoseStamped(pose=make_pose(7, 2, 3)),
    ])
    harness._call = lambda *args, **kwargs: PlanPath.Response(success=True, path=path)
    harness._execute_path(Blackboard())

    assert harness._verify_retry_after == deadline
    assert harness.state.mode == 'AUTO.LOITER'
    assert harness._requested_mission_state == 'Verify'
    harness._check_running()
    harness._stop.clock.now = deadline
    with pytest.raises(MissionStateRequested) as request:
        harness._check_running()
    assert request.value.state == 'Verify'


@pytest.mark.parametrize('failure', ['first_leg', 'destination_loiter'])
def test_retry_deadline_is_independent_of_incomplete_exploration(harness, failure):
    harness._goal_handle = Goal(skip_takeoff=True)
    harness._parent_state = 'Exploration'
    deadline = harness.config['verify_retry_cooldown']
    harness._verify_retry_after = deadline
    assert harness.request_mission_state('Verify')
    path = Path(header=Header(frame_id='px4_odom'), poses=[
        PoseStamped(pose=make_pose(1, 2, 3)),
        PoseStamped(pose=make_pose(7, 2, 3)),
    ])
    harness._call = lambda *args, **kwargs: PlanPath.Response(success=True, path=path)
    report = harness._report

    def fail_after_first_leg(message, error=False):
        report(message, error)
        if message == 'EXECUTE_PATH: sub-path 1/3 done':
            raise MissionError('Controlled incomplete Exploration path')

    def fail_destination_loiter():
        raise MissionError('Controlled incomplete Exploration path')

    if failure == 'first_leg':
        harness._report = fail_after_first_leg
    else:
        harness._loiter = fail_destination_loiter

    with pytest.raises(MissionError, match='Controlled incomplete Exploration path'):
        harness._execute_path(Blackboard())

    assert harness._verify_retry_after == deadline
    assert harness._requested_mission_state == 'Verify'
    harness._check_running()
    if failure == 'first_leg':
        assert xyz(harness.odometry.pose.pose) != pytest.approx((7, 2, 3))
    harness._stop.clock.now = deadline
    with pytest.raises(MissionStateRequested) as request:
        harness._check_running()
    assert request.value.state == 'Verify'


@pytest.mark.parametrize('interruption', ['cancel', 'Abort', 'Return'])
def test_verify_retry_cooldown_preserves_stop_request_priority(harness, interruption):
    goal = Goal(skip_takeoff=True)
    harness._goal_handle = goal
    harness._parent_state = 'Exploration'
    harness._verify_retry_after = harness.config['verify_retry_cooldown']
    assert harness.request_mission_state('Verify')
    harness._check_running()
    if interruption == 'cancel':
        goal.is_cancel_requested = True
        exception = MissionCanceled
    else:
        assert harness.request_mission_state(interruption)
        exception = MissionAbortRequested if interruption == 'Abort' else MissionStateRequested

    with pytest.raises(exception) as stopped:
        harness._wait_until(lambda: False, 10.0, 'Exploration motion')

    if interruption == 'Return':
        assert stopped.value.state == 'Return'
    assert harness._verify_retry_after == harness.config['verify_retry_cooldown']
    assert harness._stop.clock.now == 0.0


def test_state_request_topic_uses_same_transition_entry_point(harness):
    harness._goal_handle = Goal()
    harness._mission_state_request_callback(String(data='Verify'))
    assert harness._requested_mission_state == 'Verify'


@pytest.mark.parametrize('state', ['Abort', 'Return', 'Verify'])
def test_goal_state_request_rejects_previous_goal_uuid(harness, state):
    goal = Goal()
    goal.goal_id = UUID(uuid=[2] * 16)
    harness._goal_handle = goal
    harness._goal_state_request_callback(MissionStateRequest(
        goal_id=UUID(uuid=[1] * 16), state=state,
    ))
    assert harness._requested_mission_state is None

    harness._goal_state_request_callback(MissionStateRequest(goal_id=goal.goal_id, state=state))
    assert harness._requested_mission_state == state


def test_new_goal_clears_previous_state_request(harness):
    harness._requested_mission_state = 'Return'
    harness._verify_retry_after = 10.0
    executed = []
    goal = SimpleNamespace(execute=lambda: executed.append(True))

    harness._goal_accepted_callback(goal)

    assert harness._requested_mission_state is None
    assert harness._verify_retry_after == 0.0
    assert harness._goal_handle is goal
    assert executed == [True]


def test_return_supersedes_verify_between_machine_exit_and_recovery(harness, monkeypatch):
    def request_verify_then_return(blackboard):
        assert harness.request_mission_state('Verify')
        with pytest.raises(MissionStateRequested) as observed:
            harness._check_running()
        assert observed.value.outcome == 'verify_requested'
        assert harness.request_mission_state('Return')
        return observed.value.outcome

    def recover():
        assert harness._recovering
        assert not harness.request_mission_state('Verify')
        return ''

    monkeypatch.setattr(
        executive_module, 'build_state_machine',
        lambda *args, **kwargs: request_verify_then_return,
    )
    harness._recover_to_loiter = recover
    goal = Goal(skip_takeoff=True)

    result = run_goal(harness, goal)

    assert goal.outcome == 'aborted'
    assert not result.success
    assert 'Return is not implemented' in result.message
    assert 'Verify is not implemented' not in result.message
    assert harness._requested_mission_state is None
    assert not harness._mission_lock.locked()


@pytest.mark.parametrize('state', ['Abort', 'Return'])
@pytest.mark.parametrize('loiter_acknowledged', [True, False])
def test_stop_request_aborts_recovers_and_clears_request(
    harness, state, loiter_acknowledged,
):
    goal = Goal(skip_takeoff=True)

    def request_during_sweep(blackboard):
        harness.state.mode = 'OFFBOARD'
        harness._target = make_pose(100, 100, 100)
        assert harness.request_mission_state(state)
        harness._check_running()

    def unexpected_path(blackboard):
        raise AssertionError('Path must not run after a transition request')

    harness._sweep = request_during_sweep
    harness._execute_path = unexpected_path
    if not loiter_acknowledged:
        def reject_loiter():
            raise MissionError('mode refused')

        harness._loiter = reject_loiter

    result = run_goal(harness, goal)

    assert goal.outcome == 'aborted'
    assert not result.success
    expected = {
        'Abort': 'aborted by a monitor request',
        'Return': 'Return is not implemented',
    }[state]
    assert expected in result.message
    assert harness._requested_mission_state is None
    assert harness._goal_handle is None
    assert harness._state_machine is None
    assert not harness._mission_lock.locked()
    if loiter_acknowledged:
        assert harness.state.mode == 'AUTO.LOITER'
        assert harness._target is None
    else:
        assert harness.state.mode == 'OFFBOARD'
        assert harness._target == harness.odometry.pose.pose
        assert harness._recovery_hold
        assert 'retaining the local hold setpoint' in result.message


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
    assert harness._requested_mission_state is None
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
