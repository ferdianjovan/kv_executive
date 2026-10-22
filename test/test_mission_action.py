"""Exercise cancelable mission goals through DDS and a simulated FCU."""

from copy import deepcopy
import math
import os
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from kv_executive.action import ExecuteMission
from kv_executive.mission_executive import MissionExecutive
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
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Header, String


class SimulatedFcu(Node):
    """Respond through DDS; no production callbacks or waits are replaced."""

    def __init__(self):
        super().__init__('mission_action_test_fcu')
        self.callbacks = ReentrantCallbackGroup()
        self.telemetry_callbacks = MutuallyExclusiveCallbackGroup()
        self.setpoint_callbacks = MutuallyExclusiveCallbackGroup()
        self.lock = threading.Lock()
        self.state = VehicleState(connected=True, armed=False, mode='MANUAL')
        self.pose = Pose(
            position=Point(x=1.0, y=-2.0, z=7.0),
            orientation=quaternion_from_yaw(math.radians(170.0)),
        )
        self.takeoff_height = 1.0
        self.takeoff_z = self.pose.position.z + self.takeoff_height
        self.climb_enabled = True
        self.reject_modes = set()
        self.planner_success = True
        self.planning_started = threading.Event()
        self.planning_release = threading.Event()
        self.planning_release.set()
        self.pending_mode = None
        self.setpoints = []
        self.last_setpoint = 0.0
        self.offboard_loss_timeout = 0.5
        self.offboard_losses = []
        self.statuses = []
        self.mode_requests = []
        self.mode_waits = []
        self.parameter_requests = []
        self.arming_requests = []
        self.planning_requests = 0
        # MAVROS emits state on the FCU heartbeat, independently of the much
        # faster local odometry stream. Keep their delivery controls separate.
        self.heartbeat_interval = 1.0
        self.heartbeat_paused_until = 0.0
        self.next_heartbeat = 0.0
        self.state_enabled = True
        self.odometry_enabled = True
        self.state_publications = []
        self.status_publisher = self.create_publisher(
            VehicleState, '/mavros/state',
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.odometry_publisher = self.create_publisher(
            Odometry, '/mavros/local_position/odom', qos_profile_sensor_data,
        )
        self.setpoint_subscription = self.create_subscription(
            PoseStamped, '/mavros/setpoint_position/local', self.receive_setpoint,
            100, callback_group=self.setpoint_callbacks,
        )
        self.status_subscription = self.create_subscription(
            String, 'mission_status', self.receive_status,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.callbacks,
        )
        self.fake_services = [
            self.create_service(
                SetMode, '/mavros/set_mode', self.set_mode,
                callback_group=self.callbacks,
            ),
            self.create_service(
                CommandBool, '/mavros/cmd/arming', self.arm,
                callback_group=self.callbacks,
            ),
            self.create_service(
                ParamPull, '/mavros/param/pull', self.pull_parameters,
                callback_group=self.callbacks,
            ),
            self.create_service(
                GetParameters, '/mavros/param/get_parameters', self.read_fcu_parameters,
                callback_group=self.callbacks,
            ),
            self.create_service(
                PlanPath, 'plan_path', self.plan_path,
                callback_group=self.callbacks,
            ),
        ]
        self.telemetry_timer = self.create_timer(
            0.05, self.publish_telemetry, callback_group=self.telemetry_callbacks,
        )
        self.mission_client = ActionClient(
            self, ExecuteMission, 'execute_mission', callback_group=self.callbacks,
        )

    def publish_telemetry(self):
        with self.lock:
            now = time.monotonic()
            if self.pending_mode and now >= self.pending_mode[0]:
                self.state.mode = self.pending_mode[1]
                self.pending_mode = None
            if (
                self.state.armed and self.state.mode == 'OFFBOARD'
                and now - self.last_setpoint > self.offboard_loss_timeout
            ):
                self.offboard_losses.append((now, now - self.last_setpoint))
                self.state.mode = 'AUTO.RTL'
            if self.climb_enabled and self.state.mode == 'AUTO.TAKEOFF' and self.state.armed:
                self.pose.position.z = min(self.takeoff_z, self.pose.position.z + 0.2)
            publish_state = (
                self.state_enabled and now >= self.next_heartbeat
                and now >= self.heartbeat_paused_until
            )
            if publish_state:
                self.next_heartbeat = now + self.heartbeat_interval
                self.state_publications.append(now)
            state = deepcopy(self.state)
            publish_odometry = self.odometry_enabled
            odometry = Odometry(
                header=Header(
                    stamp=self.get_clock().now().to_msg(), frame_id='px4_odom',
                ),
                child_frame_id='kv_drone/base_link',
            )
            odometry.pose.pose = deepcopy(self.pose)
        if publish_state:
            self.status_publisher.publish(state)
        if publish_odometry:
            self.odometry_publisher.publish(odometry)

    def receive_setpoint(self, message):
        with self.lock:
            self.last_setpoint = time.monotonic()
            self.setpoints.append((
                self.last_setpoint, self.state.mode, deepcopy(message),
            ))
            if self.state.mode == 'OFFBOARD':
                self.pose = deepcopy(message.pose)

    def receive_status(self, message):
        with self.lock:
            self.statuses.append((time.monotonic(), message.data))

    def set_mode(self, request, response):
        with self.lock:
            self.mode_requests.append(request.custom_mode)
            before = len(self.setpoints)
        # A real service can take time, and mode_sent precedes telemetry ack.
        # The production setpoint timer must keep running through both waits.
        time.sleep(0.2)
        with self.lock:
            after = len(self.setpoints)
            self.mode_waits.append((request.custom_mode, before, after))
            accepted = request.custom_mode not in self.reject_modes
            if accepted:
                # Service reply plus telemetry acknowledgement exceeds the
                # OFFBOARD watchdog: setpoints must continue across both.
                self.pending_mode = (time.monotonic() + 0.4, request.custom_mode)
        response.mode_sent = accepted
        return response

    def arm(self, request, response):
        with self.lock:
            self.arming_requests.append((request.value, self.state.mode))
            self.state.armed = request.value
        response.success = True
        response.result = 0
        return response

    def pull_parameters(self, request, response):
        response.success = True
        response.param_received = 1
        return response

    def read_fcu_parameters(self, request, response):
        with self.lock:
            self.parameter_requests.append(list(request.names))
        response.values = [ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=self.takeoff_height,
        )]
        return response

    def plan_path(self, request, response):
        with self.lock:
            self.planning_requests += 1
            start = deepcopy(self.pose)
        self.planning_started.set()
        self.planning_release.wait(5.0)
        path = Path(header=Header(frame_id='px4_odom'))
        # Uneven waypoint spacing checks that the three legs have equal length.
        for distance in (0.0, 0.05, 0.25, 0.6):
            pose = deepcopy(start)
            pose.position.x += distance
            path.poses.append(PoseStamped(
                header=Header(frame_id='px4_odom'), pose=pose,
            ))
        response.success = self.planner_success
        response.message = 'Simulated local route' if self.planner_success else 'No route found'
        response.path = path
        return response


def wait_for(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        threading.Event().wait(0.01)
    pytest.fail(f'Timed out waiting for {description}')


def path_length(path):
    return math.fsum(
        math.dist(
            (start.pose.position.x, start.pose.position.y, start.pose.position.z),
            (end.pose.position.x, end.pose.position.y, end.pose.position.z),
        )
        for start, end in zip(path.poses, path.poses[1:])
    )


def received_state(mission, *, mode=None, armed=None):
    """Wait on what the executive has observed, rather than FCU internal state."""
    with mission.executive._data_lock:
        state = mission.executive._vehicle_state
        return (
            state is not None
            and (mode is None or state.mode == mode)
            and (armed is None or state.armed == armed)
        )


def prepare_airborne(mission):
    with mission.fcu.lock:
        mission.fcu.state.armed = True
        mission.fcu.state.mode = 'AUTO.LOITER'
    wait_for(
        lambda: received_state(mission, mode='AUTO.LOITER', armed=True),
        2.0, 'airborne LOITER heartbeat',
    )


@pytest.fixture
def mission(monkeypatch, tmp_path):
    # Keep this test's simulated MAVROS services isolated from other ROS graphs.
    monkeypatch.setenv('ROS_DOMAIN_ID', str(40 + os.getpid() % 50))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    parameters = {
        'setpoint_rate': 30.0,
        'offboard_warmup': 1.0,
        'sweep_step_duration': 0.15,
        'hold_duration': 0.05,
        'path_speed': 3.0,
        'position_tolerance': 0.03,
        'takeoff_altitude_tolerance': 0.02,
        'service_timeout': 3.0,
        'planning_timeout': 3.0,
        'mode_timeout': 3.0,
        'takeoff_timeout': 3.0,
        'waypoint_timeout': 3.0,
    }
    namespace = f'/mission_test_{os.getpid()}'
    arguments = ['--ros-args', '-r', f'__ns:={namespace}']
    for topic_or_service in (
        'state', 'statustext/recv', 'local_position/odom', 'setpoint_position/local',
        'set_mode', 'cmd/arming', 'param/pull', 'param/get_parameters',
    ):
        name = f'/mavros/{topic_or_service}'
        arguments.extend(['-r', f'{name}:={namespace}{name}'])
    for name, value in parameters.items():
        arguments.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=arguments)
    executive = None
    fcu = None
    executor = None
    thread = None
    try:
        executive = MissionExecutive()
        fcu = SimulatedFcu()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(executive)
        executor.add_node(fcu)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        wait_for(
            lambda: fcu.mission_client.server_is_ready() and executive._telemetry_ready(),
            3.0, 'mission action and telemetry discovery',
        )
        yield SimpleNamespace(executive=executive, fcu=fcu)
    finally:
        if executive is not None:
            executive.stop_mission()
            executive._setpoint_timer.cancel()
        if fcu is not None:
            fcu.planning_release.set()
            fcu.telemetry_timer.cancel()
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if thread is not None:
            thread.join(timeout=3.0)
        if executor is not None:
            # Some Jazzy rclpy versions leave queued worker callbacks running
            # after Executor.shutdown(); drain them before destroying nodes.
            callback_pool = getattr(executor, '_executor', None)
            if callback_pool is not None:
                callback_pool.shutdown(wait=True)
        if executive is not None:
            executive.destroy_node()
        if fcu is not None:
            fcu.destroy_node()
        rclpy.shutdown()


def send_goal(mission, skip_takeoff=False, feedback=None):
    callback = None if feedback is None else lambda event: feedback.append(event.feedback)
    future = mission.fcu.mission_client.send_goal_async(
        ExecuteMission.Goal(skip_takeoff=skip_takeoff), feedback_callback=callback,
    )
    wait_for(future.done, 3.0, 'goal acknowledgement')
    return future.result()


def mission_result(goal):
    future = goal.get_result_async()
    wait_for(future.done, 20.0, 'terminal mission action result')
    return future.result()


def cancel_goal(goal):
    future = goal.cancel_goal_async()
    wait_for(future.done, 3.0, 'cancel acknowledgement')
    assert len(future.result().goals_canceling) == 1
    result = mission_result(goal)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not result.result.success
    return result


def test_action_completes_and_accepts_another_airborne_goal(mission):
    executive, fcu = mission.executive, mission.fcu
    feedback = []
    goal = send_goal(mission, feedback=feedback)
    assert goal.accepted
    response = mission_result(goal)
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.success, response.result.message
    assert 'AUTO.LOITER' in response.result.message
    wait_for(
        lambda: any(text.startswith('MISSION: done') for _, text in fcu.statuses),
        1.0, 'mission completion status',
    )

    with fcu.lock:
        statuses = list(fcu.statuses)
        setpoints = list(fcu.setpoints)
        mode_waits = list(fcu.mode_waits)
        final_pose = deepcopy(fcu.pose)
        assert fcu.state.mode == 'AUTO.LOITER'
        assert fcu.state.armed
        assert fcu.mode_requests == [
            'AUTO.TAKEOFF', 'OFFBOARD', 'AUTO.LOITER', 'OFFBOARD', 'AUTO.LOITER',
        ]
        assert fcu.parameter_requests == [['MIS_TAKEOFF_ALT']]
        assert fcu.arming_requests == [(True, 'AUTO.TAKEOFF')]
        assert fcu.planning_requests == 1
        assert not fcu.offboard_losses

    messages = [text for _, text in statuses]
    assert [text for text in messages if text.endswith((': started', ': done'))] == [
        'TAKEOFF: started', 'TAKEOFF: done', 'SWEEP: started', 'SWEEP: done',
        'EXECUTE_PATH: started', 'EXECUTE_PATH: done',
    ]
    assert [text for text in messages if text.startswith('SWEEP: step')] == [
        f'SWEEP: step {step}/6 reached' for step in range(1, 7)
    ]
    assert [text for text in messages if text.startswith('EXECUTE_PATH: sub-path')] == [
        f'EXECUTE_PATH: sub-path {leg}/3 done' for leg in range(1, 4)
    ]
    assert {'TAKEOFF', 'SWEEP', 'EXECUTE_PATH'} <= {event.state for event in feedback}
    assert all(event.message for event in feedback)
    assert len(executive.sub_paths) == 3
    assert [path_length(leg) for leg in executive.sub_paths] == pytest.approx([0.2] * 3)
    assert final_pose.position.x == pytest.approx(1.6, abs=0.03)
    assert final_pose.position.y == pytest.approx(-2.0, abs=0.03)
    assert final_pose.position.z == pytest.approx(8.0, abs=0.03)
    assert yaw_from_quaternion(final_pose.orientation) == pytest.approx(0.0, abs=1e-6)
    assert all(message.header.frame_id == 'px4_odom' for _, _, message in setpoints)

    sweep_done = next(stamp for stamp, text in statuses if text == 'SWEEP: done')
    sweep_setpoints = [message for stamp, _, message in setpoints if stamp < sweep_done]
    headings = [yaw_from_quaternion(message.pose.orientation) for message in sweep_setpoints]
    rotation = math.fsum(
        angular_distance(current, previous)
        for previous, current in zip(headings, headings[1:])
    )
    assert rotation == pytest.approx(math.tau, abs=0.02)
    assert all(
        message.pose.position.z == pytest.approx(8.0, abs=0.03)
        for message in sweep_setpoints
    )
    path_started = next(stamp for stamp, text in statuses if text == 'EXECUTE_PATH: started')
    path_setpoints = [
        message for stamp, mode, message in setpoints
        if stamp >= path_started and mode == 'OFFBOARD'
    ]
    translating = []
    for previous, current in zip(path_setpoints, path_setpoints[1:]):
        dx = current.pose.position.x - previous.pose.position.x
        dy = current.pose.position.y - previous.pose.position.y
        if math.hypot(dx, dy) <= 1e-6:
            continue  # OFFBOARD warmup, stationary holds and yaw-only turns.
        heading = math.atan2(dy, dx)
        for message in (previous, current):
            yaw = yaw_from_quaternion(message.pose.orientation)
            assert abs(angular_distance(yaw, heading)) < 1e-6
        translating.append(current)
    assert translating, 'The path must contain observed horizontal movement'
    # Independent setpoint callbacks continue through delayed mode replies.
    assert all(after > before for mode, before, after in mode_waits
               if mode in {'OFFBOARD', 'AUTO.LOITER'}), mode_waits
    assert mode_waits[1][1] >= 20
    for state_name in ('SWEEP', 'EXECUTE_PATH'):
        started = next(stamp for stamp, text in statuses if text == f'{state_name}: started')
        finished = next(stamp for stamp, text in statuses if text == f'{state_name}: done')
        publications = [stamp for stamp, _, _ in setpoints if started <= stamp <= finished]
        assert len(publications) >= 30
        assert max(b - a for a, b in zip(publications, publications[1:])) < 0.5

    # Once LOITER owns flight, ending the stream must not cause a return home.
    wait_for(lambda: time.monotonic() - fcu.last_setpoint > 0.1,
             1.0, 'completed mission setpoint stream to stop')
    with fcu.lock:
        completed_count = len(fcu.setpoints)
    threading.Event().wait(fcu.offboard_loss_timeout + 0.1)
    with fcu.lock:
        assert len(fcu.setpoints) == completed_count
        assert fcu.pose == final_pose
        assert fcu.state.mode == 'AUTO.LOITER'
        assert not fcu.offboard_losses
    assert executive._target is None

    # The result releases the mission slot, and an armed drone can run again.
    future = executive.send_mission_goal(skip_takeoff=True)
    wait_for(future.done, 3.0, 'internal mission goal acknowledgement')
    repeat = future.result()
    assert repeat.accepted
    result = mission_result(repeat)
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.success, result.result.message
    with fcu.lock:
        assert fcu.state.mode == 'AUTO.LOITER'
        assert fcu.pose.position.x == pytest.approx(2.2, abs=0.03)
        assert fcu.pose.position.z == pytest.approx(8.0, abs=0.03)
        assert yaw_from_quaternion(fcu.pose.orientation) == pytest.approx(0.0, abs=1e-6)
        assert fcu.arming_requests == [(True, 'AUTO.TAKEOFF')]
        assert not {'AUTO.RTL', 'AUTO.LAND'} & set(fcu.mode_requests)
        assert not fcu.offboard_losses


@pytest.mark.parametrize('stage', ['takeoff', 'sweep', 'planning', 'path'])
def test_cancel_interrupts_each_flight_stage_and_releases_goal_slot(mission, stage):
    fcu = mission.fcu
    if stage == 'takeoff':
        fcu.climb_enabled = False
    elif stage == 'planning':
        fcu.planning_release.clear()
    goal = send_goal(mission)
    assert goal.accepted
    if stage == 'takeoff':
        wait_for(lambda: received_state(mission, armed=True), 4.0, 'takeoff climb')
    elif stage == 'sweep':
        wait_for(lambda: received_state(mission, mode='OFFBOARD'), 6.0, 'survey OFFBOARD')
    elif stage == 'planning':
        wait_for(fcu.planning_started.is_set, 12.0, 'pending planner response')
    else:
        wait_for(lambda: fcu.mode_requests.count('OFFBOARD') >= 2
                 and received_state(mission, mode='OFFBOARD'), 16.0, 'path OFFBOARD')

    occupied = send_goal(mission, skip_takeoff=True)
    assert not occupied.accepted
    started = time.monotonic()
    cancel_goal(goal)
    assert time.monotonic() - started < 3.0
    fcu.planning_release.set()
    with fcu.lock:
        assert fcu.state.mode == 'AUTO.LOITER'
        assert fcu.state.armed
        assert fcu.planning_requests == (1 if stage in {'planning', 'path'} else 0)
        assert not {'AUTO.RTL', 'AUTO.LAND'} & set(fcu.mode_requests)
        assert not fcu.offboard_losses

    next_goal = send_goal(mission, skip_takeoff=True)
    assert next_goal.accepted
    # Exercise the same cancellation entry point available to executive_node.
    future = mission.executive.cancel_mission()
    assert future is not None
    wait_for(future.done, 3.0, 'internal cancel acknowledgement')
    result = mission_result(next_goal)
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not fcu.offboard_losses


def test_planner_failure_aborts_and_action_server_remains_available(mission):
    fcu = mission.fcu
    fcu.planner_success = False
    goal = send_goal(mission)
    assert goal.accepted
    result = mission_result(goal)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert 'No route found' in result.result.message
    assert fcu.state.mode == 'AUTO.LOITER'
    assert not fcu.offboard_losses
    next_goal = send_goal(mission, skip_takeoff=True)
    assert next_goal.accepted
    cancel_goal(next_goal)


def test_cancel_keeps_offboard_hold_when_px4_rejects_loiter(mission):
    fcu = mission.fcu
    goal = send_goal(mission)
    assert goal.accepted
    wait_for(lambda: received_state(mission, mode='OFFBOARD'), 6.0, 'survey OFFBOARD')
    with fcu.lock:
        fcu.reject_modes.add('AUTO.LOITER')
    result = cancel_goal(goal)
    assert 'AUTO.LOITER' in result.result.message
    with fcu.lock:
        before = len(fcu.setpoints)
    wait_for(lambda: len(fcu.setpoints) >= before + 15, 2.0, 'recovery hold stream')
    with fcu.lock:
        hold_messages = [message for _, _, message in fcu.setpoints[before:]]
        assert fcu.state.mode == 'OFFBOARD'
        assert fcu.state.armed
        assert not {'AUTO.RTL', 'AUTO.LAND'} & set(fcu.mode_requests)
        assert not fcu.offboard_losses
    assert all(message.pose == hold_messages[0].pose for message in hold_messages)
    # External takeover releases the retained OFFBOARD hold.
    with fcu.lock:
        fcu.state.mode = 'AUTO.LOITER'
        fcu.reject_modes.clear()
    wait_for(lambda: mission.executive._target is None, 2.0, 'LOITER takeover')
    next_goal = send_goal(mission, skip_takeoff=True)
    assert next_goal.accepted
    cancel_goal(next_goal)


def test_offboard_loss_aborts_without_overriding_px4_return_mode(mission):
    executive, fcu = mission.executive, mission.fcu
    goal = send_goal(mission)
    assert goal.accepted
    wait_for(lambda: received_state(mission, mode='OFFBOARD'), 6.0, 'survey OFFBOARD')
    # Simulate a dead publisher. The FCU, rather than the executive, requests RTL.
    executive._setpoint_timer.cancel()
    result = mission_result(goal)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert 'AUTO.RTL' in result.result.message
    with fcu.lock:
        assert len(fcu.offboard_losses) == 1
        assert fcu.offboard_losses[0][1] > fcu.offboard_loss_timeout
        assert fcu.state.mode == 'AUTO.RTL'
        assert fcu.mode_requests == ['AUTO.TAKEOFF', 'OFFBOARD']
    assert executive._target is None
    assert not executive._mission_lock.locked()


def test_heartbeat_gap_does_not_abort_sweep_with_live_odometry(mission):
    executive, fcu = mission.executive, mission.fcu
    executive.config['sweep_step_duration'] = 0.7
    prepare_airborne(mission)
    goal = send_goal(mission, skip_takeoff=True)
    assert goal.accepted
    wait_for(lambda: received_state(mission, mode='OFFBOARD'), 4.0, 'survey OFFBOARD')
    with fcu.lock:
        fcu.heartbeat_paused_until = time.monotonic() + 2.6
        previous_heartbeat = fcu.state_publications[-1]
    future = goal.get_result_async()
    wait_for(
        lambda: time.monotonic() - previous_heartbeat > 2.2,
        2.5, 'heartbeat gap beyond the former two-second timeout',
    )
    assert not future.done()
    assert executive._telemetry_ready()
    with executive._data_lock:
        assert time.monotonic() - executive._odometry_received < 0.2
    with fcu.lock:
        assert fcu.state_publications[-1] == previous_heartbeat
        assert not fcu.offboard_losses
        fcu.heartbeat_paused_until = 0.0
    result = cancel_goal(goal)
    assert 'AUTO.LOITER confirmed' in result.result.message
    assert not fcu.offboard_losses


@pytest.mark.parametrize('failure', ['odometry', 'state', 'disconnected'])
def test_telemetry_failure_aborts_sweep_and_retains_last_hold(mission, failure):
    executive, fcu = mission.executive, mission.fcu
    executive.config['sweep_step_duration'] = 1.0
    prepare_airborne(mission)
    goal = send_goal(mission, skip_takeoff=True)
    assert goal.accepted
    wait_for(lambda: received_state(mission, mode='OFFBOARD'), 4.0, 'survey OFFBOARD')
    if failure == 'odometry':
        executive.config['telemetry_timeout'] = 0.35
    elif failure == 'state':
        executive.config['state_timeout'] = 1.5
    with executive._data_lock:
        old_odometry = deepcopy(executive._odometry.pose.pose)
    with fcu.lock:
        fcu.reject_modes.add('AUTO.LOITER')
        if failure == 'odometry':
            fcu.odometry_enabled = False
        elif failure == 'state':
            fcu.state_enabled = False
        else:
            fcu.state.connected = False
            fcu.next_heartbeat = 0.0
    result = mission_result(goal)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    message = result.result.message.lower()
    failed_topic = (
        '/mavros/local_position/odom' if failure == 'odometry' else '/mavros/state'
    )
    assert failed_topic in message
    assert ('disconnected' if failure == 'disconnected' else 'stale') in message
    assert not executive._mission_lock.locked()
    with executive._data_lock:
        target = deepcopy(executive._target)
    assert target is not None
    if failure == 'odometry':
        # Recovery must retain the latest setpoint rather than rewind the
        # ongoing sweep to the pose from the now-stale odometry message.
        assert abs(angular_distance(
            yaw_from_quaternion(target.orientation),
            yaw_from_quaternion(old_odometry.orientation),
        )) > 0.1
    with fcu.lock:
        before = len(fcu.setpoints)
    wait_for(lambda: len(fcu.setpoints) >= before + 5, 1.0, 'retained recovery hold')
    with fcu.lock:
        assert all(message.pose == target for _, _, message in fcu.setpoints[before:])
        assert fcu.state.mode == 'OFFBOARD'
        assert not fcu.offboard_losses
