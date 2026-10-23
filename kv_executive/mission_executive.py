#!/usr/bin/env python3
"""
Cancelable ROS action for a YASMIN takeoff, sweep and planned flight mission.

Run this node with a MultiThreadedExecutor: mission states wait for telemetry
and service replies while independent callbacks keep OFFBOARD setpoints alive.
"""

from copy import deepcopy
import math
import threading
import time

from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Pose, PoseStamped
from kv_executive.action import ExecuteMission
from kv_executive.mission_geometry import (
    angular_distance,
    interpolate_pose,
    quaternion_from_yaw,
    relative_transform,
    split_path,
    transform_pose,
    yaw_from_quaternion,
)
from kv_executive.srv import PlanPath
from mavros_msgs.msg import State as VehicleState, StatusText
from mavros_msgs.srv import CommandBool, ParamPull, SetMode
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from yasmin import Blackboard, State, StateMachine


class MissionError(RuntimeError):
    """An operation failed or timed out; stop advancing the mission."""


class MissionCanceled(MissionError):
    """A client requested cancellation; stop motion and hand over to LOITER."""


class _MissionState(State):
    def __init__(self, executive, name, action):
        super().__init__(['succeeded', 'aborted', 'canceled'])
        self.executive = executive
        self.name = name
        self.action = action

    def execute(self, blackboard):
        self.executive._report(f'{self.name}: started')
        try:
            self.executive._check_running()
            if self.is_canceled():
                raise MissionCanceled('Mission canceled')
            self.action(blackboard)
            self.executive._check_running()
        except MissionCanceled:
            return 'canceled'
        except (MissionError, ValueError) as error:
            self.executive._last_error = f'{self.name}: {error}'
            self.executive._report(self.executive._last_error, error=True)
            return 'aborted'
        self.executive._report(f'{self.name}: done')
        return 'succeeded'


class TakeoffState(_MissionState):
    """Enter takeoff mode, arm and verify the configured climb height."""

    def __init__(self, executive):
        super().__init__(executive, 'TAKEOFF', executive._takeoff)


class SweepState(_MissionState):
    """Scan six slow yaw increments, dwelling at each heading for smoke detection."""

    def __init__(self, executive):
        super().__init__(executive, 'SWEEP', executive._sweep)


class ExecutePathState(_MissionState):
    """Obtain one plan and fly its three equal arc-length sections."""

    def __init__(self, executive):
        super().__init__(executive, 'EXECUTE_PATH', executive._execute_path)


def build_state_machine(executive, skip_takeoff=False):
    """Build one mission; airborne goals start directly at SWEEP."""
    machine = StateMachine(outcomes=['succeeded', 'aborted', 'canceled'])
    states = (
        ('TAKEOFF', TakeoffState(executive), 'SWEEP'),
        ('SWEEP', SweepState(executive), 'EXECUTE_PATH'),
        ('EXECUTE_PATH', ExecutePathState(executive), 'succeeded'),
    )
    for name, state, next_state in states[1:] if skip_takeoff else states:
        machine.add_state(name, state, transitions={
            'succeeded': next_state, 'aborted': 'aborted', 'canceled': 'canceled',
        })
    machine.set_start_state('SWEEP' if skip_takeoff else 'TAKEOFF')
    return machine


class MissionExecutive(Node):
    """Expose sequential flight through a ROS action with feedback and cancel."""

    def __init__(self):
        super().__init__('mission_executive')
        defaults = {
            'auto_start_mission': False,
            'robot_base_frame': 'kv_drone/base_link',
            'local_odometry_topic': '/mavros/local_position/odom',
            'local_frame': 'px4_odom',
            'takeoff_altitude_parameter': 'MIS_TAKEOFF_ALT',
            'setpoint_rate': 20.0,
            'path_speed': 0.75,
            'path_turn_step_duration': 5.0,
            'hold_duration': 0.5,
            'offboard_warmup': 2.0,
            'sweep_step_duration': 12.0,
            'sweep_hold_duration': 3.0,
            'position_tolerance': 0.3,
            'yaw_tolerance': math.radians(5.0),
            'takeoff_altitude_tolerance': 0.3,
            'service_timeout': 10.0,
            'takeoff_timeout': 120.0,
            'waypoint_timeout': 60.0,
            'state_timeout': 10.0,
            'telemetry_timeout': 2.0,
            'planning_timeout': 10.0,
            'mode_timeout': 10.0,
            'transform_timeout': 0.5,
        }
        self.config = {
            name: self.declare_parameter(name, default).value
            for name, default in defaults.items()
        }
        for name, default in defaults.items():
            value = self.config[name]
            if isinstance(default, float) and (
                not math.isfinite(value) or value <= 0.0
            ):
                raise ValueError(f'{name} must be finite and positive')
            if isinstance(default, str) and not value.strip():
                raise ValueError(f'{name} must not be empty')
        if self.config['setpoint_rate'] <= 2.0:
            raise ValueError('setpoint_rate must exceed the PX4 2 Hz minimum')
        if self.config['offboard_warmup'] < 1.0:
            raise ValueError('offboard_warmup must be at least one second')

        self._callbacks = ReentrantCallbackGroup()
        self._data_lock = threading.Lock()
        self._mission_lock = threading.Lock()
        self._stop = threading.Event()
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._recovering = False
        self._finalizing = False
        self._command_inflight = False
        self._vehicle_state = None
        self._odometry = None
        self._state_received = 0.0
        self._odometry_received = 0.0
        self._target = None
        self._published_count = 0
        self._last_publish = 0.0
        self._stream_started = 0.0
        self._owns_flight = False
        self._recovery_hold = False
        self._last_error = ''
        self._state_machine = None
        self.latest_path = None
        self.sub_paths = []

        # Match MAVROS's reliable, latched state publisher; odometry stays sensor QoS.
        self._state_subscription = self.create_subscription(
            VehicleState, '/mavros/state', self._state_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._callbacks,
        )
        self._statustext_subscription = self.create_subscription(
            StatusText, '/mavros/statustext/recv', self._statustext_callback,
            qos_profile_sensor_data, callback_group=self._callbacks,
        )
        self._odometry_subscription = self.create_subscription(
            Odometry, self.config['local_odometry_topic'], self._odometry_callback,
            qos_profile_sensor_data, callback_group=self._callbacks,
        )
        self._setpoint_publisher = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10,
        )
        self._status_publisher = self.create_publisher(
            String, 'mission_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._setpoint_timer = self.create_timer(
            1.0 / self.config['setpoint_rate'], self._publish_setpoint,
            callback_group=self._callbacks,
        )
        self._mode_client = self.create_client(
            SetMode, '/mavros/set_mode', callback_group=self._callbacks,
        )
        self._arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming', callback_group=self._callbacks,
        )
        self._pull_client = self.create_client(
            ParamPull, '/mavros/param/pull', callback_group=self._callbacks,
        )
        self._parameter_client = self.create_client(
            GetParameters, '/mavros/param/get_parameters',
            callback_group=self._callbacks,
        )
        self._planning_client = self.create_client(
            PlanPath, 'plan_path', callback_group=self._callbacks,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._mission_action = ActionServer(
            self, ExecuteMission, 'execute_mission',
            execute_callback=self._execute_mission_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            handle_accepted_callback=self._goal_accepted_callback,
            callback_group=self._callbacks,
        )
        self._mission_client = ActionClient(
            self, ExecuteMission, 'execute_mission', callback_group=self._callbacks,
        )
        self._cancel_client = self.create_client(
            CancelGoal, 'execute_mission/_action/cancel_goal', callback_group=self._callbacks,
        )
        self._auto_start_timer = None
        if self.config['auto_start_mission']:
            self._auto_start_timer = self.create_timer(
                1.0, self._auto_start, callback_group=self._callbacks,
            )
        self._report('IDLE: send an execute_mission action goal to start')

    def _state_callback(self, message):
        with self._data_lock:
            previous = self._vehicle_state
            self._vehicle_state = message
            self._state_received = time.monotonic()
            if self._recovery_hold and (not message.armed or message.mode != 'OFFBOARD'):
                self._target = None
                self._recovery_hold = False
        if previous is not None and (
            previous.mode != message.mode or previous.armed != message.armed
            or previous.connected != message.connected
        ):
            self._report(
                f'FCU: {previous.mode} -> {message.mode}; '
                f'armed={message.armed}, connected={message.connected}'
            )

    def _statustext_callback(self, message):
        self._report(f'PX4: {message.text}', error=message.severity <= 3)

    def _odometry_callback(self, message):
        with self._data_lock:
            self._odometry = message
            self._odometry_received = time.monotonic()

    def _report(self, message, error=False):
        self._status_publisher.publish(String(data=message))
        # rclpy caches severity by call site; each level needs its own call.
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)
        with self._goal_lock:
            goal = self._goal_handle
            if goal is not None and goal.is_active:
                goal.publish_feedback(ExecuteMission.Feedback(
                    state=message.partition(':')[0], message=message,
                ))

    def _check_running(self):
        if self._stop.is_set() or not rclpy.ok(context=self.context):
            raise MissionError('Mission canceled or ROS is shutting down')
        with self._goal_lock:
            goal = self._goal_handle
            if (
                not self._recovering and not self._command_inflight
                and goal is not None and goal.is_cancel_requested
            ):
                raise MissionCanceled('Mission canceled by client')

    def _wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while True:
            self._check_running()
            if predicate():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise MissionError(f'Timed out: {description}')
            self._stop.wait(min(0.05, remaining))

    def _telemetry_error_locked(self, now):
        """Check receive ages under _data_lock; state follows the 1 Hz heartbeat."""
        if self._vehicle_state is None:
            return 'Missing FCU state on /mavros/state'
        if not self._vehicle_state.connected:
            return 'FCU disconnected: /mavros/state reports connected=false'
        state_age = now - self._state_received
        if state_age >= self.config['state_timeout']:
            return (
                f'Stale FCU state on /mavros/state: last received {state_age:.2f} s ago '
                f'(state_timeout={self.config["state_timeout"]:.2f} s)'
            )
        odometry_topic = self.config['local_odometry_topic']
        if self._odometry is None:
            return f'Missing local odometry on {odometry_topic}'
        odometry_age = now - self._odometry_received
        if odometry_age >= self.config['telemetry_timeout']:
            return (
                f'Stale local odometry on {odometry_topic}: '
                f'last received {odometry_age:.2f} s ago '
                f'(telemetry_timeout={self.config["telemetry_timeout"]:.2f} s)'
            )
        return ''

    def _telemetry_ready(self):
        with self._data_lock:
            return not self._telemetry_error_locked(time.monotonic())

    def _snapshot(self, expected_mode=None, require_armed=False):
        with self._data_lock:
            error = self._telemetry_error_locked(time.monotonic())
            if error:
                raise MissionError(error)
            state = deepcopy(self._vehicle_state)
            odometry = deepcopy(self._odometry)
        if odometry.header.frame_id != self.config['local_frame']:
            raise MissionError(
                f'Local odometry frame is {odometry.header.frame_id!r}; '
                f'expected {self.config["local_frame"]!r}'
            )
        # Validate both position and quaternion before using flight telemetry.
        interpolate_pose(odometry.pose.pose, odometry.pose.pose, 0.0)
        if require_armed and not state.armed:
            raise MissionError('Vehicle disarmed during mission')
        if expected_mode is not None and state.mode != expected_mode:
            raise MissionError(f'Expected {expected_mode}, vehicle is in {state.mode}')
        return state, odometry

    def _call(self, client, request, description, timeout=None):
        timeout = self.config['service_timeout'] if timeout is None else timeout
        deadline = time.monotonic() + timeout
        self._wait_until(client.service_is_ready, timeout, f'{description} service')
        future = client.call_async(request)
        try:
            self._wait_until(
                future.done, max(0.0, deadline - time.monotonic()), description,
            )
            response = future.result()
        except BaseException:
            future.cancel()
            raise
        if response is None:
            raise MissionError(f'{description} returned no response')
        return response

    def _change_mode(self, mode):
        # A sent mode command cannot be recalled. Observe its bounded outcome
        # before honoring cancellation, so recovery knows who owns the vehicle.
        self._command_inflight = True
        try:
            self._change_mode_command(mode)
        finally:
            self._command_inflight = False

    def _change_mode_command(self, mode):
        response = self._call(
            self._mode_client, SetMode.Request(base_mode=0, custom_mode=mode),
            f'Set mode {mode}',
        )
        if not response.mode_sent:
            raise MissionError(f'PX4 rejected mode request {mode}')

        def acknowledged():
            state, _ = self._snapshot()
            return state.mode == mode

        self._wait_until(
            acknowledged, self.config['mode_timeout'], f'FCU mode {mode}',
        )

    def _read_takeoff_altitude(self):
        response = self._call(
            self._pull_client, ParamPull.Request(force_pull=False),
            'Import PX4 parameters',
        )
        if not response.success or response.param_received == 0:
            raise MissionError('Could not import the PX4 parameter catalogue')
        name = self.config['takeoff_altitude_parameter']
        response = self._call(
            self._parameter_client, GetParameters.Request(names=[name]),
            f'Read {name}',
        )
        if len(response.values) != 1:
            raise MissionError(f'PX4 parameter {name} was not returned')
        value = response.values[0]
        if value.type == ParameterType.PARAMETER_DOUBLE:
            altitude = value.double_value
        elif value.type == ParameterType.PARAMETER_INTEGER:
            altitude = float(value.integer_value)
        else:
            raise MissionError(f'PX4 parameter {name} is unset or nonnumeric')
        if not math.isfinite(altitude) or altitude <= 0.0:
            raise MissionError(f'Invalid PX4 takeoff altitude: {altitude}')
        if self.config['takeoff_altitude_tolerance'] >= altitude:
            raise MissionError('Takeoff altitude tolerance must be below target height')
        return altitude

    def _takeoff(self, blackboard):
        self._wait_until(
            self._telemetry_ready, self.config['service_timeout'],
            'connected FCU and local odometry',
        )
        state, _ = self._snapshot()
        if state.armed:
            raise MissionError('TAKEOFF requires a disarmed vehicle at mission start')
        height = self._read_takeoff_altitude()
        state, odometry = self._snapshot()
        if state.armed:
            raise MissionError('Vehicle was armed while preparing TAKEOFF')
        start_z = odometry.pose.pose.position.z
        blackboard['takeoff_altitude'] = height
        target_z = start_z + height
        self._report(f'TAKEOFF: PX4 target climb {height:.2f} m')
        self._owns_flight = True
        self._change_mode('AUTO.TAKEOFF')
        self._check_running()
        self._command_inflight = True
        try:
            response = self._call(
                self._arming_client, CommandBool.Request(value=True), 'Arm vehicle',
            )
            if not response.success:
                raise MissionError(f'PX4 rejected arming (result {response.result})')
            self._wait_until(
                lambda: self._snapshot()[0].armed,
                self.config['mode_timeout'], 'armed confirmation',
            )
        finally:
            self._command_inflight = False

        def altitude_reached():
            state, current = self._snapshot(require_armed=True)
            if state.mode not in {'AUTO.TAKEOFF', 'AUTO.LOITER'}:
                raise MissionError(f'Unexpected mode during TAKEOFF: {state.mode}')
            return (
                current.pose.pose.position.z
                >= target_z - self.config['takeoff_altitude_tolerance']
            )

        self._wait_until(
            altitude_reached, self.config['takeoff_timeout'], 'takeoff altitude',
        )

    def _set_target(self, pose):
        # Normalize/validate a copy; never expose a mutable pose to the timer.
        pose = interpolate_pose(pose, pose, 0.0)
        with self._data_lock:
            self._target = pose

    def _publish_setpoint(self):
        with self._data_lock:
            target = deepcopy(self._target)
        if target is None or self._stop.is_set():
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.config['local_frame']
        message.pose = target
        self._setpoint_publisher.publish(message)
        with self._data_lock:
            now = time.monotonic()
            if self._stream_started == 0.0 or now - self._last_publish >= 0.5:
                self._stream_started = now
            self._published_count += 1
            self._last_publish = now

    def _enter_offboard(self, hold_pose, allowed_modes=('AUTO.TAKEOFF', 'AUTO.LOITER')):
        def check_source_mode():
            state, _ = self._snapshot(require_armed=True)
            if state.mode not in allowed_modes:
                raise MissionError(
                    f'OFFBOARD entry interrupted: vehicle is in {state.mode}'
                )

        check_source_mode()
        self._set_target(hold_pose)
        required_count = math.ceil(
            self.config['setpoint_rate'] * self.config['offboard_warmup'] * 0.75
        )
        with self._data_lock:
            initial_count = self._published_count
            self._stream_started = 0.0

        def primed():
            check_source_mode()
            with self._data_lock:
                return (
                    self._stream_started > 0.0
                    and time.monotonic() - self._stream_started >= self.config['offboard_warmup']
                    and self._published_count - initial_count >= required_count
                    and time.monotonic() - self._last_publish < 0.5
                )

        self._wait_until(
            primed, self.config['offboard_warmup'] + self.config['mode_timeout'],
            'OFFBOARD setpoint warmup',
        )
        check_source_mode()
        self._change_mode('OFFBOARD')

    def _wait_at_pose(self, target, hold_duration=None):
        """Require continuous measured stability for the requested dwell time."""
        if hold_duration is None:
            hold_duration = self.config['hold_duration']
        stable_since = None

        def arrived():
            nonlocal stable_since
            _, odometry = self._snapshot('OFFBOARD', require_armed=True)
            current = odometry.pose.pose
            distance = math.dist(
                (current.position.x, current.position.y, current.position.z),
                (target.position.x, target.position.y, target.position.z),
            )
            yaw_error = abs(angular_distance(
                yaw_from_quaternion(target.orientation),
                yaw_from_quaternion(current.orientation),
            ))
            if (
                distance > self.config['position_tolerance']
                or yaw_error > self.config['yaw_tolerance']
            ):
                stable_since = None
                return False
            if stable_since is None:
                stable_since = time.monotonic()
            return time.monotonic() - stable_since >= hold_duration

        self._wait_until(arrived, self.config['waypoint_timeout'], 'target convergence')

    def _ramp(self, start, target, duration, hold_duration=None):
        """Stream an interpolated move, then wait for a stable measured pose."""
        started = time.monotonic()
        while True:
            self._check_running()
            self._snapshot('OFFBOARD', require_armed=True)
            elapsed = time.monotonic() - started
            fraction = min(1.0, elapsed / max(duration, 0.001))
            self._set_target(interpolate_pose(start, target, fraction))
            if fraction >= 1.0:
                break
            self._stop.wait(min(1.0 / self.config['setpoint_rate'], duration - elapsed))
        self._wait_at_pose(target, hold_duration=hold_duration)

    def _fly_to(self, start, target):
        """Face the local XY travel direction before translating to a waypoint."""
        target = deepcopy(target)
        dx = target.position.x - start.position.x
        dy = target.position.y - start.position.y
        current_yaw = yaw_from_quaternion(start.orientation)
        # A vertical or repeated point has no horizontal direction. Keep the
        # previous heading instead of using the planner's arbitrary orientation.
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else current_yaw
        target.orientation = quaternion_from_yaw(yaw)
        facing_start = deepcopy(start)
        facing_start.orientation = deepcopy(target.orientation)
        yaw_change = abs(angular_distance(yaw, current_yaw))
        if yaw_change > 1e-6:
            self._ramp(
                start, facing_start,
                yaw_change * self.config['path_turn_step_duration'] / (math.tau / 6.0),
            )
        distance = math.dist(
            (start.position.x, start.position.y, start.position.z),
            (target.position.x, target.position.y, target.position.z),
        )
        self._ramp(facing_start, target, distance / self.config['path_speed'])
        return target

    def _loiter(self):
        # Keep streaming until the FCU actually acknowledges AUTO.LOITER.
        self._change_mode('AUTO.LOITER')
        with self._data_lock:
            self._target = None

    def _sweep(self, blackboard):
        _, odometry = self._snapshot(require_armed=True)
        original = deepcopy(odometry.pose.pose)
        original_yaw = yaw_from_quaternion(original.orientation)
        blackboard['sweep_start_orientation'] = deepcopy(original.orientation)
        self._enter_offboard(original)
        previous = original
        # Reaching all six intermediate headings proves progress, not merely
        # the equality between the initial heading and a full-turn heading.
        for step in range(1, 7):
            target = deepcopy(original)
            target.orientation = quaternion_from_yaw(original_yaw + step * math.tau / 6.0)
            # Keep each heading stable long enough for consecutive detector frames.
            self._ramp(
                previous, target, self.config['sweep_step_duration'],
                hold_duration=self.config['sweep_hold_duration'],
            )
            self._report(f'SWEEP: step {step}/6 reached')
            previous = target
        self._wait_at_pose(original)
        self._loiter()

    def _path_frame_transform(self, frame):
        _, odometry = self._snapshot(require_armed=True)
        if frame == self.config['local_frame']:
            # The relative transform of an identical pose is identity.
            return relative_transform(odometry.pose.pose, odometry.pose.pose)
        stamp = odometry.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            raise MissionError('Cannot align map/local frames with unstamped odometry')
        try:
            body_in_source = self._tf_buffer.lookup_transform(
                frame, self.config['robot_base_frame'], Time.from_msg(stamp),
                timeout=Duration(seconds=self.config['transform_timeout']),
            )
        except TransformException as error:
            raise MissionError(f'Cannot align {frame} with PX4 local frame: {error}') from error
        source_pose = Pose()
        source_pose.position.x = body_in_source.transform.translation.x
        source_pose.position.y = body_in_source.transform.translation.y
        source_pose.position.z = body_in_source.transform.translation.z
        source_pose.orientation = body_in_source.transform.rotation
        # The same physical body at the same timestamp anchors the two origins.
        # Merely changing PoseStamped.header.frame_id does not convert MAVROS
        # setpoints, and no identity map -> px4_odom transform is assumed here.
        return relative_transform(odometry.pose.pose, source_pose)

    def _execute_path(self, blackboard):
        self._snapshot('AUTO.LOITER', require_armed=True)
        response = self._call(
            self._planning_client, PlanPath.Request(), 'Plan path',
            timeout=self.config['planning_timeout'],
        )
        self._snapshot('AUTO.LOITER', require_armed=True)
        if not response.success:
            raise MissionError(f'Planner failed: {response.message}')
        path = response.path
        if not path.header.frame_id:
            raise MissionError('Planner returned a path without a frame')
        self.sub_paths = split_path(path, segments=3)
        self.latest_path = deepcopy(path)
        blackboard['sub_paths'] = self.sub_paths
        conversion = self._path_frame_transform(path.header.frame_id)
        _, odometry = self._snapshot('AUTO.LOITER', require_armed=True)
        start = transform_pose(self.sub_paths[0].poses[0].pose, conversion)
        current = odometry.pose.pose
        if math.dist(
            (start.position.x, start.position.y, start.position.z),
            (current.position.x, current.position.y, current.position.z),
        ) > self.config['position_tolerance']:
            raise MissionError('Vehicle moved too far from the planned start; replan required')
        self._enter_offboard(current, allowed_modes=('AUTO.LOITER',))
        # Recheck after priming: even LOITER may drift while the stream warms up.
        _, odometry = self._snapshot('OFFBOARD', require_armed=True)
        current = odometry.pose.pose
        start_distance = math.dist(
            (start.position.x, start.position.y, start.position.z),
            (current.position.x, current.position.y, current.position.z),
        )
        if start_distance > self.config['position_tolerance']:
            raise MissionError('Vehicle moved too far from the planned start; replan required')
        previous = self._fly_to(current, start)
        for index, leg in enumerate(self.sub_paths, start=1):
            for stamped in leg.poses[1:]:
                target = transform_pose(stamped.pose, conversion)
                previous = self._fly_to(previous, target)
            self._report(f'EXECUTE_PATH: sub-path {index}/3 done')
        self._loiter()

    def _goal_callback(self, _request):
        if self._stop.is_set() or not self._mission_lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _goal_accepted_callback(self, goal_handle):
        with self._goal_lock:
            self._goal_handle = goal_handle
            self._finalizing = False
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        with self._goal_lock:
            if self._finalizing or goal_handle is not self._goal_handle:
                return CancelResponse.REJECT
        # rclpy changes the goal to CANCELING after this callback returns.
        # Poll that state in mission waits; keep the setpoint timer independent.
        return CancelResponse.ACCEPT

    def send_mission_goal(self, skip_takeoff=False, feedback_callback=None):
        """Submit internally without blocking the executor; return a goal future."""
        return self._mission_client.send_goal_async(
            ExecuteMission.Goal(skip_takeoff=skip_takeoff),
            feedback_callback=feedback_callback,
        )

    def cancel_mission(self):
        """Request cancellation of the current goal, or return None when idle."""
        with self._goal_lock:
            goal = self._goal_handle
            if goal is None or not goal.is_active or self._finalizing:
                return None
            info = GoalInfo(goal_id=deepcopy(goal.goal_id))
        return self._cancel_client.call_async(CancelGoal.Request(goal_info=info))

    def _auto_start(self):
        if not self._mission_client.server_is_ready():
            return
        self._auto_start_timer.cancel()
        self.send_mission_goal()

    def _recover_to_loiter(self):
        with self._data_lock:
            state = deepcopy(self._vehicle_state)
        if not self._owns_flight or state is None or not state.armed or state.mode not in {
            'OFFBOARD', 'AUTO.TAKEOFF',
        }:
            return ''
        try:
            current_state, odometry = self._snapshot(require_armed=True)
            if current_state.mode not in {'OFFBOARD', 'AUTO.TAKEOFF'}:
                return ''  # Preserve a pilot or failsafe takeover during recovery.
            self._set_target(odometry.pose.pose)
        except (MissionError, ValueError):
            pass  # Keep the last validated target if telemetry is unavailable.
        with self._data_lock:
            state = self._vehicle_state
            if state is None or not state.armed or state.mode not in {
                'OFFBOARD', 'AUTO.TAKEOFF',
            }:
                return ''
        self._report('MISSION: holding position; requesting AUTO.LOITER')
        try:
            self._loiter()
        except Exception as error:  # noqa: BLE001
            message = f'; AUTO.LOITER recovery failed: {error}'
            message += '; retaining the local hold setpoint while OFFBOARD'
            self._report(f'MISSION: {message.lstrip("; ")}', error=True)
            return message
        return '; AUTO.LOITER confirmed'

    def _execute_mission_callback(self, goal_handle):
        """Run one accepted action and always finish its lifecycle and busy slot."""
        outcome = 'aborted'
        message = ''
        self._owns_flight = False
        try:
            self._check_running()
            self.latest_path = None
            self.sub_paths = []
            self._last_error = ''
            if goal_handle.request.skip_takeoff:
                self._snapshot('AUTO.LOITER', require_armed=True)
                self._owns_flight = True
            self._state_machine = build_state_machine(
                self, skip_takeoff=goal_handle.request.skip_takeoff,
            )
            outcome = self._state_machine(Blackboard())
            self._check_running()
            if outcome == 'canceled':
                raise MissionCanceled('Mission canceled by client')
            if outcome != 'succeeded':
                raise MissionError(self._last_error or 'Mission aborted')
            self._report('MISSION: done; vehicle in AUTO.LOITER')
            states = 'SWEEP and EXECUTE_PATH' if goal_handle.request.skip_takeoff else (
                'TAKEOFF, SWEEP and EXECUTE_PATH'
            )
            message = f'{states} completed; AUTO.LOITER confirmed'
        except MissionCanceled as error:
            outcome = 'canceled'
            message = str(error)
        except Exception as error:  # noqa: BLE001
            outcome = 'aborted'
            message = str(error)
        finally:
            try:
                if outcome != 'succeeded' and rclpy.ok(context=self.context):
                    # Cancel interrupts mission work, never this bounded handoff.
                    self._recovering = True
                    self._report(f'MISSION: {outcome}: {message}', error=outcome == 'aborted')
                    if not self._stop.is_set():
                        message += self._recover_to_loiter()
                with self._goal_lock:
                    self._finalizing = True
                    # A late accepted cancel may arrive just after the last state.
                    if goal_handle.is_cancel_requested:
                        outcome = 'canceled'
                        if 'canceled' not in message.lower():
                            message = f'Mission canceled by client; {message}'
                    if outcome == 'succeeded':
                        goal_handle.succeed()
                    elif outcome == 'canceled':
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
            finally:
                with self._data_lock:
                    state = self._vehicle_state
                    if (
                        self._stop.is_set() or state is None
                        or not state.armed or state.mode != 'OFFBOARD'
                    ):
                        self._target = None
                    self._recovery_hold = self._target is not None
                with self._goal_lock:
                    self._goal_handle = None
                    self._recovering = False
                self._state_machine = None
                self._mission_lock.release()
        return ExecuteMission.Result(success=outcome == 'succeeded', message=message)

    def stop_mission(self):
        """Interrupt bounded waits before shutting down the executor."""
        self._stop.set()
