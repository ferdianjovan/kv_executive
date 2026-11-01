#!/usr/bin/env python3
"""
Cancelable ROS action for Exploration and observation-driven Verify.

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
from kv_executive.completed_sites import COMPLETED_SITE_DEFAULTS, CompletedSitesMixin
from kv_executive.inspection_navigation import InspectionNavigationMixin
from kv_executive.mission_geometry import (
    angular_distance,
    interpolate_pose,
    quaternion_from_yaw,
    relative_transform,
    split_path,
    transform_pose,
    yaw_from_quaternion,
)
from kv_executive.mission_navigation import VerifyNavigationMixin
from kv_executive.msg import (
    BatteryReturnEstimate, FireSourceReport, InspectionReport, MissionPhase, MissionStateRequest,
)
from kv_executive.srv import PlanInspectionPath, PlanPath, PlanToPose, QueryInspectionGeometry
from kv_executive.verification import VERIFY_DEFAULTS, VerifyController, VerifyStateMachine
from kv_perception.msg import FireDetection, SmokeDetection
from mavros_msgs.msg import State as VehicleState, StatusText
from mavros_msgs.srv import CommandBool, ParamPull, SetMode
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from yasmin import Blackboard, State, StateMachine


class MissionError(RuntimeError):
    """An operation failed or timed out; stop advancing the mission."""


class MissionTimeout(MissionError):
    """A bounded wait expired; callers can distinguish it from control faults."""


class MissionCanceled(MissionError):
    """A client requested cancellation; stop motion and hand over to LOITER."""


class MissionAbortRequested(MissionError):
    """A monitor requested an aborted action and bounded LOITER recovery."""


class MissionStateRequested(MissionError):
    """Unwind the active child for a requested higher-state transition."""

    def __init__(self, state):
        self.state = state
        self.outcome = {'Return': 'return_requested', 'Verify': 'verify_requested'}[state]
        super().__init__(f'Mission transition to {state} requested')


_MISSION_OUTCOMES = [
    'succeeded', 'aborted', 'canceled', 'return_requested', 'verify_requested',
]


class _MissionState(State):
    def __init__(self, executive, name, action):
        super().__init__(_MISSION_OUTCOMES)
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
        except MissionStateRequested as request:
            self.executive._report(f'Exploration: leaving for {request.state}')
            return request.outcome
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


class ExplorationStateMachine(StateMachine):
    """Take off once, then repeat scans and freshly planned random walks."""

    def __init__(self, executive, skip_takeoff=False):
        super().__init__(outcomes=_MISSION_OUTCOMES)
        self.executive = executive
        states = (
            ('TAKEOFF', TakeoffState(executive), 'SWEEP'),
            ('SWEEP', SweepState(executive), 'EXECUTE_PATH'),
            ('EXECUTE_PATH', ExecutePathState(executive), 'SWEEP'),
        )
        for name, state, next_state in states[1:] if skip_takeoff else states:
            transitions = {outcome: outcome for outcome in _MISSION_OUTCOMES}
            transitions['succeeded'] = next_state
            self.add_state(name, state, transitions=transitions)
        # Native YASMIN nested dispatch does not invoke a Python override of
        # StateMachine.execute; parent entry must be an actual child State.
        self.add_state(
            'ENTER_EXPLORATION',
            _MissionState(executive, 'Exploration', executive._exploration_parent_entry),
            transitions={
                **{outcome: outcome for outcome in _MISSION_OUTCOMES},
                'succeeded': 'SWEEP' if skip_takeoff else 'TAKEOFF',
            },
        )
        self.set_start_state('ENTER_EXPLORATION')


def build_state_machine(executive, skip_takeoff=False):
    """Repeat Exploration, entering Verify on confirmed smoke or fire."""
    machine = StateMachine(outcomes=_MISSION_OUTCOMES)
    machine.add_state(
        'Exploration', ExplorationStateMachine(executive, skip_takeoff),
        transitions={
            **{outcome: outcome for outcome in _MISSION_OUTCOMES},
            'verify_requested': 'Verify',
        },
    )
    machine.add_state(
        'Verify', VerifyStateMachine(executive),
        transitions={
            'explore': 'Exploration', 'aborted': 'aborted',
            'canceled': 'canceled', 'return_requested': 'return_requested',
        },
    )
    machine.set_start_state('Exploration')
    return machine


class MissionExecutive(
        CompletedSitesMixin, InspectionNavigationMixin, VerifyNavigationMixin, Node):
    """Explore and verify detections within one cancelable mission goal."""

    def __init__(self):
        super().__init__('mission_executive')
        defaults = {
            **VERIFY_DEFAULTS,
            **COMPLETED_SITE_DEFAULTS,
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
            'battery_alert_topic': 'battery_return_estimate',
            'battery_alert_timeout': 5.0,
            'smoke_detection_topic': '/kv_drone/smoke/detection',
            'fire_detection_topic': '/kv_drone/fire/detection',
            'verify_planning_frame': 'map',
            'global_position_topic': '/mavros/global_position/global',
            'global_position_timeout': 2.0,
            'verify_path_step': 0.25,
            'fire_exclusion_radius': 15.0,
        }
        self.config = {
            name: self.declare_parameter(name, default).value
            for name, default in defaults.items()
        }
        # Stock MAVROS calls its independent PX4 local ENU coordinates "map".
        # Normalize that known input label, keeping the planner map distinct.
        aliases = self.declare_parameter(
            'local_odometry_frame_aliases', ['map'],
            descriptor=ParameterDescriptor(dynamic_typing=True),
        ).value
        if (not isinstance(aliases, (list, tuple))
                or any(not isinstance(frame, str) or not frame.strip() for frame in aliases)):
            raise ValueError('local_odometry_frame_aliases must contain nonempty frame names')
        self.config['local_odometry_frame_aliases'] = list(aliases)
        for name, default in defaults.items():
            value = self.config[name]
            if isinstance(default, float) and (
                not math.isfinite(value) or value < 0.0
                or (value == 0.0 and name != 'verify_future_tolerance')
            ):
                raise ValueError(
                    f'{name} must be finite and positive (future tolerance may be zero)')
            if isinstance(default, int) and not isinstance(default, bool) and (
                value < 0 or (value == 0 and name != 'verify_fire_approach_max_steps')
            ):
                raise ValueError(f'{name} must be a positive count')
            if isinstance(default, str) and not value.strip():
                raise ValueError(f'{name} must not be empty')
        if self.config['setpoint_rate'] <= 2.0:
            raise ValueError('setpoint_rate must exceed the PX4 2 Hz minimum')
        if self.config['offboard_warmup'] < 1.0:
            raise ValueError('offboard_warmup must be at least one second')
        if self.config['verify_step_min'] > self.config['verify_step_max']:
            raise ValueError('verify_step_min must not exceed verify_step_max')
        for name in ('verify_full_coverage', 'verify_track_center_tolerance'):
            if self.config[name] > 1.0:
                raise ValueError(f'{name} must not exceed one')
        if self.config['verify_track_area_ratio'] < 1.0:
            raise ValueError('verify_track_area_ratio must be at least one')
        if self.config['verify_inspection_max_attempts'] > 3:
            raise ValueError('verify_inspection_max_attempts must not exceed three')
        if not (self.config['verify_inspection_altitude']
                <= self.config['verify_navigation_preferred_agl']
                <= self.config['verify_navigation_max_agl']):
            raise ValueError('Navigation clearance must satisfy minimum <= preferred <= maximum')
        if not (self.config['verify_vertical_position_tolerance']
                < self.config['verify_vertical_minimum_change']
                <= self.config['verify_vertical_step']):
            raise ValueError(
                'Vertical guidance must satisfy position tolerance < minimum change <= step')

        self._callbacks = ReentrantCallbackGroup()
        self._data_lock = threading.Lock()
        self._mission_lock = threading.Lock()
        self._stop = threading.Event()
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._mission_started_ns = 0
        self._requested_mission_state = None
        self._requested_verify_automatic = False
        self._parent_state = None
        self._phase_sequence = 0
        self._airborne_ready = False
        self._verify_retry_after = 0.0
        self._motion_guard = None
        self._recovering = False
        self._finalizing = False
        self._command_inflight = False
        self._vehicle_state = None
        self._odometry = None
        self._odometry_aliases_reported = set()
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
        self._global_position = None
        self._global_position_received = 0.0
        self._fire_sites = []
        self._site_sequence = 0
        self._initialize_completed_sites()
        self._verify = VerifyController(self, error_type=MissionError)

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
        self._battery_alert_subscription = self.create_subscription(
            BatteryReturnEstimate, self.config['battery_alert_topic'],
            self._battery_alert_callback,
            10, callback_group=self._callbacks,
        )
        self._smoke_subscription = self.create_subscription(
            SmokeDetection, self.config['smoke_detection_topic'], self._verify.smoke_callback,
            1, callback_group=self._callbacks,
        )
        self._fire_subscription = self.create_subscription(
            FireDetection, self.config['fire_detection_topic'], self._verify.fire_callback,
            1, callback_group=self._callbacks,
        )
        self._global_position_subscription = self.create_subscription(
            NavSatFix, self.config['global_position_topic'], self._global_position_callback,
            qos_profile_sensor_data, callback_group=self._callbacks,
        )
        self._mission_state_subscription = self.create_subscription(
            String, 'mission_state_request', self._mission_state_request_callback,
            10, callback_group=self._callbacks,
        )
        self._goal_state_subscription = self.create_subscription(
            MissionStateRequest, 'mission_state_request_for_goal',
            self._goal_state_request_callback, 10, callback_group=self._callbacks,
        )
        self._setpoint_publisher = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10,
        )
        self._status_publisher = self.create_publisher(
            String, 'mission_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._phase_publisher = self.create_publisher(
            MissionPhase, 'mission_phase',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._fire_report_publisher = self.create_publisher(
            FireSourceReport, 'fire_sources',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._inspection_report_publisher = self.create_publisher(
            InspectionReport, 'inspection_reports',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
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
        self._target_planning_client = self.create_client(
            PlanToPose, 'plan_to_pose', callback_group=self._callbacks,
        )
        self._inspection_geometry_client = self.create_client(
            QueryInspectionGeometry, 'query_inspection_geometry', callback_group=self._callbacks,
        )
        self._inspection_planning_client = self.create_client(
            PlanInspectionPath, 'plan_inspection_path', callback_group=self._callbacks,
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

    def _global_position_callback(self, message):
        with self._data_lock:
            self._global_position = deepcopy(message)
            self._global_position_received = time.monotonic()

    def _odometry_callback(self, message):
        input_frame = message.header.frame_id
        is_alias = (
            input_frame != self.config['local_frame']
            and input_frame in self.config['local_odometry_frame_aliases']
        )
        if is_alias:
            message = deepcopy(message)
            message.header.frame_id = self.config['local_frame']
        with self._data_lock:
            self._odometry = message
            self._odometry_received = time.monotonic()
            CompletedSitesMixin._completed_odometry_locked(self, message)
            report_alias = is_alias and input_frame not in self._odometry_aliases_reported
            if report_alias:
                self._odometry_aliases_reported.add(input_frame)
        if report_alias:
            self._report(
                f'ODOMETRY: MAVROS local label {input_frame!r} uses '
                f'{self.config["local_frame"]!r}; PX4 coordinates are unchanged',
            )

    def _battery_alert_callback(self, message):
        """Abort only this goal for a fresh, coherent return or reserve alert."""
        if not message.return_alert:
            return
        with self._goal_lock:
            goal = self._goal_handle
            if goal is None or not goal.is_active or self._finalizing:
                return
            goal_id = deepcopy(goal.goal_id)
            accepted_ns = self._mission_started_ns
        now_ns = self.get_clock().now().nanoseconds
        stamp = message.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (accepted_ns <= 0 or source_ns <= accepted_ns or now_ns <= 0
                or not -0.25 <= (now_ns - source_ns) / 1e9
                <= self.config['battery_alert_timeout']):
            return
        frame = self.config['local_frame']
        if (message.header.frame_id != frame
                or message.current_pose.header.frame_id != frame
                or message.home_pose.header.frame_id != frame):
            return
        pose_stamp = message.current_pose.header.stamp
        pose_ns = pose_stamp.sec * 1_000_000_000 + pose_stamp.nanosec
        if (pose_ns <= 0 or not -0.25 <= (now_ns - pose_ns) / 1e9
                <= self.config['telemetry_timeout']):
            return
        try:
            for pose in (message.current_pose.pose, message.home_pose.pose):
                interpolate_pose(pose, pose, 0.0)
        except ValueError:
            return
        remaining = message.remaining_percent
        required = message.buffered_required_percent
        margin = message.return_margin_percent
        if (not all(math.isfinite(value) for value in (
                remaining, required, margin, message.home_distance_m))
                or not 0.0 <= remaining <= 100.0 or required < 0.0
                or message.home_distance_m < 0.0 or remaining > required
                or not math.isclose(margin, remaining - required, abs_tol=1e-3)):
            return
        if message.estimate_available and (
            message.sample_count == 0
            or not all(math.isfinite(value) and value >= 0.0 for value in (
                message.mean_percent_per_meter, message.return_required_percent))
        ):
            return
        # The monitor emits reserve-only alerts before learning a return rate.
        # In that case the rate fields are NaN but the reserve budget is valid.
        if self.request_mission_state('Abort', goal_id=goal_id):
            reason = 'return budget' if message.estimate_available else 'battery reserve'
            self._report(
                f'BATTERY: {reason} alert; {remaining:.2f}% remaining, '
                f'{required:.2f}% required; stopping mission',
            )

    def _verify_parent_entry(self, state):
        """Stop prior motion before publishing admission to a higher state."""
        with self._goal_lock:
            # Set before releasing admission so duplicate requests cannot leak
            # into the next state while the vehicle is being stopped.
            self._parent_state = state
            if state == 'Verify' and self._requested_mission_state == 'Verify':
                self._requested_mission_state = None
                self._requested_verify_automatic = False
        self._check_running()
        if self._airborne_ready:
            self._recovering = True
            try:
                recovery = self._recover_to_loiter()
            finally:
                self._recovering = False
            self._snapshot('AUTO.LOITER', require_armed=True)
            if 'failed' in recovery:
                raise MissionError(recovery)

        def clock_ready():
            stamp = self.get_clock().now().to_msg()
            return stamp.sec > 0 or (stamp.sec == 0 and stamp.nanosec > 0)

        self._wait_until(
            clock_ready, self.config['service_timeout'], 'initialized mission ROS clock',
        )
        self._check_running()
        with self._goal_lock:
            goal = self._goal_handle
            if goal is None or not goal.is_active:
                raise MissionError('Higher-state entry requires an active mission')
            self._phase_sequence += 1
            if self._mission_started_ns <= 0:
                self._mission_started_ns = self.get_clock().now().nanoseconds
            phase = MissionPhase(
                goal_id=deepcopy(goal.goal_id), state=state,
                sequence=self._phase_sequence,
            )
        phase.header.stamp = self.get_clock().now().to_msg()
        phase.header.frame_id = self.config['local_frame']
        self._phase_publisher.publish(phase)
        self._report(f'{state}: entered')

    def _exploration_parent_entry(self, blackboard):
        """Publish the goal-scoped phase used by the detection monitor."""
        try:
            retry_delayed = bool(blackboard['verify_exploration_required'])
        except (KeyError, RuntimeError):
            retry_delayed = False
        blackboard['verify_exploration_required'] = False
        if retry_delayed and self._verify_defer_viewpoint():
            self._report(
                'Exploration: deferring the blocked source until translation or a new bearing')
        cooldown = self.config['verify_retry_cooldown'] if retry_delayed else 0.0
        self._verify_retry_after = time.monotonic() + cooldown if retry_delayed else 0.0
        if retry_delayed:
            self._report(f'Exploration: deferring repeat verification for {cooldown:.1f} s')
        blackboard['verify_entered'] = False
        blackboard['verify_approaches'] = 0
        blackboard['verify_smoke_approach_completed'] = False
        blackboard['verify_smoke_approach_after_ns'] = 0
        blackboard['verify_reference_pose'] = None
        blackboard['verify_smoke_bearing'] = None
        blackboard['verify_arrival_after_ns'] = 0
        blackboard['verify_inspection_geometry'] = None
        blackboard['verify_inspection_origin'] = None
        blackboard['verify_inspection_started'] = None
        blackboard['verify_inspection_attempts'] = 0
        blackboard['verify_inspection_after_ns'] = 0
        blackboard['verify_standoff_reached'] = False
        blackboard['verify_viewpoint_required'] = False
        blackboard['verify_search_active'] = False
        blackboard['verify_search_recovery_required'] = False
        blackboard['verify_search_failures'] = 0
        blackboard['verify_localization_steps'] = 0
        blackboard['verify_localization_origin'] = None
        blackboard['verify_navigation_start_refreshes'] = 0
        blackboard['verify_navigation_refresh_origin'] = None
        blackboard['verify_navigation_refresh_deadline'] = None
        blackboard['verify_navigation_refresh_stamp_ns'] = 0
        blackboard['verify_navigation_stalls'] = 0
        blackboard['verify_navigation_visited'] = None
        blackboard['verify_navigation_legs'] = 0
        blackboard['verify_smoke_boundary_reached'] = False
        blackboard['verify_smoke_walls'] = []
        blackboard['verify_smoke_wall_probe'] = None
        blackboard['verify_wall_latest'] = None
        blackboard['verify_wall_last_stamp_ns'] = 0
        blackboard['verify_wall_near_count'] = 0
        blackboard['verify_wall_recheck_deadline'] = None
        blackboard['verify_height_hint'] = None
        blackboard['verify_height_hint_consumed'] = 0
        blackboard['verify_height_hint_epoch'] = 0
        blackboard['verify_descent_deferred'] = False
        blackboard['verify_height_only_attempted'] = False
        blackboard['verify_vertical_track'] = None
        blackboard['verify_vertical_stamp'] = 0
        blackboard['verify_vertical_direction'] = 0
        blackboard['verify_vertical_attempts'] = 0
        blackboard['verify_fire_pending'] = False
        blackboard['verify_fire_observed'] = False
        blackboard['verify_fire_seen_ns'] = 0
        blackboard['verify_fire_approach_track'] = None
        blackboard['verify_fire_approach_steps'] = 0
        blackboard['verify_fire_standoff_established'] = False
        self._verify.discard_fire_confirmation()
        self._verify_reset_source_localization()
        self._verify_parent_entry('Exploration')

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
        # Admission can race an inconclusive inspection finishing. Recheck an
        # automatic request in the mission worker before dispatch, without
        # holding the goal lock across source-time TF/odometry association.
        with self._goal_lock:
            goal = self._goal_handle
            automatic_goal = (
                deepcopy(goal.goal_id)
                if goal is not None and self._requested_mission_state == 'Verify'
                and getattr(self, '_requested_verify_automatic', False) else None)
        if automatic_goal is not None and (
                CompletedSitesMixin._verify_request_is_completed(self)
                or CompletedSitesMixin._verify_request_is_deferred(self, automatic_goal)):
            with self._goal_lock:
                goal = self._goal_handle
                if (goal is not None
                        and tuple(goal.goal_id.uuid) == tuple(automatic_goal.uuid)
                        and self._requested_mission_state == 'Verify'
                        and getattr(self, '_requested_verify_automatic', False)):
                    self._requested_mission_state = None
                    self._requested_verify_automatic = False
        with self._goal_lock:
            goal = self._goal_handle
            if not self._recovering and not self._command_inflight:
                if goal is not None and goal.is_cancel_requested:
                    raise MissionCanceled('Mission canceled by client')
                if self._requested_mission_state == 'Abort':
                    raise MissionAbortRequested('Mission aborted by a monitor request')
                if self._requested_mission_state is not None:
                    if self._requested_mission_state == 'Verify':
                        if not self._airborne_ready:
                            return  # Complete takeoff before honoring a detection.
                        if (self._parent_state == 'Exploration'
                                and time.monotonic() < self._verify_retry_after):
                            return  # Keep the request latched through the brief retry delay.
                        self._verify_retry_after = 0.0
                    raise MissionStateRequested(self._requested_mission_state)

    def _check_motion_observation(self):
        if not self._recovering and not self._command_inflight and self._motion_guard is not None:
            self._motion_guard()

    def _wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while True:
            self._check_running()
            self._check_motion_observation()
            if predicate():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise MissionTimeout(f'Timed out: {description}')
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
        if self._airborne_ready:
            self._snapshot('AUTO.LOITER', require_armed=True)
            return  # Resuming Exploration within an already airborne action.
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
        self._airborne_ready = True
        self._check_running()

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

    def _wait_at_pose(self, target, hold_duration=None, *, height_tolerance=None,
                      pose_validator=None):
        """Require continuous measured stability for the requested dwell time."""
        if height_tolerance is not None and (
                isinstance(height_tolerance, bool)
                or not math.isfinite(height_tolerance) or height_tolerance <= 0.0):
            raise MissionError('Height convergence tolerance must be finite and positive')
        if hold_duration is None:
            hold_duration = self.config['hold_duration']
        stable_since = None

        def arrived():
            nonlocal stable_since
            _, odometry = self._snapshot('OFFBOARD', require_armed=True)
            current = odometry.pose.pose
            if pose_validator is not None:
                pose_validator(current)
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
                or (height_tolerance is not None
                    and abs(current.position.z - target.position.z) > height_tolerance)
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
            self._check_motion_observation()
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
            self._requested_mission_state = None
            self._requested_verify_automatic = False
            self._finalizing = False
            self._parent_state = None
            self._verify_retry_after = 0.0
            self._mission_started_ns = self.get_clock().now().nanoseconds
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

    def request_mission_state(self, state, goal_id=None):
        """Latch a higher-state transition or stop request for the active goal.

        Verify interrupts Exploration; Abort and unimplemented Return end the action.
        Requests apply only to the active goal and never start a mission.
        A supplied action UUID must match while holding the same admission lock.
        """
        if state not in {'Abort', 'Return', 'Verify'}:
            return False
        completed = (state == 'Verify' and goal_id is not None
                     and CompletedSitesMixin._verify_request_is_completed(self))
        deferred = (state == 'Verify' and goal_id is not None
                    and CompletedSitesMixin._verify_request_is_deferred(self, goal_id))
        with self._goal_lock:
            goal = self._goal_handle
            if (
                goal is None or not goal.is_active or self._finalizing
                or self._recovering or goal.is_cancel_requested or self._stop.is_set()
            ):
                return False
            if goal_id is not None and tuple(goal_id.uuid) != tuple(goal.goal_id.uuid):
                return False
            if state == 'Verify' and self._parent_state == 'Verify':
                return False
            if completed or deferred:
                return False
            priority = {None: 0, 'Verify': 1, 'Return': 2, 'Abort': 3}
            if priority[state] >= priority[self._requested_mission_state]:
                self._requested_mission_state = state
                self._requested_verify_automatic = state == 'Verify' and goal_id is not None
        return True

    def _mission_state_request_callback(self, message):
        if not self.request_mission_state(message.data):
            self._report(f'MISSION: ignored state request {message.data!r}')

    def _goal_state_request_callback(self, message):
        if not self.request_mission_state(message.state, goal_id=message.goal_id):
            self._report(f'MISSION: ignored goal state request {message.state!r}')

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
        self._airborne_ready = False
        self._motion_guard = None
        try:
            self._verify.reset()
            self._check_running()
            self.latest_path = None
            self.sub_paths = []
            self._last_error = ''
            if goal_handle.request.skip_takeoff:
                self._snapshot('AUTO.LOITER', require_armed=True)
                self._owns_flight = True
                self._airborne_ready = True
            self._report('Exploration: started; repeating SWEEP and EXECUTE_PATH')
            self._state_machine = build_state_machine(
                self, skip_takeoff=goal_handle.request.skip_takeoff,
            )
            outcome = self._state_machine(Blackboard())
            if outcome == 'canceled':
                raise MissionCanceled('Mission canceled by client')
            if outcome in {'return_requested', 'verify_requested'}:
                state = 'Return' if outcome == 'return_requested' else 'Verify'
                raise MissionStateRequested(state)
            self._check_running()
            if outcome != 'succeeded':
                raise MissionError(self._last_error or 'Mission aborted')
            message = 'Mission completed; AUTO.LOITER confirmed'
            self._report(f'MISSION: {message}')
        except MissionCanceled as error:
            outcome = 'canceled'
            message = str(error)
        except MissionStateRequested as request:
            # Return may supersede Verify while the nested machine unwinds.
            # Close transition admission before starting the bounded handoff.
            with self._goal_lock:
                state = self._requested_mission_state or request.state
                self._recovering = True
            outcome = 'aborted'
            message = {
                'Abort': 'Mission aborted by a monitor request',
                'Return': 'Return requested; Return is not implemented',
                'Verify': 'Verify transition did not complete; mission stopped',
            }[state]
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
                    self._requested_mission_state = None
                    self._recovering = False
                    self._parent_state = None
                    self._mission_started_ns = 0
                self._motion_guard = None
                self._state_machine = None
                self._mission_lock.release()
        return ExecuteMission.Result(success=outcome == 'succeeded', message=message)

    def stop_mission(self):
        """Interrupt bounded waits before shutting down the executor."""
        self._stop.set()
