"""Python reference monitor; production launches use the C++ battery_monitor_node."""

from copy import deepcopy
import math
import time

from geometry_msgs.msg import PoseStamped
from kv_executive.battery_model import BatteryUsageModel
from kv_executive.msg import BatteryReturnEstimate, BatteryUsage
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


DEFAULTS = {
    'battery_topic': '/mavros/battery',
    'local_odometry_topic': '/mavros/local_position/odom',
    'vehicle_state_topic': '/mavros/state',
    'mission_status_topic': 'mission_status',
    'home_position_topic': '/mavros/home_position/home',
    'local_frame': 'px4_odom',
    'battery_timeout': 5.0,
    'odometry_timeout': 2.0,
    'state_timeout': 10.0,
    'minimum_sample_distance': 1.0,
    'reserve_percent': 15.0,
    'return_safety_factor': 1.3,
    'alert_repeat_period': 10.0,
    'monitor_rate': 2.0,
}


class BatteryMonitor(Node):
    """Learn from measured flight only; never issue vehicle commands.

    Callbacks run serially in the node's default callback group. Samples are
    dictionaries retained in ``records`` until this process exits. All battery
    values stored or published use percentage points on the 0..100 scale.
    """

    def __init__(self, **kwargs):
        super().__init__('battery_monitor', **kwargs)
        self.config = {
            name: self.declare_parameter(name, default).value
            for name, default in DEFAULTS.items()
        }
        for name, default in DEFAULTS.items():
            value = self.config[name]
            if isinstance(default, str) and not value.strip():
                raise ValueError(f'{name} must not be empty')
            if isinstance(default, float) and (
                not math.isfinite(value) or value < 0.0
                or (value == 0.0 and name != 'reserve_percent')
            ):
                raise ValueError(f'{name} must be finite and positive')
        self._initialize_tracking()
        self._usage_publisher = self.create_publisher(
            BatteryUsage, 'battery_usage',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._estimate_publisher = self.create_publisher(
            BatteryReturnEstimate, 'battery_return_estimate', 10,
        )
        # Node._subscriptions also owns the TimeSource /clock subscription.
        # Keep our references separate so the executor can service every entity.
        self._monitor_subscriptions = [
            self.create_subscription(
                BatteryState, self.config['battery_topic'], self._battery_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry, self.config['local_odometry_topic'], self._odometry_callback,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                State, self.config['vehicle_state_topic'], self._state_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            self.create_subscription(
                HomePosition, self.config['home_position_topic'], self._home_callback,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
            self.create_subscription(
                String, self.config['mission_status_topic'], self._status_callback,
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]
        self._timer = self.create_timer(1.0 / self.config['monitor_rate'], self._tick)
        self.get_logger().info('Battery monitor ready; waiting for EXECUTE_PATH')

    def _initialize_tracking(self):
        self.model = BatteryUsageModel(
            minimum_distance=self.config['minimum_sample_distance'],
            reserve_percent=self.config['reserve_percent'],
            return_safety_factor=self.config['return_safety_factor'],
        )
        self.records = self.model.records
        self._battery_percent = None
        self._battery_received = None
        self._pose = None
        self._odometry_received = None
        self._vehicle_state = None
        self._state_received = None
        self._home_pose = None
        self._ground_pose = None
        self._home_locked = False
        self._mavros_home = None
        self._baseline = None
        self._phase_active = False
        self._interval_invalid = False
        self._last_good_pose = None
        self._last_good_percent = None
        self._last_alert = -math.inf
        self._alert_active = False
        self._unavailable = ''

    def _info_once(self, reason):
        if reason != self._unavailable:
            self.get_logger().info(f'Battery monitor: {reason}')
        self._unavailable = reason

    def _invalidate(self, reason):
        self.model.invalidate_active(reason)
        if self.model.active:
            self._interval_invalid = True
        if self._phase_active:
            self._info_once(reason)

    def _check_gap(self, received, timeout, name):
        if received is not None and time.monotonic() - received >= timeout:
            self._invalidate(f'{name} telemetry gap; this interval cannot teach the model')

    def _battery_callback(self, message):
        self._check_gap(self._battery_received, self.config['battery_timeout'], 'Battery')
        self._battery_received = time.monotonic()
        value = message.percentage
        if (
            not message.present or not math.isfinite(value) or not 0.0 <= value <= 1.0
            or message.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        ):
            self._battery_percent = None
            self._invalidate('Battery percentage unavailable, invalid, or charging')
            return
        self._battery_percent = float(value) * 100.0
        self._observe()

    def _valid_pose(self, pose):
        values = [getattr(pose.pose.position, axis) for axis in 'xyz']
        orientation = [getattr(pose.pose.orientation, axis) for axis in 'xyzw']
        return (
            pose.header.frame_id == self.config['local_frame']
            and all(math.isfinite(value) for value in values + orientation)
            and any(orientation)
        )

    def _odometry_callback(self, message):
        self._check_gap(self._odometry_received, self.config['odometry_timeout'], 'Odometry')
        self._odometry_received = time.monotonic()
        pose = PoseStamped(header=deepcopy(message.header), pose=deepcopy(message.pose.pose))
        if not self._valid_pose(pose):
            self._pose = None
            self._invalidate('Invalid odometry or unexpected local frame')
            return
        self._pose = pose
        state = self._vehicle_state
        if (
            state is not None and not state.armed and state.connected
            and not self._home_locked
            and time.monotonic() - self._state_received < self.config['state_timeout']
        ):
            self._ground_pose = deepcopy(pose)
            self._home_pose = deepcopy(pose)
        self._observe()

    def _state_callback(self, message):
        self._check_gap(self._state_received, self.config['state_timeout'], 'Vehicle state')
        previous = self._vehicle_state
        self._vehicle_state = deepcopy(message)
        self._state_received = time.monotonic()
        if not message.connected:
            self._invalidate('FCU disconnected')
        if not message.armed:
            if self._phase_active:
                self._finish_phase('disarmed')
            self._baseline = None
            if previous is not None and previous.armed:
                self._home_locked = False
            if (
                not self._home_locked and message.connected
                and self._pose is not None and self._odometry_fresh()
            ):
                self._ground_pose = deepcopy(self._pose)
                self._home_pose = deepcopy(self._pose)
        elif previous is None or not previous.armed:
            if self._ground_pose is not None:
                self._home_pose = deepcopy(self._ground_pose)
            elif self._home_pose is None:
                self._home_pose = deepcopy(self._mavros_home)
        self._observe()

    def _home_callback(self, message):
        # MAVROS converts HOME_POSITION.position from PX4 local NED to ENU.
        # Its header may name a global frame; the numeric position shares the
        # local estimator origin, unlike map/planner coordinates.
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.header.frame_id = self.config['local_frame']
        pose.pose.position = deepcopy(message.position)
        pose.pose.orientation.w = 1.0
        if not self._valid_pose(pose):
            return
        self._mavros_home = pose
        if self._ground_pose is None and self._home_pose is None:
            self._home_pose = deepcopy(pose)

    def _odometry_fresh(self):
        return (
            self._odometry_received is not None
            and time.monotonic() - self._odometry_received < self.config['odometry_timeout']
        )

    def _telemetry_problem(self, require_armed=True):
        now = time.monotonic()
        if self._battery_percent is None or self._battery_received is None:
            return 'Waiting for a valid MAVROS battery percentage'
        if now - self._battery_received >= self.config['battery_timeout']:
            return 'Battery telemetry is stale'
        if self._pose is None or not self._odometry_fresh():
            return 'Local odometry is unavailable or stale'
        if (
            self._vehicle_state is None or self._state_received is None
            or now - self._state_received >= self.config['state_timeout']
            or not self._vehicle_state.connected
        ):
            return 'Vehicle state is unavailable, stale, or disconnected'
        if require_armed and not self._vehicle_state.armed:
            return 'Vehicle is disarmed'
        return ''

    def _establish_baseline(self):
        if self._telemetry_problem():
            return False
        self._baseline = {
            'pose': deepcopy(self._pose),
            'battery_percent': self._battery_percent,
        }
        return True

    def _status_callback(self, message):
        text = message.data
        if text == 'TAKEOFF: started':
            self._baseline = None
            # Freeze the ground reference before asynchronous armed telemetry
            # catches up with the first climbing odometry samples.
            if self._ground_pose is not None:
                self._home_pose = deepcopy(self._ground_pose)
                self._home_locked = True
        elif text == 'TAKEOFF: done':
            if self._establish_baseline():
                self.get_logger().info(
                    f'Battery after takeoff: {self._battery_percent:.2f}%'
                )
        elif text == 'EXECUTE_PATH: started':
            if self._phase_active:
                return  # Do not reset an interval on duplicate delivery.
            self._phase_active = True
            self._interval_invalid = False
            self._alert_active = False
            self._last_alert = -math.inf
            if self._establish_baseline():
                self.model.begin(self._pose, self._battery_percent)
                self._last_good_pose = deepcopy(self._pose)
                self._last_good_percent = self._battery_percent
                self.get_logger().info(
                    f'EXECUTE_PATH battery baseline: {self._battery_percent:.2f}%'
                )
            else:
                self._info_once(
                    'No fresh baseline at EXECUTE_PATH start; skipping learning for this interval'
                )
        elif text == 'EXECUTE_PATH: done':
            self._finish_phase('completed')
        elif text.startswith(('MISSION: canceled', 'MISSION: aborted')):
            self._finish_phase('canceled' if 'canceled' in text.lower() else 'aborted')
            state = self._vehicle_state
            if state is not None and state.connected and not state.armed:
                self._home_locked = False
                if self._pose is not None and self._odometry_fresh():
                    self._ground_pose = deepcopy(self._pose)
                    self._home_pose = deepcopy(self._pose)
        elif self._phase_active and (
            text.startswith('EXECUTE_PATH:') and not text.startswith('EXECUTE_PATH: sub-path')
        ):
            self._finish_phase('aborted')

    def _observe(self):
        if not self._phase_active or not self.model.active or self._interval_invalid:
            return
        reason = self._telemetry_problem()
        if reason:
            self._invalidate(reason)
            return
        try:
            self.model.update(self._pose, self._battery_percent)
        except ValueError as error:
            self._invalidate(str(error))
            return
        self._last_good_pose = deepcopy(self._pose)
        self._last_good_percent = self._battery_percent

    def _finish_phase(self, outcome):
        if not self._phase_active:
            return
        self._observe()
        self._phase_active = False
        self._alert_active = False
        if not self.model.active:
            return
        # If telemetry failed, retain the last trustworthy endpoint and mark
        # the record unusable instead of inventing consumption across the gap.
        sample = self.model.finish(
            self._last_good_pose, self._last_good_percent, outcome=outcome,
        )
        message = BatteryUsage()
        message.header = deepcopy(sample['end_pose'].header)
        for name in (
            'sample_id', 'start_pose', 'end_pose', 'distance_m',
            'start_battery_percent', 'end_battery_percent', 'consumed_percent',
            'percentage_per_meter', 'valid_for_estimate', 'outcome',
        ):
            setattr(message, name, deepcopy(sample[name]))
        self._usage_publisher.publish(message)
        exclusion = (
            f"; excluded: {sample['invalid_reason']}" if not sample['valid_for_estimate'] else ''
        )
        self.get_logger().info(
            f"EXECUTE_PATH battery sample {sample['sample_id']}: "
            f"{sample['distance_m']:.2f} m, {sample['consumed_percent']:.2f} percentage points "
            f"consumed; usable={sample['valid_for_estimate']} ({outcome})"
            + exclusion
        )

    def _tick(self):
        if not self._phase_active:
            return
        self._observe()
        reason = self._telemetry_problem()
        if reason:
            self._invalidate(reason)
            return
        if self._home_pose is None:
            self._info_once('Home unavailable; waiting for ground odometry or MAVROS home')
            return
        try:
            estimate = self.model.estimate(self._pose, self._home_pose, self._battery_percent)
        except ValueError as error:
            self._info_once(f'Return estimate unavailable: {error}')
            return
        message = BatteryReturnEstimate()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.config['local_frame']
        message.current_pose = deepcopy(self._pose)
        message.home_pose = deepcopy(self._home_pose)
        message.remaining_percent = self._battery_percent
        for name, value in estimate.items():
            setattr(message, name, value)
        self._estimate_publisher.publish(message)
        if not estimate['estimate_available']:
            self._info_once('Learning battery use per metre; return estimate not available yet')
        else:
            self._unavailable = ''
        now = time.monotonic()
        if estimate['return_alert'] and (
            not self._alert_active
            or now - self._last_alert >= self.config['alert_repeat_period']
        ):
            if estimate['estimate_available']:
                self.get_logger().info(
                    f'BATTERY RETURN ALERT: {self._battery_percent:.2f}% remaining; '
                    f"straight-line home distance {estimate['home_distance_m']:.2f} m; "
                    f"return needs {estimate['return_required_percent']:.2f} points, "
                    f"{estimate['buffered_required_percent']:.2f} with safety buffer; "
                    f"margin {estimate['return_margin_percent']:.2f} points"
                )
            else:
                self.get_logger().info(
                    f'BATTERY RESERVE ALERT: {self._battery_percent:.2f}% remaining; '
                    f"reserve {self.config['reserve_percent']:.2f}%; return rate still unknown"
                )
            self._last_alert = now
        self._alert_active = estimate['return_alert']


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
