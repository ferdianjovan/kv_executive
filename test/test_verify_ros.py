"""Verify stays dormant while real monitor alerts abort into safe LOITER."""

from copy import deepcopy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from kv_executive.detection_monitor import DetectionMonitor
from kv_executive.msg import BatteryReturnEstimate, FireSourceReport, MissionPhase
from kv_perception.msg import FireDetection, SmokeDetection
import pytest
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from test_mission_action import (
    cancel_goal, mission as mission_fixture, mission_result, prepare_airborne,
    received_state, send_goal, wait_for,
)


mission = mission_fixture


class MonitoringSensors(Node):
    """Publish confirmed detector observations and typed battery estimates."""

    def __init__(self):
        super().__init__('dormant_verify_test_sensors')
        self.lock = threading.Lock()
        self.source = None
        self.closed = False
        self.phases = []
        self.reports = []
        reliable = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.smoke_publisher = self.create_publisher(
            SmokeDetection, 'smoke/detection', reliable,
        )
        self.fire_publisher = self.create_publisher(
            FireDetection, 'fire/detection', reliable,
        )
        self.battery_publisher = self.create_publisher(
            BatteryReturnEstimate, 'battery_return_estimate', reliable,
        )
        self.phase_subscription = self.create_subscription(
            MissionPhase, 'mission_phase', self.receive_phase, retained,
        )
        self.report_subscription = self.create_subscription(
            FireSourceReport, 'fire_sources', self.receive_report, retained,
        )
        self.timer = self.create_timer(0.05, self.publish_observations)

    def receive_phase(self, message):
        with self.lock:
            self.phases.append(deepcopy(message))

    def receive_report(self, message):
        with self.lock:
            self.reports.append(deepcopy(message))

    def publish_observations(self):
        with self.lock:
            if self.closed:
                return
            stamp = self.get_clock().now().to_msg()
            smoke = SmokeDetection()
            smoke.header.stamp = stamp
            smoke.header.frame_id = 'test_rgb_camera'
            smoke.background_ready = True
            smoke.status = SmokeDetection.SEARCHING
            if self.source == 'smoke':
                smoke.detected = True
                smoke.status = SmokeDetection.CONFIRMED
                smoke.confirmation_frames = 10
                smoke.region = SmokeDetection.TOP_LEFT
                smoke.regions = [SmokeDetection.TOP_LEFT]
                smoke.quadrant_coverage = [0.4, 0.0, 0.0, 0.0]
                smoke.coverage = 0.1
            self.smoke_publisher.publish(smoke)
            fire = FireDetection()
            fire.header.stamp = stamp
            fire.header.frame_id = 'test_thermal_camera'
            fire.status = FireDetection.SEARCHING
            if self.source == 'fire':
                fire.detected = True
                fire.status = FireDetection.CONFIRMED
                fire.image_width = 640
                fire.image_height = 480
                fire.confirmation_frames = 50
                fire.persistence_sec = 2.0
                fire.track_id = 42
                fire.source_count = 2
                fire.area_px = 2000
                fire.bbox_x = 295
                fire.bbox_y = 220
                fire.bbox_width = 50
                fire.bbox_height = 40
                fire.center_x = 319.5
                fire.center_y = 239.5
            self.fire_publisher.publish(fire)

    def battery_alert(self, fcu, *, available=True):
        with fcu.lock:
            pose = deepcopy(fcu.pose)
        stamp = self.get_clock().now().to_msg()
        message = BatteryReturnEstimate()
        message.header.stamp = stamp
        message.header.frame_id = 'px4_odom'
        message.current_pose.header = deepcopy(message.header)
        message.current_pose.pose = pose
        message.home_pose.header = deepcopy(message.header)
        message.home_pose.pose = deepcopy(pose)
        message.remaining_percent = 8.0
        message.buffered_required_percent = 10.0
        message.return_margin_percent = -2.0
        message.home_distance_m = 0.0
        message.mean_percent_per_meter = 0.1 if available else math.nan
        message.return_required_percent = 0.0 if available else math.nan
        message.sample_count = 1 if available else 0
        message.estimate_available = available
        message.return_alert = True
        return message

    def snapshot(self):
        with self.lock:
            return deepcopy(self.phases), deepcopy(self.reports)


@pytest.fixture
def monitoring_sensors(mission):
    executive, fcu = mission.executive, mission.fcu
    with fcu.lock:
        fcu.offboard_loss_timeout = 2.0
        fcu.heartbeat_interval = 0.2
    sensors = MonitoringSensors()
    monitor = DetectionMonitor()
    executor = executive.executor
    executor.add_node(sensors)
    executor.add_node(monitor)
    try:
        wait_for(
            lambda: sensors.smoke_publisher.get_subscription_count() >= 1
            and sensors.fire_publisher.get_subscription_count() >= 1
            and sensors.battery_publisher.get_subscription_count() >= 1,
            3.0, 'detector monitor and executive battery discovery',
        )
        yield sensors
    finally:
        with sensors.lock:
            sensors.closed = True
            sensors.timer.cancel()
        executor.remove_node(monitor)
        executor.remove_node(sensors)
        monitor.destroy_node()
        sensors.destroy_node()


def _start_airborne_exploration(mission):
    mission.executive.config['sweep_step_duration'] = 0.5
    prepare_airborne(mission)
    goal = send_goal(mission, skip_takeoff=True)
    assert goal.accepted
    wait_for(
        lambda: received_state(mission, mode='OFFBOARD'),
        5.0, 'Exploration survey OFFBOARD',
    )
    return goal


def _assert_dormant_and_loitering(mission, sensors, result):
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert 'AUTO.LOITER confirmed' in result.result.message
    assert not hasattr(mission.executive, '_verify')
    phases, reports = sensors.snapshot()
    assert phases and {phase.state for phase in phases} == {'Exploration'}
    assert not reports
    with mission.fcu.lock:
        assert mission.fcu.state.mode == 'AUTO.LOITER'
        assert mission.fcu.state.armed
        assert not mission.fcu.target_requests
        assert not mission.fcu.arming_requests
        assert not {'AUTO.TAKEOFF', 'AUTO.RTL', 'AUTO.LAND'} & set(
            mission.fcu.mode_requests)
        assert not mission.fcu.offboard_losses
    assert mission.executive._goal_handle is None
    assert not mission.executive._mission_lock.locked()


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_confirmed_detector_aborts_exploration_without_entering_verify(
    mission, monitoring_sensors, source,
):
    sensors = monitoring_sensors
    goal = _start_airborne_exploration(mission)
    with sensors.lock:
        sensors.source = source

    result = mission_result(goal)

    _assert_dormant_and_loitering(mission, sensors, result)
    with mission.fcu.lock:
        requests_after_abort = list(mission.fcu.mode_requests)
    time.sleep(0.3)
    with mission.fcu.lock:
        assert mission.fcu.mode_requests == requests_after_abort
    assert mission.executive._goal_handle is None


@pytest.mark.parametrize('available', [True, False])
def test_battery_alert_aborts_and_old_alert_cannot_abort_a_new_goal(
    mission, monitoring_sensors, available,
):
    sensors = monitoring_sensors
    goal = _start_airborne_exploration(mission)
    if available:
        wait_for(
            lambda: mission.fcu.mode_requests.count('OFFBOARD') >= 2
            and received_state(mission, mode='OFFBOARD'),
            15.0, 'random-path OFFBOARD before learned battery alert',
        )
    old_alert = sensors.battery_alert(mission.fcu, available=available)
    sensors.battery_publisher.publish(old_alert)

    _assert_dormant_and_loitering(mission, sensors, mission_result(goal))

    next_goal = send_goal(mission, skip_takeoff=True)
    assert next_goal.accepted
    sensors.battery_publisher.publish(old_alert)
    result = next_goal.get_result_async()
    wait_for(
        lambda: received_state(mission, mode='OFFBOARD'),
        5.0, 'explicitly restarted Exploration',
    )
    assert not result.done(), 'A previous goal battery alert must not abort its successor'
    sensors.battery_publisher.publish(sensors.battery_alert(mission.fcu, available=available))
    wait_for(result.done, 5.0, 'fresh battery alert abort')
    _assert_dormant_and_loitering(mission, sensors, result.result())


def test_nonalert_battery_estimate_does_not_interrupt_exploration(
    mission, monitoring_sensors,
):
    sensors = monitoring_sensors
    goal = _start_airborne_exploration(mission)
    message = sensors.battery_alert(mission.fcu)
    message.return_alert = False
    sensors.battery_publisher.publish(message)
    result = goal.get_result_async()
    time.sleep(0.2)
    assert not result.done()
    cancel_goal(goal)
