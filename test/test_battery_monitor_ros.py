"""Observe battery monitoring over DDS using isolated simulated telemetry."""

import math
import os
import time

from geometry_msgs.msg import Point, Pose, Quaternion
from kv_executive.battery_monitor import BatteryMonitor
from kv_executive.msg import BatteryReturnEstimate, BatteryUsage
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, String


def test_walk_usage_and_buffered_return_alert_over_dds(monkeypatch, tmp_path):
    # No publisher or subscriber can reach the normal MAVROS graph.
    monkeypatch.setenv('ROS_DOMAIN_ID', str(120 + os.getpid() % 50))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    namespace = f'/battery_monitor_test_{os.getpid()}'
    arguments = ['--ros-args', '-r', f'__ns:={namespace}']
    for topic in ('battery', 'local_position/odom', 'state', 'home_position/home'):
        name = f'/mavros/{topic}'
        arguments.extend(['-r', f'{name}:={namespace}{name}'])
    for name, value in {
        'monitor_rate': 20.0,
        'battery_timeout': 60.0,
        'odometry_timeout': 60.0,
        'state_timeout': 60.0,
    }.items():
        arguments.extend(['-p', f'{name}:={value}'])

    rclpy.init(args=arguments)
    monitor = None
    telemetry = None
    executor = None
    try:
        monitor = BatteryMonitor()
        telemetry = Node('battery_monitor_test_telemetry')
        executor = SingleThreadedExecutor()
        executor.add_node(monitor)
        executor.add_node(telemetry)
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        battery_publisher = telemetry.create_publisher(
            BatteryState, '/mavros/battery', qos_profile_sensor_data,
        )
        odometry_publisher = telemetry.create_publisher(
            Odometry, '/mavros/local_position/odom', qos_profile_sensor_data,
        )
        state_publisher = telemetry.create_publisher(State, '/mavros/state', latched)
        home_publisher = telemetry.create_publisher(
            HomePosition, '/mavros/home_position/home', latched,
        )
        status_publisher = telemetry.create_publisher(String, 'mission_status', latched)
        usages = []
        estimates = []
        subscriptions = [
            telemetry.create_subscription(BatteryUsage, 'battery_usage', usages.append, latched),
            telemetry.create_subscription(
                BatteryReturnEstimate, 'battery_return_estimate', estimates.append, 10,
            ),
        ]

        def wait_for(predicate, description, publish=None, timeout=5.0):
            deadline = time.monotonic() + timeout
            next_publish = -math.inf
            while time.monotonic() < deadline:
                if predicate():
                    return
                if publish is not None and time.monotonic() >= next_publish:
                    publish()
                    next_publish = time.monotonic() + 0.05
                executor.spin_once(timeout_sec=0.02)
            assert predicate(), f'Timed out waiting for {description}'

        def spin_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(
                    timeout_sec=max(0.0, min(0.02, deadline - time.monotonic())),
                )

        def battery(fraction):
            message = BatteryState(
                percentage=float(fraction), present=True,
                power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
            )
            wait_for(
                lambda: monitor._battery_percent == pytest.approx(fraction * 100.0),
                f'battery fraction {fraction}',
                lambda: battery_publisher.publish(message),
            )

        def odometry(x=0.0, y=0.0, z=0.0):
            point = Point(x=float(x), y=float(y), z=float(z))
            message = Odometry(header=Header(frame_id='px4_odom'))
            message.header.stamp = telemetry.get_clock().now().to_msg()
            message.pose.pose = Pose(position=point, orientation=Quaternion(w=1.0))
            wait_for(
                lambda: monitor._pose is not None and monitor._pose.pose.position == point,
                f'odometry {point}', lambda: odometry_publisher.publish(message),
            )

        def status(text, predicate):
            wait_for(predicate, text, lambda: status_publisher.publish(String(data=text)))

        wait_for(
            lambda: all(publisher.get_subscription_count() >= 1 for publisher in (
                battery_publisher, odometry_publisher, state_publisher,
                home_publisher, status_publisher,
            )) and all(subscription.get_publisher_count() >= 1 for subscription in subscriptions),
            'simulated telemetry and output discovery',
        )
        assert battery_publisher.topic_name == f'{namespace}/mavros/battery'
        assert odometry_publisher.topic_name == f'{namespace}/mavros/local_position/odom'

        wait_for(
            lambda: monitor._vehicle_state is not None and not monitor._vehicle_state.armed,
            'connected disarmed state',
            lambda: state_publisher.publish(State(connected=True, armed=False)),
        )
        battery(0.9)
        odometry()
        assert monitor._home_pose.pose.position == Point()
        wait_for(
            lambda: monitor._vehicle_state.armed,
            'armed state', lambda: state_publisher.publish(State(connected=True, armed=True)),
        )
        odometry(z=5.0)
        status('TAKEOFF: done', lambda: monitor._baseline is not None)
        assert monitor._baseline['battery_percent'] == pytest.approx(90.0)

        status_publisher.publish(String(data='SWEEP: started'))
        battery(0.8)
        # Allow several actual timer callbacks to prove SWEEP stays inactive.
        spin_for(0.16)
        assert not monitor.model.active
        assert monitor.records == {}
        assert usages == []
        assert estimates == []
        status_publisher.publish(String(data='SWEEP: done'))
        status('EXECUTE_PATH: started', lambda: monitor.model.active)

        # A vertical five metres followed by a horizontal five metres.
        odometry(z=10.0)
        battery(0.75)
        odometry(3.0, 4.0, 10.0)
        battery(0.7)
        wait_for(
            lambda: any(message.estimate_available and message.sample_count == 1
                        and message.remaining_percent == pytest.approx(70.0)
                        for message in estimates),
            'live return estimate from the first active WALK',
        )
        first_estimate = next(message for message in reversed(estimates)
                              if message.remaining_percent == pytest.approx(70.0))
        assert isinstance(first_estimate, BatteryReturnEstimate)
        assert first_estimate.home_distance_m == pytest.approx(math.sqrt(125.0))
        assert first_estimate.mean_percent_per_meter == pytest.approx(1.0)
        assert not first_estimate.return_alert

        status('EXECUTE_PATH: done', lambda: len(monitor.records) == 1)
        wait_for(lambda: len(usages) == 1, 'typed usage publication')
        sample = monitor.records[1]
        usage = usages[0]
        assert isinstance(sample, dict)
        assert isinstance(usage, BatteryUsage)
        assert usage.sample_id == 1
        assert usage.distance_m == pytest.approx(10.0)
        assert usage.start_battery_percent == pytest.approx(80.0)
        assert usage.end_battery_percent == pytest.approx(70.0)
        assert usage.consumed_percent == pytest.approx(10.0)
        assert usage.valid_for_estimate
        assert usage.outcome == 'completed'
        assert usage.start_pose == sample['start_pose']
        assert usage.end_pose == sample['end_pose']
        assert usage.start_pose.pose.position == Point(z=5.0)
        assert usage.end_pose.pose.position == Point(x=3.0, y=4.0, z=10.0)
        assert sample['distance_m'] == pytest.approx(10.0)
        assert sample['consumed_percent'] == pytest.approx(10.0)

        battery(0.25)
        status('EXECUTE_PATH: started', lambda: monitor.model.active)
        odometry(103.0, 4.0, 10.0)
        battery(0.24)
        wait_for(
            lambda: any(message.return_alert and message.sample_count == 2
                        and message.remaining_percent == pytest.approx(24.0)
                        for message in estimates),
            'buffered return alert using completed and current WALKs',
        )
        alert = next(message for message in reversed(estimates)
                     if message.remaining_percent == pytest.approx(24.0))
        assert alert.estimate_available
        assert alert.return_alert
        assert monitor._alert_active
        assert alert.home_pose.pose.position == Point()
        assert alert.home_distance_m == pytest.approx(math.sqrt(10725.0))
        assert alert.mean_percent_per_meter == pytest.approx(11.0 / 110.0)
        assert alert.buffered_required_percent == pytest.approx(
            alert.return_required_percent * 1.3 + 15.0,
        )
        assert alert.return_margin_percent < 0.0

        status('MISSION: canceled', lambda: len(monitor.records) == 2)
        wait_for(lambda: len(usages) == 2, 'partial canceled WALK publication')
        assert usages[1].outcome == 'canceled'
        assert usages[1].distance_m == pytest.approx(100.0)
        assert usages[1].consumed_percent == pytest.approx(1.0)
        assert monitor.records[2]['outcome'] == 'canceled'
        assert not monitor.model.active
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if monitor is not None:
            monitor.destroy_node()
        if telemetry is not None:
            telemetry.destroy_node()
        rclpy.shutdown()
