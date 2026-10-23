"""Exercise the native monitor through isolated DDS and simulated telemetry."""

import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix

from geometry_msgs.msg import Point, Pose, Quaternion

from kv_executive.msg import BatteryReturnEstimate, BatteryUsage

from mavros_msgs.msg import HomePosition, State

from nav_msgs.msg import Odometry

import pytest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from rosgraph_msgs.msg import Clock

from sensor_msgs.msg import BatteryState

from std_msgs.msg import Header, String


def test_native_monitor_repeated_walks_and_buffered_alert_over_dds(monkeypatch, tmp_path):
    """Use public records/estimates; no flight-control endpoints are created."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(170 + os.getpid() % 30))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    namespace = f'/native_battery_test_{os.getpid()}'
    executable = (
        Path(get_package_prefix('kv_executive'))
        / 'lib' / 'kv_executive' / 'battery_monitor_node'
    )
    assert executable.is_file(), f'Build kv_executive first: missing {executable}'
    arguments = [str(executable), '--ros-args', '-r', f'__ns:={namespace}']
    remaps = {'/clock': f'{namespace}/clock'}
    for topic in ('battery', 'local_position/odom', 'state', 'home_position/home'):
        remaps[f'/mavros/{topic}'] = f'{namespace}/mavros/{topic}'
    for source, destination in remaps.items():
        arguments.extend(['-r', f'{source}:={destination}'])
    for name, value in {
        'use_sim_time': 'true', 'monitor_rate': '20.0',
        'battery_timeout': '10.0', 'odometry_timeout': '1.0',
        'state_timeout': '10.0',
    }.items():
        arguments.extend(['-p', f'{name}:={value}'])

    context = Context()
    rclpy.init(args=['--ros-args', '-r', f'__ns:={namespace}'], context=context)
    telemetry = Node('native_monitor_telemetry', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(telemetry)
    latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    battery_publisher = telemetry.create_publisher(
        BatteryState, remaps['/mavros/battery'], qos_profile_sensor_data,
    )
    odometry_publisher = telemetry.create_publisher(
        Odometry, remaps['/mavros/local_position/odom'], qos_profile_sensor_data,
    )
    state_publisher = telemetry.create_publisher(State, remaps['/mavros/state'], latched)
    home_publisher = telemetry.create_publisher(
        HomePosition, remaps['/mavros/home_position/home'], latched,
    )
    status_publisher = telemetry.create_publisher(String, 'mission_status', latched)
    clock_publisher = telemetry.create_publisher(Clock, remaps['/clock'], 10)
    usages = []
    estimates = []
    subscriptions = [
        telemetry.create_subscription(BatteryUsage, 'battery_usage', usages.append, latched),
        telemetry.create_subscription(
            BatteryReturnEstimate, 'battery_return_estimate', estimates.append, 10,
        ),
    ]
    lock = threading.Lock()
    stop = threading.Event()
    thread_errors = []
    values = {
        'fraction': 0.5, 'position': (0.0, 0.0, 0.0), 'armed': False,
        'frame': 'px4_odom', 'home': (0.0, 0.0, 0.0), 'publish_odometry': True,
    }

    def stream_telemetry():
        try:
            while not stop.is_set():
                with lock:
                    snapshot = dict(values)
                stamp = Clock()
                elapsed = time.monotonic() - started
                stamp.clock.sec = 1000 + int(elapsed)
                stamp.clock.nanosec = int((elapsed % 1.0) * 1e9)
                clock_publisher.publish(stamp)
                state_publisher.publish(State(connected=True, armed=snapshot['armed']))
                battery_publisher.publish(BatteryState(
                    percentage=snapshot['fraction'], present=True,
                    power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
                ))
                x, y, z = snapshot['position']
                odometry = Odometry(header=Header(frame_id=snapshot['frame']))
                odometry.header.stamp = stamp.clock
                odometry.pose.pose = Pose(
                    position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0),
                )
                if snapshot['publish_odometry']:
                    odometry_publisher.publish(odometry)
                hx, hy, hz = snapshot['home']
                home_publisher.publish(HomePosition(position=Point(x=hx, y=hy, z=hz)))
                stop.wait(0.02)
        except Exception as error:
            thread_errors.append(error)

    log_path = tmp_path / 'native_battery_monitor.log'
    monitor = None
    thread = None

    def logs():
        return log_path.read_text() if log_path.exists() else ''

    def check_alive():
        assert not thread_errors, thread_errors
        assert monitor.poll() is None, logs()

    def wait_for(predicate, description, publish=None, timeout=6.0):
        deadline = time.monotonic() + timeout
        next_publish = -math.inf
        while time.monotonic() < deadline:
            check_alive()
            if predicate():
                return
            if publish is not None and time.monotonic() >= next_publish:
                publish()
                next_publish = time.monotonic() + 0.05
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), f'Timed out waiting for {description}\n{logs()}'

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            check_alive()
            executor.spin_once(timeout_sec=0.02)

    def update(**changes):
        with lock:
            values.update(changes)
        # Supply several complete telemetry cycles before a status transition.
        spin_for(0.18)

    def status(text, predicate):
        wait_for(
            predicate, text,
            lambda: status_publisher.publish(String(data=text)),
        )

    def begin(fraction, count):
        update(fraction=fraction)
        status(
            'EXECUTE_PATH: started',
            lambda: logs().count('EXECUTE_PATH battery baseline:') >= count,
        )
        assert f'EXECUTE_PATH battery baseline: {fraction * 100:.2f}%' in logs()

    def finish(count):
        status('EXECUTE_PATH: done', lambda: len(usages) >= count)
        assert usages[count - 1].sample_id == count
        assert usages[count - 1].outcome == 'completed'

    def estimate_where(predicate, description):
        wait_for(lambda: any(predicate(item) for item in estimates), description)
        return next(item for item in reversed(estimates) if predicate(item))

    try:
        with log_path.open('w') as log_file:
            monitor = subprocess.Popen(
                arguments, stdout=log_file, stderr=subprocess.STDOUT,
            )
        started = time.monotonic()
        thread = threading.Thread(target=stream_telemetry, daemon=True)
        thread.start()
        wait_for(
            lambda: all(publisher.get_subscription_count() >= 1 for publisher in (
                battery_publisher, odometry_publisher, state_publisher,
                home_publisher, status_publisher, clock_publisher,
            )) and all(item.get_publisher_count() >= 1 for item in subscriptions),
            'native process discovery with every input isolated',
        )
        assert all(publisher.topic_name.startswith(namespace + '/') for publisher in (
            battery_publisher, odometry_publisher, state_publisher,
            home_publisher, status_publisher, clock_publisher,
        ))
        spin_for(0.2)
        status_publisher.publish(String(data='TAKEOFF: started'))
        spin_for(0.15)
        update(armed=True, position=(0.0, 0.0, 5.0))
        status('TAKEOFF: done', lambda: 'Battery after takeoff: 50.00%' in logs())
        # HOME_POSITION changes must not move the established ground reference.
        update(home=(999.0, 999.0, 999.0))
        status_publisher.publish(String(data='SWEEP: started'))
        spin_for(0.16)
        assert not usages
        assert not estimates

        begin(0.5, 1)
        update(position=(3.0, 4.0, 5.0))
        unknown = estimate_where(
            lambda item: item.current_pose.pose.position == Point(x=3.0, y=4.0, z=5.0),
            'zero-consumption estimate over the simulated clock',
        )
        assert isinstance(unknown, BatteryReturnEstimate)
        assert 1000 <= unknown.header.stamp.sec < 1100
        assert unknown.home_pose.pose.position == Point()
        assert unknown.remaining_percent == pytest.approx(50.0)
        assert unknown.sample_count == 0
        assert not unknown.estimate_available
        assert math.isnan(unknown.mean_percent_per_meter)
        assert math.isnan(unknown.return_required_percent)
        assert unknown.buffered_required_percent == pytest.approx(15.0)
        assert unknown.return_margin_percent == pytest.approx(35.0)
        assert not unknown.return_alert
        finish(1)
        zero = usages[0]
        assert isinstance(zero, BatteryUsage)
        assert zero.distance_m == pytest.approx(5.0)
        assert zero.start_battery_percent == zero.end_battery_percent == 50.0
        assert zero.consumed_percent == zero.percentage_per_meter == 0.0
        assert not zero.valid_for_estimate

        update(position=(0.0, 0.0, 5.0))
        begin(0.8, 2)
        update(position=(0.0, 0.0, 10.0), fraction=0.75)
        update(position=(3.0, 4.0, 10.0), fraction=0.7)
        learned = estimate_where(
            lambda item: item.estimate_available and item.sample_count == 1
            and item.remaining_percent == pytest.approx(70.0)
            and item.current_pose.pose.position == Point(x=3.0, y=4.0, z=10.0),
            'first positive-consumption rate excluding the zero record',
        )
        assert learned.mean_percent_per_meter == pytest.approx(1.0, abs=1e-6)
        assert learned.home_distance_m == pytest.approx(math.sqrt(125.0))
        assert not learned.return_alert
        finish(2)
        positive = usages[1]
        assert positive.distance_m == pytest.approx(10.0)
        assert positive.start_pose.pose.position == Point(z=5.0)
        assert positive.end_pose.pose.position == Point(x=3.0, y=4.0, z=10.0)
        assert positive.start_pose.header.frame_id == 'px4_odom'
        assert positive.end_pose.header.frame_id == 'px4_odom'
        assert positive.start_battery_percent == pytest.approx(80.0, abs=1e-5)
        assert positive.end_battery_percent == pytest.approx(70.0, abs=1e-5)
        assert positive.consumed_percent == pytest.approx(10.0, abs=1e-5)
        assert positive.percentage_per_meter == pytest.approx(1.0, abs=1e-6)
        assert positive.valid_for_estimate

        begin(0.25, 3)
        update(position=(103.0, 4.0, 10.0), fraction=0.24)
        alert = estimate_where(
            lambda item: item.return_alert and item.sample_count == 2
            and item.remaining_percent == pytest.approx(24.0)
            and item.current_pose.pose.position == Point(x=103.0, y=4.0, z=10.0),
            'weighted return consumption and buffered alert',
        )
        assert alert.estimate_available
        assert alert.home_pose.pose.position == Point()
        assert alert.home_distance_m == pytest.approx(math.sqrt(10725.0))
        assert alert.mean_percent_per_meter == pytest.approx(11.0 / 110.0, abs=1e-6)
        assert alert.return_required_percent == pytest.approx(
            alert.home_distance_m * alert.mean_percent_per_meter,
        )
        assert alert.buffered_required_percent == pytest.approx(
            alert.return_required_percent * 1.3 + 15.0,
        )
        assert alert.return_margin_percent == pytest.approx(
            alert.remaining_percent - alert.buffered_required_percent,
        )
        assert alert.return_margin_percent < 0.0
        wait_for(lambda: 'BATTERY RETURN ALERT:' in logs(), 'ROS INFO return alert')
        finish(3)
        assert usages[2].distance_m == pytest.approx(100.0)
        assert usages[2].consumed_percent == pytest.approx(1.0, abs=1e-5)
        assert usages[2].valid_for_estimate

        begin(0.24, 4)
        update(position=(104.0, 4.0, 10.0), fraction=0.23)
        update(frame='unexpected_frame')
        wait_for(
            lambda: 'Invalid odometry or unexpected local frame' in logs(),
            'invalid interval detection',
        )
        update(frame='px4_odom', position=(105.0, 4.0, 10.0), fraction=0.22)
        recovered = estimate_where(
            lambda item: item.current_pose.pose.position == Point(x=105.0, y=4.0, z=10.0),
            'return estimate retaining only trustworthy completed records',
        )
        assert recovered.sample_count == 2
        assert recovered.mean_percent_per_meter == pytest.approx(11.0 / 110.0, abs=1e-6)
        finish(4)
        invalid = usages[3]
        assert not invalid.valid_for_estimate
        assert invalid.distance_m == pytest.approx(1.0)
        assert invalid.end_pose.pose.position == Point(x=104.0, y=4.0, z=10.0)
        assert invalid.end_battery_percent == pytest.approx(23.0, abs=1e-5)
        assert invalid.consumed_percent == pytest.approx(1.0, abs=1e-5)
        spin_for(0.1)
        assert len(usages) == 4

        begin(0.22, 5)
        update(position=(106.0, 4.0, 10.0), fraction=0.21)
        stale_reason = 'Local odometry is unavailable or stale'
        previous_stale_logs = logs().count(stale_reason)
        update(publish_odometry=False, position=(115.0, 4.0, 10.0))
        # Battery, FCU state and /clock continue while local odometry stops.
        # The gap must invalidate learning before later movement reappears.
        wait_for(
            lambda: logs().count(stale_reason) > previous_stale_logs,
            'steady-clock odometry timeout',
        )
        spin_for(0.1)
        stale_count = len(estimates)
        spin_for(0.15)
        assert len(estimates) == stale_count
        update(position=(116.0, 4.0, 10.0), fraction=0.20)
        update(publish_odometry=True)
        after_gap = estimate_where(
            lambda item: item.current_pose.pose.position == Point(x=116.0, y=4.0, z=10.0),
            'fresh return estimates after the odometry gap',
        )
        assert after_gap.estimate_available
        assert after_gap.sample_count == 2
        assert after_gap.mean_percent_per_meter == pytest.approx(11.0 / 110.0, abs=1e-6)
        finish(5)
        stale = usages[4]
        assert not stale.valid_for_estimate
        assert stale.distance_m == pytest.approx(1.0)
        assert stale.end_pose.pose.position == Point(x=106.0, y=4.0, z=10.0)
        assert stale.end_battery_percent == pytest.approx(21.0, abs=1e-5)
        assert stale.consumed_percent == pytest.approx(1.0, abs=1e-5)

        update(publish_odometry=False)
        spin_for(1.1)
        status(
            'EXECUTE_PATH: started',
            lambda: 'No fresh baseline at EXECUTE_PATH start' in logs(),
        )
        update(publish_odometry=True, position=(117.0, 4.0, 10.0), fraction=0.19)
        no_baseline = estimate_where(
            lambda item: item.current_pose.pose.position == Point(x=117.0, y=4.0, z=10.0),
            'historical estimate after starting without fresh odometry',
        )
        assert no_baseline.sample_count == 2
        assert no_baseline.mean_percent_per_meter == pytest.approx(11.0 / 110.0, abs=1e-6)
        status_publisher.publish(String(data='EXECUTE_PATH: done'))
        spin_for(0.15)
        assert len(usages) == 5, 'An interval without a baseline must not invent a usage record'
        check_alive()
    finally:
        stop.set()
        if thread is not None:
            thread.join(timeout=2.0)
        if monitor is not None and monitor.poll() is None:
            monitor.send_signal(signal.SIGINT)
            try:
                monitor.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                monitor.kill()
                monitor.wait(timeout=3.0)
        executor.shutdown(timeout_sec=2.0)
        telemetry.destroy_node()
        context.shutdown()
