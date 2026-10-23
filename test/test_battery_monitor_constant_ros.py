"""Keep ROS processes alive when flown distance consumes no measured SOC."""

import math
import signal
import subprocess
import sys
from pathlib import Path

from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_prefix

from kv_executive.action import ExecuteMission
from kv_executive.msg import BatteryReturnEstimate, BatteryUsage

import pytest

from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from sensor_msgs.msg import BatteryState

# Reuse the isolated simulated FCU and /clock; none of these endpoints can
# reach the vehicle's normal MAVROS graph.
from test_mission_action import cancel_goal, mission, wait_for  # noqa: F401


@pytest.mark.parametrize('mission', [True], indirect=True, ids=['sim_clock'])
def test_constant_battery_survives_repeated_missions_over_dds(
    mission, tmp_path,  # noqa: F811
):
    """Complete three zero-depletion missions and a fourth goal's sweep."""
    executive, fcu = mission.executive, mission.fcu
    action_clients = []
    original_destroy_node = fcu.destroy_node

    def destroy_fcu_with_action_clients():
        # The mission fixture calls destroy_node only after stopping the
        # executor and draining its worker pool. A result Future can be done
        # while that client's status/future cleanup callbacks are still queued.
        try:
            for client in action_clients:
                client.destroy()
        finally:
            action_clients.clear()
            original_destroy_node()

    fcu.destroy_node = destroy_fcu_with_action_clients
    fcu.heartbeat_interval = 0.2
    # This test covers message/goal lifetimes; the mission action suite checks
    # its tighter 0.5 s setpoint watchdog independently.
    fcu.offboard_loss_timeout = 2.0
    # The extra DDS process must not turn scheduler delays into flight-stage
    # failures; production values and the simulated flight remain unchanged.
    for name in (
        'service_timeout', 'mode_timeout',
        'takeoff_timeout', 'waypoint_timeout',
    ):
        executive.config[name] = 8.0
    namespace = fcu.get_namespace()
    executable = (
        Path(get_package_prefix('kv_executive'))
        / 'lib' / 'kv_executive' / 'battery_monitor_node.py'
    )
    arguments = [
        sys.executable, str(executable), '--ros-args',
        '-r', f'__ns:={namespace}',
        '-r', f'/clock:={namespace}/clock',
        '-p', 'use_sim_time:=true',
        '-p', 'battery_topic:=test_battery',
        '-p', 'home_position_topic:=test_home',
        '-p', 'minimum_sample_distance:=0.1',
    ]
    for topic in ('state', 'local_position/odom'):
        arguments.extend([
            '-r', f'/mavros/{topic}:={namespace}/mavros/{topic}',
        ])
    usages = []
    estimates = []
    # These entities belong to the fixture and are destroyed after its
    # executor has stopped, avoiding destruction during an active callback.
    battery_publisher = fcu.create_publisher(
        BatteryState, 'test_battery', qos_profile_sensor_data,
    )
    battery_timer = fcu.create_timer(0.05, lambda: battery_publisher.publish(
        BatteryState(
            percentage=0.5, present=True,
            power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
        ),
    ))
    fcu.create_subscription(
        BatteryUsage, 'battery_usage', usages.append,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    fcu.create_subscription(
        BatteryReturnEstimate, 'battery_return_estimate', estimates.append, 10,
    )
    log_path = tmp_path / 'battery_monitor.log'
    with log_path.open('w') as log_file:
        monitor = subprocess.Popen(
            arguments, stdout=log_file, stderr=subprocess.STDOUT,
        )

    def wait_for_monitor(predicate, timeout, description):
        def observed():
            assert monitor.poll() is None, log_path.read_text()
            return predicate()

        wait_for(observed, timeout, description)

    try:
        assert battery_publisher.topic_name.startswith(namespace + '/')
        wait_for_monitor(
            lambda: battery_publisher.get_subscription_count() == 1
            and fcu.status_publisher.get_subscription_count() >= 2
            and fcu.odometry_publisher.get_subscription_count() >= 2,
            5.0, 'separate battery monitor telemetry discovery',
        )
        ready_message = 'Battery monitor ready (C++ runtime)'
        wait_for_monitor(
            lambda: ready_message in log_path.read_text(),
            3.0, 'native battery monitor startup',
        )

        for sample_id, skip_takeoff in enumerate((False, True, True), start=1):
            estimate_count = len(estimates)
            # Use independent clients as repeated CLI invocations do. Retain
            # them until fixture teardown so executor work can finish safely.
            client = ActionClient(
                fcu, ExecuteMission, 'execute_mission',
                callback_group=fcu.callbacks,
            )
            action_clients.append(client)
            wait_for(client.server_is_ready, 3.0, 'mission action discovery')
            future = client.send_goal_async(
                ExecuteMission.Goal(skip_takeoff=skip_takeoff),
            )
            wait_for(future.done, 3.0, 'goal acknowledgement')
            goal = future.result()
            assert goal.accepted
            result_future = goal.get_result_async()
            wait_for_monitor(
                result_future.done, 60.0,
                'mission result with unchanged battery',
            )
            response = result_future.result()
            assert response.status == GoalStatus.STATUS_SUCCEEDED, (
                response.result.message
            )
            assert response.result.success, response.result.message

            wait_for_monitor(
                lambda: len(usages) == sample_id,
                3.0, 'zero-depletion record publication',
            )
            usage = usages[-1]
            assert usage.sample_id == sample_id
            assert usage.distance_m > 0.1
            assert usage.start_battery_percent == 50.0
            assert usage.end_battery_percent == 50.0
            assert usage.consumed_percent == usage.percentage_per_meter == 0.0
            assert not usage.valid_for_estimate
            assert usage.outcome == 'completed'
            wait_for_monitor(
                lambda: len(estimates) > estimate_count,
                3.0, f'unknown return estimate during mission {sample_id}',
            )

        assert estimates, 'Active WALKs must publish typed return estimates'
        for estimate in estimates:
            assert estimate.remaining_percent == 50.0
            assert not estimate.estimate_available
            assert estimate.sample_count == 0
            assert math.isnan(estimate.mean_percent_per_meter)
            assert math.isnan(estimate.return_required_percent)
            assert estimate.buffered_required_percent == 15.0
            assert estimate.return_margin_percent == 35.0
            assert not estimate.return_alert

        with fcu.lock:
            fourth_steps = sum(
                text == 'SWEEP: step 4/6 reached' for _, text in fcu.statuses
            )
        goal_future = fcu.mission_client.send_goal_async(
            ExecuteMission.Goal(skip_takeoff=True),
        )
        wait_for(goal_future.done, 3.0, 'fourth goal acknowledgement')
        goal = goal_future.result()
        assert goal.accepted
        wait_for_monitor(
            lambda: sum(
                text == 'SWEEP: step 4/6 reached' for _, text in fcu.statuses
            ) > fourth_steps,
            20.0, 'fourth goal reaches sweep 4 after three stored records',
        )
        cancel_goal(goal)
        assert monitor.poll() is None, log_path.read_text()
        assert executive._goal_handle is None
        assert [usage.sample_id for usage in usages] == [1, 2, 3]
        assert not fcu.offboard_losses
        assert 'Battery after takeoff: 50.00%' in log_path.read_text()
        assert 'No measurable battery depletion' in log_path.read_text()
    finally:
        battery_timer.cancel()
        if monitor.poll() is None:
            monitor.send_signal(signal.SIGINT)
            try:
                monitor.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                monitor.kill()
                monitor.wait(timeout=3.0)
