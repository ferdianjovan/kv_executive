"""Repeat smoke approach and bounded fire centering in an isolated process.

KV_VERIFY_PROCESS_PYTHON selects a compatible Python executable or launcher.
Only a private simulated FCU is used; every MAVROS endpoint is remapped.
"""

from copy import deepcopy
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Point
from kv_executive.mission_geometry import (
    angular_distance, quaternion_from_yaw, yaw_from_quaternion,
)
from kv_executive.verification import VERIFY_DEFAULTS
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from test_mission_action import cancel_goal, send_goal, SimulatedFcu, wait_for
from test_verify_ros import (
    _assert_directional_cluster, _install_camera_transforms, _point_position,
    _with_wall_hit, MonitoringSensors,
)


def test_smoke_terrain_leg_then_fire_centering_in_executive_process(monkeypatch, tmp_path):
    """Repeated goals retain native runtime stability through both motion types."""
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get(
        'ROS_DOMAIN_ID', str(130 + os.getpid() % 50)))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    profile = str(Path(get_package_share_directory('kv_executive'))
                  / 'config' / 'fastdds_udp_localhost.xml')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', profile)
    monkeypatch.setenv('FASTDDS_DEFAULT_PROFILES_FILE', profile)
    monkeypatch.setenv('PYTHONFAULTHANDLER', '1')
    namespace = f'/verify_process_{os.getpid()}_{time.monotonic_ns()}'
    arguments = ['--ros-args', '-r', f'__ns:={namespace}']
    for endpoint in (
        'state', 'statustext/recv', 'local_position/odom', 'global_position/global',
        'setpoint_position/local', 'set_mode', 'cmd/arming', 'param/pull',
        'param/get_parameters',
    ):
        arguments.extend(['-r', f'/mavros/{endpoint}:={namespace}/mavros/{endpoint}'])
    settings = {
        'smoke_detection_topic': 'smoke/detection',
        'fire_detection_topic': 'fire/detection',
        'verify_planning_frame': 'px4_odom',
        'sweep_step_duration': 0.2,
        'sweep_hold_duration': 0.05,
        'path_turn_step_duration': 0.2,
        'verify_step_min': 0.4,
        'verify_step_max': 0.8,
        'verify_navigation_step': 0.8,
        'verify_motion_speed': 5.0,
        'verify_center_timeout': 20.0,
        # Isolate native runtime and centering at an explicitly measured smoke wall.
        'verify_fire_approach_max_steps': 3,
        'verify_inspection_yaw_step': 0.25,
        'verify_reacquire_timeout': 1.0,
        'verify_viewpoint_max_steps': 2,
        'verify_viewpoint_timeout': 90.0,
        'verify_smoke_center_timeout': 0.3,
        'service_timeout': 5.0,
        'planning_timeout': 5.0,
        'mode_timeout': 5.0,
        'telemetry_timeout': 3.0,
        # Keep real mode handoffs while bounding three inspection/handoff cycles.
        'offboard_warmup': 1.0,
        'hold_duration': 0.05,
        'setpoint_rate': 30.0,
    }
    for name, value in settings.items():
        arguments.extend(['-p', f'{name}:={value}'])
    runtime = Path(get_package_prefix('kv_executive')) / 'lib/kv_executive/python_shared_runtime'
    # Exercise the deployed runtime by default. Explicit overrides remain for
    # comparing interpreter controls when investigating a native crash.
    default_python = shlex.join([str(runtime), '--python', sys.executable, '--'])
    executable = shlex.split(os.environ.get('KV_VERIFY_PROCESS_PYTHON', default_python))
    monitor_executable = shlex.split(os.environ.get(
        'KV_VERIFY_MONITOR_PYTHON', os.environ.get('KV_VERIFY_PROCESS_PYTHON', default_python)))
    entrypoint = str(Path(get_package_prefix('kv_executive'))
                     / 'lib' / 'kv_executive' / 'executive_node.py')
    log_path = tmp_path / 'executive.log'
    monitor_log_path = tmp_path / 'monitor.log'
    rclpy.init(args=arguments)
    process = None
    monitor_process = None
    fcu = None
    sensors = None
    executor = None
    thread = None
    active = None
    log = log_path.open('w')
    monitor_log = monitor_log_path.open('w')
    try:
        fcu = SimulatedFcu()
        with fcu.lock:
            fcu.state.armed = True
            fcu.state.mode = 'AUTO.LOITER'
            fcu.offboard_loss_timeout = 2.0
            fcu.heartbeat_interval = 0.2
        sensors = MonitoringSensors()
        # Model processing after exposure so image-time odometry brackets are
        # available when the target binding first consumes a detector frame.
        sensors.detection_latency = 0.08
        _install_camera_transforms(sensors)
        original_geometry = fcu.query_inspection_geometry
        measured_wall_origins = {}

        def near_smoke_wall(request, response):
            response = original_geometry(request, response)
            if request.smoke_wall_width:
                with sensors.lock:
                    near_fire = sensors.source in {'fire', 'smoke_fire'}
                response.smoke_wall_valid = (
                    request.origin.header.frame_id == 'test_thermal_camera' or near_fire)
                response.smoke_wall_range = response.smoke_wall_nearest_range = (
                    VERIFY_DEFAULTS['verify_smoke_standoff'] + 0.4)
            response = _with_wall_hit(response)
            if request.smoke_wall_width and response.smoke_wall_valid:
                stamp = response.map_header.stamp
                measured_wall_origins[stamp.sec * 10**9 + stamp.nanosec] = deepcopy(
                    response.ray_origin.point)
            return response

        fcu.geometry_service.callback = near_smoke_wall
        sensors.smoke_regions = [1, 2]
        source = {}

        def fire_guidance():
            with fcu.lock:
                pose = deepcopy(fcu.pose)
            if not source:
                return 0.0, 0.0
            return (angular_distance(source['yaw'], yaw_from_quaternion(pose.orientation)),
                    0.2 if pose.position.z < source['z'] - 0.03 else 0.0)

        sensors.fire_guidance = fire_guidance

        def smoke_guidance():
            with fcu.lock:
                yaw = yaw_from_quaternion(fcu.pose.orientation)
            return angular_distance(source.get('smoke_yaw', yaw), yaw), 0.0

        sensors.smoke_guidance = smoke_guidance
        original_plan = fcu.plan_to_pose
        recovery_targets = []

        def record_terrain_motion(request, response):
            response = original_plan(request, response)
            with fcu.lock:
                recovery_targets.append(deepcopy(request.target))
            return response

        fcu.fake_services[-1].callback = record_terrain_motion
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(fcu)
        executor.add_node(sensors)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        process = subprocess.Popen(
            [*executable, entrypoint, *arguments], stdout=log, stderr=subprocess.STDOUT)
        monitor_entrypoint = str(Path(get_package_prefix('kv_executive'))
                                 / 'lib' / 'kv_executive' / 'detection_monitor_node.py')
        monitor_process = subprocess.Popen(
            [*monitor_executable, monitor_entrypoint, *arguments,
             '-p', 'smoke_topic:=smoke/detection', '-p', 'fire_topic:=fire/detection'],
            stdout=monitor_log, stderr=subprocess.STDOUT)

        def alive_wait(predicate, seconds, description):
            def check():
                assert process.poll() is None, (
                    f'Executive exited {process.returncode}; log:\n{log_path.read_text()}')
                assert monitor_process.poll() is None, (
                    f'Monitor exited {monitor_process.returncode}; '
                    f'log:\n{monitor_log_path.read_text()}')
                assert active is None or active.status != GoalStatus.STATUS_ABORTED, (
                    f'Mission aborted while waiting for {description}; '
                    f'executive log:\n{log_path.read_text()}')
                return predicate()
            wait_for(check, seconds, description)

        alive_wait(lambda: fcu.mission_client.server_is_ready()
                   and sensors.smoke_publisher.get_subscription_count() >= 2,
                   8.0, 'isolated executive discovery')
        alive_wait(lambda: fcu.status_publisher.get_subscription_count() > 0
                   and fcu.odometry_publisher.get_subscription_count() > 0
                   and fcu.gps_publisher.get_subscription_count() > 0,
                   8.0, 'executive FCU telemetry subscriptions')
        # Action discovery can finish before the child's state reader exists.
        # Publish a fresh heartbeat after telemetry discovery and require its
        # reliable DDS acknowledgement before admitting the first mission.
        with fcu.lock:
            fcu.next_heartbeat = 0.0
        fcu.publish_telemetry()
        assert fcu.status_publisher.wait_for_all_acked(Duration(seconds=3.0))
        for label, child in (('executive', process), ('monitor', monitor_process)):
            # Retain the actual mapped interpreter/libraries for comparing
            # runtime controls; never infer this from the requested prefix.
            (tmp_path / f'{label}.maps').write_text(
                Path(f'/proc/{child.pid}/maps').read_text())
        mission = SimpleNamespace(fcu=fcu)
        repetitions = int(os.environ.get('KV_VERIFY_PROCESS_REPETITIONS', '3'))
        assert 1 <= repetitions <= 10
        for repetition in range(repetitions):
            source.clear()
            with sensors.lock:
                sensors.source = None
                first_report = len(sensors.inspection_reports)
            with fcu.lock:
                first_status = len(fcu.statuses)
                first_target = len(recovery_targets)
            active = send_goal(mission, skip_takeoff=True)
            assert active.accepted
            alive_wait(lambda: any(text == 'SWEEP: started'
                                   for _, text in fcu.statuses[first_status:]),
                       6.0, 'new Exploration phase')
            with fcu.lock:
                yaw = yaw_from_quaternion(fcu.pose.orientation)
                source['smoke_yaw'] = yaw
                fcu.inspection_target = Point(
                    x=fcu.pose.position.x + 20.0 * math.cos(yaw),
                    y=fcu.pose.position.y + 20.0 * math.sin(yaw), z=fcu.pose.position.z)
            with sensors.lock:
                sensors.source = 'smoke'
            alive_wait(lambda: any(
                text == 'Verify: approach destination reached; reacquiring smoke or fire'
                for _, text in fcu.statuses[first_status:]),
                       25.0, 'short terrain-aware smoke approach arrival')
            with fcu.lock:
                source.update(yaw=source['smoke_yaw'] + 0.2,
                              z=fcu.pose.position.z + 0.5)
            fire_started = time.monotonic()
            with sensors.lock:
                sensors.source = 'smoke_fire'
            alive_wait(lambda: len(sensors.inspection_reports) > first_report,
                       90.0, 'bounded yaw and altitude correction followed by fire inspection')
            with sensors.lock:
                sensors.source = None
                report = deepcopy(sensors.inspection_reports[first_report])
                report_time = sensors.inspection_report_times[first_report]
            alive_wait(lambda: time.monotonic() - fire_started >= 9.0,
                       12.0, 'process survival beyond the reported motion-crash window')
            with fcu.lock:
                targets = deepcopy(recovery_targets[first_target:])
                statuses = list(fcu.statuses[first_status:])
            assert report.fire_found and report.source_count == 2
            assert report.smoke_location_valid and report.smoke_wall_confirmed
            assert report.smoke_location_method == 'confirmed_smoke_wall_nearest_hit'
            assert report.smoke_planning_position.header.frame_id == 'px4_odom'
            assert not report.evidence_complete
            assert not report.rgb_evidence.valid and not report.thermal_evidence.valid
            assert not report.rgb_evidence.image.data and not report.thermal_evidence.image.data
            assert abs(report.observer_local_pose.pose.position.z - source['z']) < 0.03
            observed_yaw = yaw_from_quaternion(report.observer_local_pose.pose.orientation)
            assert abs(angular_distance(source['yaw'], observed_yaw)) <= (
                VERIFY_DEFAULTS['verify_fire_heading_tolerance'])
            assert targets  # Terrain-checked smoke translation still precedes yaw centering.
            before_report = [(stamp, text) for stamp, text in statuses if stamp <= report_time]
            assert any(text.startswith('Verify: stopped fire heading alignment;')
                       for _, text in before_report)
            assert any(text.startswith('Verify: stopped fire heading aligned;')
                       for _, text in before_report)
            stationary = [stamp for stamp, text in before_report
                          if stamp >= fire_started
                          and text.startswith('Verify: stationary observation ready;')]
            post_stop_maps = [stamp for stamp, text in before_report
                              if stamp >= fire_started
                              and text.startswith('Verify: post-settle map update received;')]
            last_loiter = max(stamp for stamp, text in before_report
                              if text.startswith('FCU: OFFBOARD -> AUTO.LOITER'))
            assert stationary and last_loiter <= max(stationary) <= report_time
            # Two fresh maps established arrival before the final checked
            # height correction. Reporting settles again without re-ranging.
            assert post_stop_maps and max(post_stop_maps) <= report_time
            assert (report.smoke_planning_position.header.stamp.sec * 10**9
                    + report.smoke_planning_position.header.stamp.nanosec) <= (
                        report.header.stamp.sec * 10**9 + report.header.stamp.nanosec)
            with fcu.lock:
                strict = deepcopy(fcu.inspection_planning_requests)
            assert strict and all(request.terrain_envelope and request.minimum_radius == 0.0
                                  for request in strict)
            assert any(text == 'Verify/REACQUIRE: started' for _, text in statuses)
            assert any(text == 'Verify/CENTER_FIRE: started' for _, text in statuses)
            alive_wait(lambda: any(
                text == 'Verify/DEPART_SITE: handoff complete; resuming Exploration'
                for _, text in fcu.statuses[first_status:]),
                90.0, 'checked alternate-view departure before the next Exploration phase')
            alive_wait(lambda: bool(sensors.cluster_snapshots)
                       and len(sensors.cluster_snapshots[-1].clusters) == repetition + 1,
                       5.0, 'retained completed locations across mission requests')
            with fcu.lock:
                departed = deepcopy(fcu.pose.position)
                statuses = list(fcu.statuses[first_status:])
            cluster = next(value for value in sensors.cluster_snapshots[-1].clusters
                           if value.site_id == report.site_id)
            measured_stamp = report.smoke_planning_position.header.stamp
            measured_origin = measured_wall_origins[
                measured_stamp.sec * 10**9 + measured_stamp.nanosec]
            _assert_directional_cluster(report, cluster, measured_origin)
            assert math.hypot(departed.x - cluster.center.point.x,
                              departed.y - cluster.center.point.y) >= cluster.radius + 1.0
            with fcu.lock:
                # Publication precedes discretionary departure translation.
                planned_departure = [request for request in fcu.inspection_planning_requests
                                     if request.allow_completed_cluster_escape]
                assert all(request.terrain_envelope and request.minimum_agl >= 3.0
                           for request in planned_departure)
                assert planned_departure or _point_position(departed) == pytest.approx(
                    _point_position(report.observer_local_pose.pose.position), abs=0.03)
            departure_at = next(stamp for stamp, text in statuses
                                if text == 'Verify/DEPART_SITE: started')
            handoff_at = next(stamp for stamp, text in statuses
                              if text == 'Verify/DEPART_SITE: handoff complete; '
                              'resuming Exploration')
            assert report_time <= departure_at <= handoff_at
            with sensors.lock:
                sensors.source = None
            cancel_goal(active)
            active = None
            with fcu.lock:
                assert fcu.state.mode == 'AUTO.LOITER'
                assert not fcu.offboard_losses
                assert not fcu.arming_requests
                assert not {'AUTO.TAKEOFF', 'AUTO.RTL', 'AUTO.LAND'} & set(fcu.mode_requests)
                # Each repeated goal models a genuinely different site. A
                # mere 2 m translation must no longer reopen the old source.
                fcu.pose.position.y += 100.0
                fcu.pose.orientation = quaternion_from_yaw(0.0)
                fcu.inspection_target = fcu.inspection_obstacle = None
            # Let both executable subscribers receive the new simulated site's
            # telemetry/TF before admitting another goal and source image.
            fcu.publish_telemetry()
            time.sleep(0.4)
            print(f'centering cycle {repetition + 1}/{repetitions} survived', flush=True)
        assert process.poll() is None
        assert monitor_process.poll() is None
    finally:
        if (active is not None and process is not None and process.poll() is None
                and active.status not in (GoalStatus.STATUS_SUCCEEDED,
                                          GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)):
            # Failure cleanup must still reap both processes if the goal
            # finishes between the status check and cancellation response.
            canceled = active.cancel_goal_async()
            deadline = time.monotonic() + 3.0
            while not canceled.done() and process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.01)
        for child in (process, monitor_process):
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=5.0)
        if sensors is not None:
            with sensors.lock:
                sensors.closed = True
                sensors.timer.cancel()
        if fcu is not None:
            fcu.telemetry_timer.cancel()
            fcu.planning_release.set()
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
            pool = getattr(executor, '_executor', None)
            if pool is not None:
                pool.shutdown(wait=True)
        if thread is not None:
            thread.join(timeout=3.0)
        if sensors is not None:
            sensors.destroy_node()
        if fcu is not None:
            fcu.destroy_node()
        rclpy.shutdown()
        log.close()
        monitor_log.close()
