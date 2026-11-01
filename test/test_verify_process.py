"""Repeat smoke approach and bounded fire centering in an isolated process.

KV_VERIFY_PROCESS_PYTHON selects a compatible Python executable or launcher.
Only a private simulated FCU is used; every MAVROS endpoint is remapped.
"""

from copy import deepcopy
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from kv_executive.mission_geometry import angular_distance, yaw_from_quaternion
from kv_executive.verification import VERIFY_DEFAULTS
import rclpy
from rclpy.executors import MultiThreadedExecutor

from test_mission_action import cancel_goal, send_goal, SimulatedFcu, wait_for
from test_verify_ros import MonitoringSensors


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
        original_geometry = fcu.query_inspection_geometry

        def near_smoke_wall(request, response):
            response = original_geometry(request, response)
            if request.smoke_wall_width:
                with sensors.lock:
                    near_fire = sensors.source in {'fire', 'smoke_fire'}
                response.smoke_wall_valid = (
                    request.origin.header.frame_id == 'test_thermal_camera' or near_fire)
                response.smoke_wall_range = response.smoke_wall_nearest_range = (
                    VERIFY_DEFAULTS['verify_smoke_standoff'] + 0.4)
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
                return predicate()
            wait_for(check, seconds, description)

        alive_wait(lambda: fcu.mission_client.server_is_ready()
                   and sensors.smoke_publisher.get_subscription_count() >= 2,
                   8.0, 'isolated executive discovery')
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
            with sensors.lock:
                sensors.source = 'smoke'
            alive_wait(lambda: any(
                text == 'Verify: approach destination reached; reacquiring smoke or fire'
                for _, text in fcu.statuses[first_status:]),
                       25.0, 'short terrain-aware smoke approach arrival')
            with fcu.lock:
                source.update(yaw=yaw_from_quaternion(fcu.pose.orientation) + 0.2,
                              z=fcu.pose.position.z + 0.5)
            fire_started = time.monotonic()
            with sensors.lock:
                sensors.source = 'smoke_fire'
            alive_wait(lambda: len(sensors.inspection_reports) > first_report,
                       35.0, 'bounded yaw and altitude correction followed by fire inspection')
            with sensors.lock:
                sensors.source = None
                report = deepcopy(sensors.inspection_reports[first_report])
            alive_wait(lambda: time.monotonic() - fire_started >= 9.0,
                       12.0, 'process survival beyond the reported motion-crash window')
            with fcu.lock:
                targets = deepcopy(recovery_targets[first_target:])
                statuses = list(fcu.statuses[first_status:])
            assert report.fire_found and report.source_count == 2
            assert abs(report.observer_local_pose.pose.position.z - source['z']) < 0.03
            assert targets  # Terrain-checked smoke translation still precedes yaw centering.
            assert any('one bounded fire-heading correction' in text for _, text in statuses)
            with fcu.lock:
                strict = deepcopy(fcu.inspection_planning_requests)
            assert strict and all(request.terrain_envelope and request.minimum_radius == 0.0
                                  for request in strict)
            assert any(text == 'Verify/REACQUIRE: started' for _, text in statuses)
            assert any(text == 'Verify/CENTER_FIRE: started' for _, text in statuses)
            with sensors.lock:
                sensors.source = None
            cancel_goal(active)
            active = None
            with fcu.lock:
                assert fcu.state.mode == 'AUTO.LOITER'
                assert not fcu.offboard_losses
                assert not fcu.arming_requests
                assert not {'AUTO.TAKEOFF', 'AUTO.RTL', 'AUTO.LAND'} & set(fcu.mode_requests)
            print(f'centering cycle {repetition + 1}/{repetitions} survived', flush=True)
        assert process.poll() is None
        assert monitor_process.poll() is None
    finally:
        if active is not None and process is not None and process.poll() is None:
            cancel_goal(active)
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
