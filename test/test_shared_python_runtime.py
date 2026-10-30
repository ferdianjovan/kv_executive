"""Check interpreter compatibility and permanent launch scope without flight commands."""

import importlib.util
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys

from ament_index_python.packages import get_package_prefix
from kv_executive.launch_support import SharedPythonPrefix
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
import pytest


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def local_launch_logs(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))


@pytest.fixture
def runtime():
    executable = Path(get_package_prefix('kv_executive')) / (
        'lib/kv_executive/python_shared_runtime')
    assert executable.is_file(), 'Build kv_executive before running the runtime regression'
    return [str(executable), '--python', sys.executable, '--']


def run(runtime, *arguments):
    return subprocess.run(
        [*runtime, *arguments], capture_output=True, text=True, timeout=15,
    )


def test_runtime_keeps_sdk_and_imports_generated_interfaces(runtime):
    result = run(runtime, '-c', """
import ctypes
import json
import sys
import tf2_py
import yasmin
from rclpy.serialization import deserialize_message, serialize_message
from kv_executive.msg import InspectionReport, MissionStateRequest
from kv_executive.srv import PlanInspectionPath, QueryInspectionGeometry
from kv_perception.msg import FireDetection, SmokeDetection
types = [InspectionReport, MissionStateRequest, PlanInspectionPath.Request,
         PlanInspectionPath.Response, QueryInspectionGeometry.Request,
         QueryInspectionGeometry.Response, FireDetection, SmokeDetection]
for cls in types:
    message = cls()
    assert deserialize_message(serialize_message(message), cls) == message
maps = open('/proc/self/maps').read()
assert 'libpython3.12.so' in maps
print(json.dumps({'executable': sys.executable, 'prefix': sys.prefix,
                  'base_prefix': sys.base_prefix, 'version': list(sys.version_info[:3]),
                  'runtime_address': ctypes.addressof(
                      ctypes.c_char.in_dll(ctypes.pythonapi, '_PyRuntime'))}))
""")
    assert result.returncode == 0, result.stderr
    state = json.loads(result.stdout)
    assert state['executable'] == sys.executable
    assert state['prefix'] == sys.prefix
    assert state['base_prefix'] == sys.base_prefix
    assert state['version'] == list(sys.version_info[:3])
    assert state['runtime_address'] > 2**32


def test_arguments_and_script_paths_are_preserved(runtime, tmp_path):
    script = tmp_path / 'script with spaces.py'
    script.write_text('import json, sys; print(json.dumps(sys.argv))\n')
    result = run(runtime, str(script), 'space value', '$(literal)', '--ros-args')
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == [str(script), 'space value', '$(literal)', '--ros-args']


@pytest.mark.parametrize('argument', ['-m', '-c'])
def test_python_command_line_modes(runtime, argument):
    arguments = ['json.tool', '--help'] if argument == '-m' else ['print("command mode")']
    result = run(runtime, argument, *arguments)
    assert result.returncode == 0, result.stderr
    assert 'usage:' in result.stdout if argument == '-m' else result.stdout == 'command mode\n'


def test_exit_status_and_exceptions_are_preserved(runtime):
    assert run(runtime, '-c', 'raise SystemExit(7)').returncode == 7
    result = run(runtime, '-c', 'raise RuntimeError("expected failure")')
    assert result.returncode == 1
    assert 'RuntimeError: expected failure' in result.stderr


def test_python_signal_handler_remains_active(runtime):
    process = subprocess.Popen(
        [*runtime, '-u', '-c', 'import time; print("ready"); time.sleep(30)'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        assert process.stdout.readline().strip() == 'ready'
        process.send_signal(signal.SIGINT)
        _, stderr = process.communicate(timeout=5)
        assert process.returncode in (-signal.SIGINT, 128 + signal.SIGINT)
        assert 'KeyboardInterrupt' in stderr
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)


def test_runtime_prefix_is_scoped_and_shell_quoted(monkeypatch, tmp_path):
    prefix = tmp_path / 'package with spaces'
    binary = prefix / 'lib/kv_executive/python_shared_runtime'
    binary.parent.mkdir(parents=True)
    binary.touch()
    monkeypatch.setattr('kv_executive.launch_support.get_package_prefix', lambda _: str(prefix))
    context = LaunchContext()
    before = dict(os.environ)
    value = SharedPythonPrefix('executive_node.py').perform(context)
    assert dict(os.environ) == before
    assert shlex.split(value) == [str(binary), '--python', sys.executable, '--']


@pytest.mark.parametrize('filter_value,expected', [
    ('', 'gdb --args'), ('executive_node.*', 'gdb --args'), ('motion_planning', ''),
])
def test_runtime_composes_matching_debugger_prefix(filter_value, expected, runtime):
    context = LaunchContext()
    context.launch_configurations.update({
        'launch-prefix': 'gdb --args', 'launch-prefix-filter': filter_value,
    })
    assert shlex.split(SharedPythonPrefix('executive_node.py').perform(context)) == (
        shlex.split(expected) + runtime)


@pytest.mark.parametrize('filename', [
    'autonomy.launch.py', 'detection_monitor.launch.py', 'monitoring.launch.py',
])
def test_launch_runtime_is_permanent_and_scoped(filename):
    spec = importlib.util.spec_from_file_location(filename, PACKAGE / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    switches = [entity for entity in description.entities
                if isinstance(entity, DeclareLaunchArgument)
                and entity.name == 'shared_python_runtime']
    assert not switches
    if filename == 'monitoring.launch.py':
        includes = [entity for entity in description.entities
                    if isinstance(entity, IncludeLaunchDescription)]
        arguments = [dict(entity.launch_arguments) for entity in includes]
        assert all('shared_python_runtime' not in value for value in arguments)
        return
    scoped = [child for entity in description.entities if isinstance(entity, GroupAction)
              for child in entity.get_sub_entities() if isinstance(child, Node)]
    assert len(scoped) == 1
    assert any(isinstance(value, SharedPythonPrefix)
               for value in scoped[0].process_description.prefix)
    for entity in description.entities:
        if isinstance(entity, Node):
            assert all(not isinstance(value, SharedPythonPrefix)
                       for value in entity.process_description.prefix)
