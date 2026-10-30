"""Scoped interpreter and DDS runtime settings for launched Python nodes."""

from pathlib import Path
import re
import shlex
import sys

from ament_index_python.packages import get_package_prefix

from launch.actions import LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


class SharedPythonPrefix(Substitution):
    """Select the same-version shared runtime for only the specified Python node."""

    def __init__(self, executable_name=''):
        super().__init__()
        self.executable_name = executable_name

    def describe(self):
        return f'shared Python runtime for {self.executable_name}'

    def perform(self, context):
        executable = Path(get_package_prefix('kv_executive')) / (
            'lib/kv_executive/python_shared_runtime')
        if not executable.is_file():
            raise RuntimeError('Build kv_executive before launching its Python nodes')
        arguments = []
        selected = LaunchConfiguration('launch-prefix-filter', default='').perform(context)
        if re.match(selected, self.executable_name) is not None:
            arguments = shlex.split(
                LaunchConfiguration('launch-prefix', default='').perform(context))
        arguments.extend([str(executable), '--python', sys.executable, '--'])
        return shlex.join(arguments)


def fastdds_udp_environment(context, switch_name, label, package_name='kv_executive'):
    """Return environment actions to execute inside a scoped GroupAction.

    Keep other RMW implementations and an explicit opt-out untouched. Preserve
    localhost discovery using the loopback profile; RMW would otherwise add SHM
    again for LOCALHOST even when the XML disables builtin transports.
    """
    if not IfCondition(LaunchConfiguration(switch_name)).evaluate(context):
        return []
    selected_rmw = context.environment.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    if selected_rmw not in ('', 'rmw_fastrtps_cpp'):
        return []
    legacy_local = context.environment.get('ROS_LOCALHOST_ONLY', '').startswith('1')
    discovery = context.environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '')
    # RCL also falls back to LOCALHOST for an invalid discovery-range string.
    local_only = legacy_local or discovery not in ('', 'SYSTEM_DEFAULT', 'SUBNET', 'OFF')
    filename = 'fastdds_udp_localhost.xml' if local_only else 'fastdds_udp.xml'
    profile = PathJoinSubstitution([
        FindPackageShare(package_name), 'config', filename,
    ])
    actions = [
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', profile),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', profile),
    ]
    if local_only:
        actions.append(SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT'))
        if legacy_local:
            actions.append(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'))
    actions.append(LogInfo(
        msg=f'{label}: Fast DDS UDP-only transport (shared memory disabled)',
    ))
    return actions
