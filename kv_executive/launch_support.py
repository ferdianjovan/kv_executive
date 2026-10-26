"""Scoped launch settings for the Fast DDS shared-memory crash workaround."""

from launch.actions import LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
