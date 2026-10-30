"""Run battery and smoke/fire monitoring independently of autonomy."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start both monitors with shared clock and detector topic settings."""
    package_share = FindPackageShare('kv_executive')
    drone_name = LaunchConfiguration('drone_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_name', default_value='kv_drone',
            description='Drone topic namespace; use the same value as autonomy',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock; set false for a real vehicle',
        ),
        DeclareLaunchArgument(
            'start_battery_monitor', default_value='true',
            description='Start battery monitoring',
        ),
        DeclareLaunchArgument(
            'start_detection_monitor', default_value='true',
            description='Enter Verify from Exploration for confirmed smoke or fire',
        ),
        DeclareLaunchArgument(
            'battery_monitor_config',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'battery_monitor.yaml',
            ]),
            description='Path to the battery monitor ROS 2 parameter YAML file',
        ),
        DeclareLaunchArgument(
            'detection_monitor_config',
            default_value=PathJoinSubstitution([
                package_share, 'config', 'detection_monitor.yaml',
            ]),
            description='Path to the detection monitor ROS 2 parameter YAML file',
        ),
        DeclareLaunchArgument(
            'detection_monitor_udp_only', default_value='true',
            description=(
                'Use UDP-only Fast DDS for the detection monitor native-crash workaround; '
                'false preserves inherited transport/discovery settings'
            ),
        ),
        DeclareLaunchArgument(
            'smoke_detection_topic',
            default_value=['/', drone_name, '/smoke/detection'],
            description='Confirmed smoke observations from kv_perception',
        ),
        DeclareLaunchArgument(
            'fire_detection_topic',
            default_value=['/', drone_name, '/fire/detection'],
            description='Confirmed thermal fire observations from kv_perception',
        ),
        DeclareLaunchArgument(
            'action_status_topic', default_value='execute_mission/_action/status',
            description='ExecuteMission status topic in the executive namespace',
        ),
        DeclareLaunchArgument(
            'mission_state_request_topic', default_value='mission_state_request_for_goal',
            description='Goal-scoped Verify request topic in the executive namespace',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                package_share, 'launch', 'battery_monitor.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('start_battery_monitor')),
            launch_arguments={
                'battery_monitor_config': LaunchConfiguration('battery_monitor_config'),
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                package_share, 'launch', 'detection_monitor.launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('start_detection_monitor')),
            launch_arguments={
                'drone_name': drone_name,
                'detection_monitor_config': LaunchConfiguration('detection_monitor_config'),
                'detection_monitor_udp_only': LaunchConfiguration('detection_monitor_udp_only'),
                'smoke_detection_topic': LaunchConfiguration('smoke_detection_topic'),
                'fire_detection_topic': LaunchConfiguration('fire_detection_topic'),
                'action_status_topic': LaunchConfiguration('action_status_topic'),
                'mission_state_request_topic': LaunchConfiguration('mission_state_request_topic'),
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
