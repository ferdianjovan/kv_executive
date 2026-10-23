"""Launch the mission executive, planner and optional passive monitor."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _executive_transport_environment(context):
    """Apply the UDP workaround within the executive's scoped environment."""
    if not IfCondition(LaunchConfiguration('executive_udp_only')).evaluate(context):
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
        FindPackageShare('kv_executive'), 'config', filename,
    ])
    actions = [
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', profile),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', profile),
    ]
    if local_only:
        # Jazzy LOCALHOST discovery appends SHM even with builtins disabled.
        # The loopback-only XML preserves its network scope instead.
        actions.append(SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SYSTEM_DEFAULT'))
        if legacy_local:
            actions.append(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'))
    actions.append(LogInfo(
        msg='Mission executive: Fast DDS UDP-only transport (shared memory disabled)',
    ))
    return actions


def generate_launch_description():
    """Configure autonomy with independently selectable MAVROS and monitor startup."""
    drone_name = LaunchConfiguration('drone_name')
    planner_config = LaunchConfiguration('planner_config')
    mission_config = LaunchConfiguration('mission_config')
    battery_monitor_config = LaunchConfiguration('battery_monitor_config')
    external_odometry_topic = LaunchConfiguration('external_odometry_topic')
    external_odom_parent_frame = LaunchConfiguration(
        'external_odom_parent_frame'
    )
    external_odom_child_frame = LaunchConfiguration(
        'external_odom_child_frame'
    )
    default_planner_config = PathJoinSubstitution(
        [FindPackageShare('kv_executive'), 'config', 'planner.yaml']
    )
    default_mission_config = PathJoinSubstitution(
        [FindPackageShare('kv_executive'), 'config', 'mission.yaml']
    )
    default_battery_monitor_config = PathJoinSubstitution(
        [FindPackageShare('kv_executive'), 'config', 'battery_monitor.yaml']
    )
    px4_launch = PathJoinSubstitution(
        [FindPackageShare('kv_executive'), 'launch', 'px4.launch.py']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'drone_name',
                default_value='x500_depth',
                description='Drone topic namespace and TF frame prefix',
            ),
            DeclareLaunchArgument(
                'planner_config',
                default_value=default_planner_config,
                description='Path to the motion planner ROS 2 parameter YAML file',
            ),
            DeclareLaunchArgument(
                'mission_config',
                default_value=default_mission_config,
                description='Path to the mission executive ROS 2 parameter YAML file',
            ),
            DeclareLaunchArgument(
                'executive_udp_only',
                default_value='true',
                description=(
                    'Use UDP-only Fast DDS for the executive native-crash workaround; '
                    'false preserves inherited transport/discovery settings'
                ),
            ),
            DeclareLaunchArgument(
                'start_battery_monitor',
                default_value='false',
                description='Also start the battery monitor here; normally launched separately',
            ),
            DeclareLaunchArgument(
                'battery_monitor_config',
                default_value=default_battery_monitor_config,
                description='Path to the battery monitor ROS 2 parameter YAML file',
            ),
            DeclareLaunchArgument(
                'auto_start_mission',
                default_value='false',
                description='Start one mission automatically when the executive starts',
            ),
            DeclareLaunchArgument(
                'robot_base_frame',
                default_value=[drone_name, '/base_link'],
                description='Body frame used to align map paths with PX4 local odometry',
            ),
            DeclareLaunchArgument(
                'external_odometry_topic',
                default_value=['/', drone_name, '/odometry'],
                description=(
                    'External nav_msgs/Odometry topic; defaults to '
                    '/<drone_name>/odometry'
                ),
            ),
            DeclareLaunchArgument(
                'home_from_first_odometry',
                default_value='true',
                description='Use the first external odometry pose as planner home',
            ),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'start_mavros',
                default_value='false',
                description='Start the external-odometry MAVROS launch',
            ),
            DeclareLaunchArgument(
                'fcu_url',
                default_value='udp://:14540@127.0.0.1:14580',
            ),
            DeclareLaunchArgument(
                'external_odom_parent_frame',
                default_value=[drone_name, '/odom'],
                description='Defaults to <drone_name>/odom',
            ),
            DeclareLaunchArgument(
                'external_odom_child_frame',
                default_value=[drone_name, '/base_link'],
                description='Defaults to <drone_name>/base_link',
            ),
            DeclareLaunchArgument('configure_px4', default_value='true'),
            DeclareLaunchArgument(
                'ekf2_hgt_ref',
                default_value='1',
                description=(
                    'PX4 height reference: barometer=0, GPS=1, range=2, '
                    'vision=3, or -1 to preserve'
                ),
            ),
            DeclareLaunchArgument(
                'publish_frame_conversions',
                default_value='true',
                description='Publish MAVROS ENU/NED and FLU/FRD frame conversions',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(px4_launch),
                condition=IfCondition(LaunchConfiguration('start_mavros')),
                launch_arguments={
                    'drone_name': drone_name,
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'external_odometry_topic': external_odometry_topic,
                    'external_odom_parent_frame': external_odom_parent_frame,
                    'external_odom_child_frame': external_odom_child_frame,
                    'configure_px4': LaunchConfiguration('configure_px4'),
                    'ekf2_hgt_ref': LaunchConfiguration('ekf2_hgt_ref'),
                    'publish_frame_conversions': LaunchConfiguration(
                        'publish_frame_conversions'
                    ),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            ),
            Node(
                package='kv_executive',
                executable='motion_planning',
                name='motion_planner',
                output='screen',
                parameters=[
                    planner_config,
                    {
                        'odometry_topic': external_odometry_topic,
                        'home_from_first_odometry': ParameterValue(
                            LaunchConfiguration('home_from_first_odometry'),
                            value_type=bool,
                        ),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool
                        ),
                    },
                ],
            ),
            GroupAction(actions=[
                OpaqueFunction(function=_executive_transport_environment),
                Node(
                    package='kv_executive',
                    executable='executive_node.py',
                    name='mission_executive',
                    output='screen',
                    parameters=[
                        mission_config,
                        {
                            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                            'auto_start_mission': ParameterValue(
                                LaunchConfiguration('auto_start_mission'),
                                value_type=bool,
                            ),
                            'use_sim_time': ParameterValue(
                                LaunchConfiguration('use_sim_time'), value_type=bool
                            ),
                        },
                    ],
                ),
            ]),
            Node(
                package='kv_executive',
                executable='battery_monitor_node',
                name='battery_monitor',
                condition=IfCondition(LaunchConfiguration('start_battery_monitor')),
                output='screen',
                parameters=[
                    battery_monitor_config,
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
