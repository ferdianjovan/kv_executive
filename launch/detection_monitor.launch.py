"""Run smoke/fire monitoring that requests Exploration abort and LOITER."""

from kv_executive.launch_support import fastdds_udp_environment
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _detection_transport_environment(context):
    """Apply the UDP workaround only inside the detection monitor scope."""
    return fastdds_udp_environment(context, 'detection_monitor_udp_only', 'Detection monitor')


def generate_launch_description():
    """Configure detection inputs without starting flight-control nodes."""
    drone_name = LaunchConfiguration('drone_name')
    default_config = PathJoinSubstitution(
        [FindPackageShare('kv_executive'), 'config', 'detection_monitor.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('drone_name', default_value='kv_drone'),
        DeclareLaunchArgument(
            'detection_monitor_config',
            default_value=default_config,
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
            'action_status_topic',
            default_value='execute_mission/_action/status',
            description='ExecuteMission status topic in the executive namespace',
        ),
        DeclareLaunchArgument(
            'mission_state_request_topic',
            default_value='mission_state_request_for_goal',
            description='Goal-scoped Abort request topic in the executive namespace',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock; set false for a real vehicle',
        ),
        GroupAction(actions=[
            OpaqueFunction(function=_detection_transport_environment),
            Node(
                package='kv_executive',
                executable='detection_monitor_node.py',
                name='detection_monitor',
                output='screen',
                additional_env={'PYTHONFAULTHANDLER': '1'},
                parameters=[
                    LaunchConfiguration('detection_monitor_config'),
                    {
                        'smoke_topic': LaunchConfiguration('smoke_detection_topic'),
                        'fire_topic': LaunchConfiguration('fire_detection_topic'),
                        'action_status_topic': LaunchConfiguration('action_status_topic'),
                        'mission_state_request_topic': LaunchConfiguration(
                            'mission_state_request_topic'
                        ),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool,
                        ),
                    },
                ],
            ),
        ]),
    ])
