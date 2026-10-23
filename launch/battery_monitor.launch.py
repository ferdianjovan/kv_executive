"""Run battery monitoring independently of the autonomy launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure the passive monitor without starting flight-control nodes."""
    default_config = PathJoinSubstitution(
        [FindPackageShare('kv_executive'), 'config', 'battery_monitor.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'battery_monitor_config',
            default_value=default_config,
            description='Path to the battery monitor ROS 2 parameter YAML file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock; set false for a real vehicle',
        ),
        Node(
            package='kv_executive',
            executable='battery_monitor_node',
            name='battery_monitor',
            output='screen',
            parameters=[
                LaunchConfiguration('battery_monitor_config'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool,
                    ),
                },
            ],
        ),
    ])
