from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    planner_config = LaunchConfiguration("planner_config")
    default_planner_config = PathJoinSubstitution(
        [FindPackageShare("kv_executive"), "config", "planner.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "planner_config",
                default_value=default_planner_config,
                description="Path to the motion planner ROS 2 parameter YAML file",
            ),
            Node(
                package="kv_executive",
                executable="motion_planning",
                name="motion_planner",
                output="screen",
                parameters=[planner_config],
            ),
        ]
    )
