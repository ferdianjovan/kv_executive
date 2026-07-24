import json
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _parse_bool(value):
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _remove_runtime_config(_context, config_path):
    Path(config_path).unlink(missing_ok=True)
    return []


def _static_rotation(
    namespace, name, parent_frame, child_frame, roll, yaw, use_sim_time
):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        name=name,
        output='screen',
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--roll',
            roll,
            '--pitch',
            '0',
            '--yaw',
            yaw,
            '--frame-id',
            parent_frame,
            '--child-frame-id',
            child_frame,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('tf_static', '/tf_static')],
    )


def _launch_setup(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    external_topic = LaunchConfiguration('external_odometry_topic').perform(context)
    odom_parent = LaunchConfiguration('external_odom_parent_frame').perform(context)
    odom_child = LaunchConfiguration('external_odom_child_frame').perform(context)
    local_frame = LaunchConfiguration('mavros_local_frame').perform(context)
    use_sim_time = _parse_bool(LaunchConfiguration('use_sim_time').perform(context))
    publish_frame_conversions = _parse_bool(
        LaunchConfiguration('publish_frame_conversions').perform(context)
    )

    mavros_share = get_package_share_directory('mavros')
    executive_share = get_package_share_directory('kv_executive')

    # The odometry plugin is a MAVROS sub-node, so its frame overrides must be
    # represented as wildcard YAML rather than parameters on mavros_node.
    runtime_config = {
        '/**/odometry': {
            'ros__parameters': {
                # These parameters describe odometry received from the FCU.
                # PX4 keeps its own local origin, so do not label it as the
                # external odometry frame.
                'fcu.odom_parent_id_des': local_frame,
                'fcu.odom_child_id_des': odom_child,
                'fcu.map_id_des': local_frame,
            }
        },
        '/**/local_position': {
            'ros__parameters': {
                # PX4 local position retains the EKF local origin. Do not call
                # that frame 'map' when the external map has another origin.
                'frame_id': local_frame,
                'tf.send': False,
            }
        },
    }
    fd, runtime_config_path = tempfile.mkstemp(
        prefix='kv_mavros_external_odometry_', suffix='.yaml'
    )
    os.close(fd)
    Path(runtime_config_path).write_text(
        json.dumps(runtime_config, indent=2), encoding='utf-8'
    )

    mavros_prefix = f'/{namespace}' if namespace else ''
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        namespace=namespace,
        output='screen',
        parameters=[
            os.path.join(executive_share, 'config', 'mavros_plugins.yaml'),
            os.path.join(mavros_share, 'launch', 'px4_config.yaml'),
            runtime_config_path,
            {
                'fcu_url': ParameterValue(
                    LaunchConfiguration('fcu_url'), value_type=str
                ),
                'gcs_url': ParameterValue(
                    LaunchConfiguration('gcs_url'), value_type=str
                ),
                'tgt_system': ParameterValue(
                    LaunchConfiguration('tgt_system'), value_type=int
                ),
                'tgt_component': ParameterValue(
                    LaunchConfiguration('tgt_component'), value_type=int
                ),
                'fcu_protocol': ParameterValue(
                    LaunchConfiguration('fcu_protocol'), value_type=str
                ),
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            # MAVROS "out" means ROS odometry going out to the FCU. "in" is
            # ODOMETRY received from the FCU.
            (f'{mavros_prefix}/odometry/out', external_topic),
        ],
    )

    frame_conversion_nodes = []
    if publish_frame_conversions:
        # The MAVROS odometry plugin appends "_ned" and "_frd" to the incoming
        # nav_msgs/Odometry frame IDs and looks up these exact transforms.
        frame_conversion_nodes.append(
            _static_rotation(
                namespace,
                'external_odom_enu_to_ned',
                odom_parent,
                f'{odom_parent}_ned',
                '3.141592653589793',
                '1.5707963267948966',
                use_sim_time,
            )
        )
        if local_frame != odom_parent:
            frame_conversion_nodes.append(
                _static_rotation(
                    namespace,
                    'px4_odom_enu_to_ned',
                    local_frame,
                    f'{local_frame}_ned',
                    '3.141592653589793',
                    '1.5707963267948966',
                    use_sim_time,
                )
            )
        frame_conversion_nodes.append(
            _static_rotation(
                namespace,
                'base_link_flu_to_frd',
                odom_child,
                f'{odom_child}_frd',
                '3.141592653589793',
                '0',
                use_sim_time,
            )
        )

    configurator = Node(
        package='kv_executive',
        executable='configure_px4_external_odometry.py',
        name='configure_px4_external_odometry',
        output='screen',
        condition=IfCondition(LaunchConfiguration('configure_px4')),
        parameters=[
            {
                'mavros_parameter_node': f'{mavros_prefix}/param',
                'external_odometry_topic': external_topic,
                'expected_parent_frame': odom_parent,
                'expected_child_frame': odom_child,
                'timeout': ParameterValue(
                    LaunchConfiguration('px4_parameter_timeout'), value_type=float
                ),
                'ekf2_ev_ctrl': ParameterValue(
                    LaunchConfiguration('ekf2_ev_ctrl'), value_type=int
                ),
                'ekf2_hgt_ref': ParameterValue(
                    LaunchConfiguration('ekf2_hgt_ref'), value_type=int
                ),
                'ekf2_ev_pos_x': ParameterValue(
                    LaunchConfiguration('ekf2_ev_pos_x'), value_type=float
                ),
                'ekf2_ev_pos_y': ParameterValue(
                    LaunchConfiguration('ekf2_ev_pos_y'), value_type=float
                ),
                'ekf2_ev_pos_z': ParameterValue(
                    LaunchConfiguration('ekf2_ev_pos_z'), value_type=float
                ),
                'use_sim_time': False,
            }
        ],
    )

    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=_remove_runtime_config,
                    args=[runtime_config_path],
                )
            ]
        )
    )

    return [
        LogInfo(
            msg=(
                f'MAVROS external odometry: {external_topic} '
                f'({odom_parent} -> {odom_child})'
            )
        ),
        *frame_conversion_nodes,
        mavros_node,
        configurator,
        cleanup,
    ]


def generate_launch_description():
    drone_name = LaunchConfiguration('drone_name')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'fcu_url',
                default_value='udp://:14540@127.0.0.1:14580',
                description='MAVROS connection URL for PX4',
            ),
            DeclareLaunchArgument('gcs_url', default_value=''),
            DeclareLaunchArgument('tgt_system', default_value='1'),
            DeclareLaunchArgument('tgt_component', default_value='1'),
            DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
            DeclareLaunchArgument('namespace', default_value='mavros'),
            DeclareLaunchArgument(
                'drone_name',
                default_value='x500_depth',
                description='Drone topic namespace and TF frame prefix',
            ),
            DeclareLaunchArgument(
                'external_odometry_topic',
                default_value=['/', drone_name, '/odometry'],
                description=(
                    'nav_msgs/Odometry estimate sent to PX4; defaults to '
                    '/<drone_name>/odometry'
                ),
            ),
            DeclareLaunchArgument(
                'external_odom_parent_frame',
                default_value=[drone_name, '/odom'],
                description=(
                    'Expected odometry header.frame_id; defaults to '
                    '<drone_name>/odom'
                ),
            ),
            DeclareLaunchArgument(
                'external_odom_child_frame',
                default_value=[drone_name, '/base_link'],
                description=(
                    'Expected odometry child_frame_id; defaults to '
                    '<drone_name>/base_link'
                ),
            ),
            DeclareLaunchArgument(
                'mavros_local_frame',
                default_value='px4_odom',
                description='Frame label for the independent PX4 EKF local origin',
            ),
            DeclareLaunchArgument(
                'publish_frame_conversions',
                default_value='true',
                description=(
                    'Publish the ENU-to-NED and FLU-to-FRD rotations required '
                    'by the MAVROS odometry plugin'
                ),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'configure_px4',
                default_value='true',
                description='Set PX4 EKF2 external-vision fusion parameters through MAVROS',
            ),
            DeclareLaunchArgument(
                'ekf2_ev_ctrl',
                default_value='15',
                description=(
                    'PX4 fusion bitmask: horizontal position=1, vertical '
                    'position=2, velocity=4, yaw=8'
                ),
            ),
            DeclareLaunchArgument(
                'ekf2_hgt_ref',
                default_value='1',
                description=(
                    'PX4 height reference: barometer=0, GPS=1, range=2, '
                    'vision=3, or -1 to preserve. Changes require an FCU reboot'
                ),
            ),
            DeclareLaunchArgument('ekf2_ev_pos_x', default_value='0.0'),
            DeclareLaunchArgument('ekf2_ev_pos_y', default_value='0.0'),
            DeclareLaunchArgument('ekf2_ev_pos_z', default_value='0.0'),
            DeclareLaunchArgument('px4_parameter_timeout', default_value='60.0'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
