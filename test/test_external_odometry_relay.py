"""Exercise the external odometry bridge over an isolated DDS graph."""

import importlib.util
import os
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message


_SPEC = importlib.util.spec_from_file_location(
    'relay_external_odometry',
    Path(__file__).resolve().parents[1] / 'scripts' / 'relay_external_odometry.py',
)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
ExternalOdometryRelay = _MODULE.ExternalOdometryRelay


@pytest.fixture
def ros_context(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    context = Context()
    rclpy.init(args=[], context=context, domain_id=120 + os.getpid() % 50)
    try:
        yield context
    finally:
        rclpy.try_shutdown(context=context)


def _parameters(input_topic, output_topic):
    return [
        Parameter('input_topic', value=input_topic),
        Parameter('output_topic', value=output_topic),
    ]


def test_preserves_complete_estimate_from_best_effort_to_reliable(ros_context):
    prefix = f'/odometry_relay_test_{os.getpid()}'
    relay = ExternalOdometryRelay(
        context=ros_context,
        parameter_overrides=_parameters(f'{prefix}/estimator', f'{prefix}/mavros'),
    )
    peer = Node('odometry_relay_test_peer', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    try:
        executor.add_node(relay)
        executor.add_node(peer)
        source = peer.create_publisher(
            Odometry, f'{prefix}/estimator', qos_profile_sensor_data
        )
        received = []
        # Match MAVROS's reliable subscription, which rejects a best-effort
        # relay output even when the estimator itself is best effort.
        sink = peer.create_subscription(
            Odometry, f'{prefix}/mavros', received.append, 1
        )
        estimate = Odometry()
        estimate.header.stamp.sec = 1234
        estimate.header.stamp.nanosec = 56789
        estimate.header.frame_id = 'kv_drone/odom'
        estimate.child_frame_id = 'kv_drone/base_link'
        estimate.pose.pose.position.x = 1.25
        estimate.pose.pose.position.y = -2.5
        estimate.pose.pose.position.z = 3.75
        estimate.pose.pose.orientation.x = 0.1
        estimate.pose.pose.orientation.y = -0.2
        estimate.pose.pose.orientation.z = 0.3
        estimate.pose.pose.orientation.w = 0.9273618495495703
        estimate.pose.covariance = [index * 0.125 for index in range(36)]
        estimate.twist.twist.linear.x = -4.5
        estimate.twist.twist.linear.y = 5.25
        estimate.twist.twist.linear.z = 6.0
        estimate.twist.twist.angular.x = 0.25
        estimate.twist.twist.angular.y = -0.5
        estimate.twist.twist.angular.z = 0.75
        estimate.twist.covariance = [index * 0.25 for index in range(36)]
        expected = serialize_message(estimate)

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not received:
            if source.get_subscription_count() and sink.get_publisher_count():
                source.publish(estimate)
            executor.spin_once(timeout_sec=0.02)

        assert received, 'Odometry did not reach the reliable MAVROS-like subscriber'
        assert all(serialize_message(message) == expected for message in received)
        assert serialize_message(estimate) == expected
    finally:
        executor.shutdown()
        peer.destroy_node()
        relay.destroy_node()


@pytest.mark.parametrize(
    ('input_topic', 'output_topic', 'cli_args'),
    [
        ('/relay_loop', '/relay_loop', []),
        ('estimate', '/relay_loop_test/estimate', []),
        ('/relay_input', '/relay_output', ['--ros-args', '-r', '/relay_output:=/relay_input']),
    ],
)
def test_rejects_resolved_topic_loops(ros_context, input_topic, output_topic, cli_args):
    with pytest.raises(ValueError, match='different topics'):
        ExternalOdometryRelay(
            context=ros_context,
            namespace='/relay_loop_test',
            cli_args=cli_args,
            parameter_overrides=_parameters(input_topic, output_topic),
        )
