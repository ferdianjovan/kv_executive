#!/usr/bin/env python3
"""Forward external odometry to MAVROS without relying on plugin remapping."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ExternalOdometryRelay(Node):
    """Preserve each estimate while bridging the estimator and MAVROS topics."""

    def __init__(self, **kwargs):
        super().__init__('relay_external_odometry', **kwargs)
        input_topic = self.declare_parameter(
            'input_topic', '/external/odometry'
        ).value
        output_topic = self.declare_parameter(
            'output_topic', '/mavros/odometry/out'
        ).value
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
            self.destroy_node()
            raise ValueError('Odometry relay input and output must resolve to different topics')

        # MAVROS requests reliable delivery; the estimator may publish with
        # either reliability setting. Keep only the latest outgoing estimate.
        self.publisher = self.create_publisher(Odometry, output_topic, 1)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self.publisher.publish, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Relaying external odometry: {self.subscription.topic_name} '
            f'-> {self.publisher.topic_name}'
        )


def main():
    rclpy.init()
    node = None
    try:
        node = ExternalOdometryRelay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
