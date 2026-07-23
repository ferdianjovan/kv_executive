#!/usr/bin/env python3
import rclpy
from kv_executive.mission_executive import MinimalPublisher


if __name__ == '__main__':
    rclpy.init()
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
