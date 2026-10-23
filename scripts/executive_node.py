#!/usr/bin/env python3
import faulthandler

from kv_executive.mission_executive import MissionExecutive
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor


if __name__ == '__main__':
    faulthandler.enable(all_threads=True)
    rclpy.init()
    node = MissionExecutive()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_mission()
        executor.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
