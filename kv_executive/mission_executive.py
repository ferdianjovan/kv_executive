#!/usr/bin/env python3
from rclpy.node import Node
from tutorial_interfaces.msg import Num


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)
        timer_period = 0.5  # second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        msg = Num()
        msg.num1 = self.i
        msg.num2 = self.i + 1
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num1)
        self.i += 1