#!/usr/bin/env python3

__author__ = 'Kevin Medrano Ayala'
__contact__ = 'kevin.ejem18@gmail.com'

import rclpy
from rclpy.node import Node

class myrobot_test(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('Hello from python and c++ package')

def main(args=None):
    rclpy.init(args=args)
    node = myrobot_test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()