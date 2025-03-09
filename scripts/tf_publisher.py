#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.get_logger().info('TF Publisher Node Started')
        # Broadcaster para TF dinámicos y estáticos
        self.tf_broadcaster = TransformBroadcaster(self)

        # Suscriptor al tópico de TF de Gazebo
        self.subscription = self.create_subscription(
            TFMessage,
            'gazebo_sim/robot/tf',
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        # Publicar TF dinámicos basados en la TF de Gazebo
        for transform in msg.transforms:
            # Change frame IDs
            transform.header.frame_id = "world"
            transform.child_frame_id = "robot_base_link"
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TfPublisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()