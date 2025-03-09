#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class FrameRemapper(Node):
    def __init__(self):
        super().__init__('frame_manager')
        self.get_logger().info('Frame Manager Node Started')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribing to sensor topics from Gazebo
        self.lidar_sub = self.create_subscription(PointCloud2, "/gazebo_sim/lidar/points", self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/gazebo_sim/robot/imu", self.imu_callback, 10)

        # Publishers for modified messages with updated frame_ids
        self.lidar_pub = self.create_publisher(PointCloud2, "/myrobot/lidar/points", qos_profile)
        self.imu_pub = self.create_publisher(Imu, "/myrobot/imu", qos_profile)
        
        self.info_logged_lidar = False
        self.info_logged_imu = False

    def lidar_callback(self, msg):
        """ Modify frame_id for lidar sensor """
        msg.header.frame_id = "lidar"  # Change frame_id
        self.lidar_pub.publish(msg)
        if not self.info_logged_lidar:
            self.get_logger().info("Republished lidar with corrected frame_id")
            self.info_logged_lidar = True
    
    def imu_callback(self, msg):
        """ Modify frame_id for IMU sensor """
        msg.header.frame_id = "imu"
        self.imu_pub.publish(msg)
        if not self.info_logged_imu:
            self.get_logger().info("Republished imu with corrected frame_id")
            self.info_logged_imu = True

def main(args=None):
    rclpy.init(args=args)
    node = FrameRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
