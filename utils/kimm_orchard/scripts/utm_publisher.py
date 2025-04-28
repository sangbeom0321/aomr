#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class OdometryProcessor(Node):
    def __init__(self):
        super().__init__('odometry_processor')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10  # depth는 필요에 따라 설정
        )
        self.subscription = self.create_subscription(
            Odometry,
            #'/lio_sam/mapping/odometry',
            '/odom_baselink',
            self.odom_callback,
             qos_profile=qos_profile)
        self.xy_pub = self.create_publisher(Float64MultiArray, '/Local/utm', 10)
        self.yaw_pub = self.create_publisher(Float64, '/Local/heading', 10)

    def odom_callback(self, msg):
        # Extract x, y coordinates
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        xy_msg = Float64MultiArray()
        xy_msg.data = [x, y]
        self.xy_pub.publish(xy_msg)
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        yaw_msg = Float64()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_processor = OdometryProcessor()
    rclpy.spin(odometry_processor)
    odometry_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
