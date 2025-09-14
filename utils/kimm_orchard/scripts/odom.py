#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer


class BaseLinkToOdometryNode(Node):
    def __init__(self):
        super().__init__('base_link_to_odom_publisher')

        # Create a TF2 buffer and listener to listen to transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for Odometry messages
        self.odom_publisher = self.create_publisher(Odometry, 'odom_baselink', 10)

        # Create a timer to periodically update and publish the Odometry message
        self.timer = self.create_timer(0.01, self.publish_odometry)  # Publish every 100ms

    def publish_odometry(self):
        try:
            # Get the transform from 'odom' to 'base_link'
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            # Extract position and orientation from the transform
            position = transform.transform.translation
            orientation = transform.transform.rotation

            # Create an Odometry message
            odom_msg = Odometry()

            # Fill the header with timestamp and frame info
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'

            # Properly assign position and orientation
            odom_msg.pose.pose.position = Point(x=position.x, y=position.y, z=position.z)
            odom_msg.pose.pose.orientation = Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)

            # Optionally, set velocities (you could compute these if needed)
            odom_msg.twist.twist.linear.x = 0.0  # Linear velocity (x)
            odom_msg.twist.twist.linear.y = 0.0  # Linear velocity (y)
            odom_msg.twist.twist.linear.z = 0.0  # Linear velocity (z)
            odom_msg.twist.twist.angular.x = 0.0  # Angular velocity (x)
            odom_msg.twist.twist.angular.y = 0.0  # Angular velocity (y)
            odom_msg.twist.twist.angular.z = 0.0  # Angular velocity (z)

            # Publish the Odometry message
            self.odom_publisher.publish(odom_msg)


        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform odom to base_link: {ex}')


def main(args=None):
    rclpy.init(args=args)

    # Create and run the node
    node = BaseLinkToOdometryNode()
    rclpy.spin(node)

    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

