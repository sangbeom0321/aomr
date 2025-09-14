#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseStamped, TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
import tf_transformations
import numpy as np


class BaseLinkToOdometryNode(Node):
    def __init__(self):
        super().__init__('base_link_to_odom_publisher')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, 'odom_baselink', 10)
        self.lidar_odom_publisher = self.create_publisher(Odometry, 'odom_base_footprint', 10)

        # QoS 설정 (lio_sam은 BEST_EFFORT)
        lio_sam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscription
        self.lidar_odom_sub = self.create_subscription(
            Odometry,
            'lio_sam/mapping/odometry',
            self.lidar_publish_odometry,
            lio_sam_qos
        )

        # 타이머로 주기적으로 odometry publish
        self.timer = self.create_timer(0.01, self.publish_odometry)

    def transform_pose(self, pose_stamped: PoseStamped, transform: TransformStamped) -> PoseStamped:
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation
        pose_position = np.array([pos.x, pos.y, pos.z])
        pose_quat = [ori.x, ori.y, ori.z, ori.w]

        pose_transform_matrix = tf_transformations.quaternion_matrix(pose_quat)
        pose_transform_matrix[0:3, 3] = pose_position

        trans = transform.transform.translation
        rot = transform.transform.rotation
        trans_vec = np.array([trans.x, trans.y, trans.z])
        rot_quat = [rot.x, rot.y, rot.z, rot.w]

        transform_matrix = tf_transformations.quaternion_matrix(rot_quat)
        transform_matrix[0:3, 3] = trans_vec

        # Debug 출력 (선택)
        # self.get_logger().info(f'translation : {trans_vec}')
        # self.get_logger().info(f'before : {pose_position}')
        transformed_position = (pose_transform_matrix @ transform_matrix)[0:3, 3]
        # self.get_logger().info(f'after : {transformed_position}')

        new_pose = PoseStamped()
        new_pose.header = transform.header
        new_pose.pose.position.x = transformed_position[0]
        new_pose.pose.position.y = transformed_position[1]
        new_pose.pose.position.z = transformed_position[2]
        new_pose.pose.orientation.x = pose_quat[0]
        new_pose.pose.orientation.y = pose_quat[1]
        new_pose.pose.orientation.z = pose_quat[2]
        new_pose.pose.orientation.w = pose_quat[3]
        return new_pose

    def lidar_publish_odometry(self, msg: Odometry):
        try:
            # ✅ lidar_link → base_footprint 변환을 적용 (올바른 방향)
            tf = self.tf_buffer.lookup_transform(
                'lidar_link',  # to
                'base_footprint',      # from
                rclpy.time.Time()
            )

            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose  # lidar_link pose in odom

            # pose를 base_footprint 기준으로 변환
            transformed_pose = self.transform_pose(pose_stamped, tf)

            # 결과 Odometry 메시지 생성
            transformed_odom = Odometry()
            transformed_odom.header = msg.header
            transformed_odom.header.frame_id = 'odom'
            transformed_odom.child_frame_id = 'base_footprint'
            transformed_odom.pose.pose = transformed_pose.pose
            transformed_odom.twist = msg.twist  # twist 그대로 복사

            self.lidar_odom_publisher.publish(transformed_odom)

        except Exception as e:
            self.get_logger().warn(f'Could not transform odometry: {e}')

    def publish_odometry(self):
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())

            position = transform.transform.translation
            orientation = transform.transform.rotation

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'
            odom_msg.pose.pose.position = Point(x=position.x, y=position.y, z=position.z)
            odom_msg.pose.pose.orientation = Quaternion(
                x=orientation.x,
                y=orientation.y,
                z=orientation.z,
                w=orientation.w
            )

            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0

            self.odom_publisher.publish(odom_msg)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform odom to base_link: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkToOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
