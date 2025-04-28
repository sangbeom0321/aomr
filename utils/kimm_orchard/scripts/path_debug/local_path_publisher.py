#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import argparse
from std_msgs.msg import Float64MultiArray

class PathProcessor(Node):
    def __init__(self, topic_name):
        super().__init__('local_path_publisher')
        self.subscription = self.create_subscription(
            Path,
            topic_name,
            self.path_callback,
            10)
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/Local/utm',
            self.pos_callback,
            10)
        
        self.publisher = self.create_publisher(Path, '/Planning/local_path', 10)
        self.publisher_viz = self.create_publisher(Path, '/Planning/local_path_viz', 10)

        self.current_position = PoseStamped()
        self.global_path = Path()
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10Hz
        self.pos_flag =False
        self.utm_y_zero = 4070388.235393337
        self.utm_x_zero = 346638.99532500084

    def path_callback(self, msg):
        offset_path = Path()
        offset_path.header = msg.header

        for pose in msg.poses:
            # 각 포즈의 위치에 offset 적용
            offset_pose = PoseStamped()
            offset_pose.header = pose.header
            offset_pose.pose.position.x = pose.pose.position.x - self.utm_x_zero
            offset_pose.pose.position.y = pose.pose.position.y - self.utm_y_zero
            offset_pose.pose.position.z = pose.pose.position.z
            offset_pose.pose.orientation = pose.pose.orientation

            offset_path.poses.append(offset_pose)

        self.global_path = offset_path
        print(self.global_path)

    def timer_callback(self):
        # 가장 가까운 100개의 앞쪽 점과 30개의 뒤쪽 점 찾기
        if self.pos_flag:
            closest_points = self.find_closest_points(self.global_path, self.current_position, 100, 30)

            # 새로운 Path 메시지 생성 및 퍼블리시
            new_path = Path()
            new_path.header.stamp = rclpy.time.Time().to_msg()
            new_path.header.frame_id = "map"  # 참조할 프레임 ID
            
            if(closest_points is not None):
                print(len(closest_points))

            if closest_points is not None:
                new_path.poses = closest_points
                self.publisher_viz.publish(new_path)

            # 새로운 Path 메시지 생성 및 오프셋 적용
            offset_path = Path()
            offset_path.header.stamp = rclpy.time.Time().to_msg()
            offset_path.header.frame_id = "map"  # 참조할 프레임 ID

            if closest_points is not None:
                for pose in closest_points:
                    # 각 포즈의 위치에 offset 적용
                    offset_pose = PoseStamped()
                    offset_pose.header = pose.header
                    offset_pose.pose.position.x = pose.pose.position.x + self.utm_x_zero
                    offset_pose.pose.position.y = pose.pose.position.y + self.utm_y_zero
                    offset_pose.pose.position.z = pose.pose.position.z
                    offset_pose.pose.orientation = pose.pose.orientation

                    offset_path.poses.append(offset_pose)

                # 오프셋이 적용된 Path 메시지를 다른 토픽에 퍼블리시
                self.publisher.publish(offset_path)

    def pos_callback(self, msg):
        # 타임스탬프와 프레임 ID 설정
        self.pos_flag = True
        self.current_position.header.stamp = rclpy.time.Time().to_msg()
        self.current_position.header.frame_id = "map"  # 참조할 프레임 ID

        # 위치 설정
        self.current_position.pose.position.x = msg.data[0] - self.utm_x_zero
        self.current_position.pose.position.y = msg.data[1] - self.utm_y_zero
        self.current_position.pose.position.z = 0.0 

    def find_closest_points(self, path, current_position, num_points_forward, num_points_backward):
        # 현재 위치에서 각 점까지의 거리를 계산
        distances = []
        for i, pose in enumerate(path.poses):
            dist = self.euclidean_distance(current_position.pose, pose.pose)
            distances.append((dist, i))

        #print(dist)
        #print(distances)
        # 거리에 따라 정렬하여 가장 가까운 점 찾기
        distances.sort(key=lambda x: x[0])
        #print(distances([0][1]))
        if len(distances) > 0:
            closest_point_index = distances[0][1]

            # 선택된 점들의 범위 계산  try:
            start_index = max(closest_point_index - num_points_backward, 0)
            # start_index = max(closest_point_index+2, 0)
            end_index = min(closest_point_index + num_points_forward, len(path.poses))

            # 선택된 범위의 점들을 반환
            selected_points = path.poses[start_index:end_index]
            
            # if len(selected_points) < 3:
            # 	selected_points = path.poses[-1:]

            return selected_points
        
        return None

    def euclidean_distance(self, pose1, pose2):
        # 두 포즈 사이의 유클리드 거리를 계산
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return (dx**2 + dy**2 + dz**2)**0.5


def main(args=None):
    # argparse를 사용하여 명령줄 인자 처리
    parser = argparse.ArgumentParser(description='Path Processor Node')
    # parser.add_argument('topic_name', type=str, help='Name of the path topic to subscribe')
    # args = parser.parse_args()

    # rclpy.init에 전달하기 위한 인자 리스트 생성
    # rclpy_args = [args.topic_name]

    rclpy.init()
    path_processor = PathProcessor("/plan")
    rclpy.spin(path_processor)
    path_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
