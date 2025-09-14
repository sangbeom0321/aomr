#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import yaml
from PIL import Image

class EmptyGridMapPublisher(Node):
    def __init__(self):
        super().__init__('empty_grid_map_publisher')
        # 'reliable'와 'transient local' QoS 설정으로 발행자 생성
        qos_profile = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)
        self.publisher_local = self.create_publisher(OccupancyGrid, '/local_costmap/local_costmap/costmap_raw', qos_profile)
        self.publisher_metadata = self.create_publisher(MapMetaData, '/map/metadata', qos_profile)
        self.map_info, self.map_data, self.width, self.height = self.load_map('/root/ros2_ws/src/pcd_cal/maps/map_tree.yaml')
        # self.map_info, self.map_data, self.width, self.height = self.load_map('/root/map/kimm_map/map.yaml')
        
        # 'reliable'와 'transient local' QoS 설정으로 /clock 구독
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile)
        self.current_time = self.get_clock().now()  # 기본 시간 초기화
        self.timer = self.create_timer(1.0, self.publish_empty_map_and_metadata)
    
    def load_map(self, yaml_file):
        with open(yaml_file, 'r') as file:
            map_info = yaml.safe_load(file)

        # PNG 파일 읽기
        image = Image.open(map_info['image'])
        image = image.convert('L')  # 그레이스케일로 변환
        data = np.array(image)

        width, height = image.size
        data = data.flatten()

        # 데이터를 occupancy 값으로 변환
        data = [100 if x < 50 else 0 if x > 200 else -1 for x in data]

        return map_info, data, width, height

    def mirror_data_along_y(self, data, width, height):
        data_np = np.array(data).reshape(height, width)
        mirrored_data = np.flip(data_np, axis=0).flatten()  # y축을 기준으로 데이터 배열 반전
        return mirrored_data.tolist()

    def clock_callback(self, clock_msg):
        # /clock 토픽에서 받은 시간을 저장
        self.current_time = rclpy.time.Time.from_msg(clock_msg.clock)

    def publish_empty_map_and_metadata(self):
        self.mirrored_data = self.mirror_data_along_y(self.map_data, self.width, self.height)
        # Create an empty grid map
        grid_map = OccupancyGrid()
        grid_map.header = Header()
        grid_map.header.stamp = self.current_time.to_msg()  # Use time from /clock
        grid_map.header.frame_id = "map"
        print(self.map_info)
        # Define the map metadata
        grid_map.info.resolution = float(self.map_info['resolution'])
        grid_map.info.width = int(self.width)
        grid_map.info.height = int(self.height)
        grid_map.info.origin.position.x = float(self.map_info['origin'][0])
        grid_map.info.origin.position.y = float(self.map_info['origin'][1])
        grid_map.info.origin.position.z = float(self.map_info['origin'][2])
        grid_map.info.origin.orientation.w = 1.0
        grid_map.data = self.map_data

        # Publish the empty grid map with walls on the edges
        self.publisher_.publish(grid_map)
        # self.publisher_local.publish(grid_map)
        
        # Publish the map metadata separately
        metadata = grid_map.info
        metadata.map_load_time = self.current_time.to_msg()  # Optional: set map load time
        self.publisher_metadata.publish(metadata)

        self.get_logger().info('Published an empty grid map and metadata with /clock timestamp')


def main(args=None):
    rclpy.init(args=args)
    empty_grid_map_publisher = EmptyGridMapPublisher()
    rclpy.spin(empty_grid_map_publisher)
    empty_grid_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
