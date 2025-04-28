#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

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
        
        # 'reliable'와 'transient local' QoS 설정으로 /clock 구독
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, qos_profile)
        self.current_time = self.get_clock().now()  # 기본 시간 초기화
        self.create_map()
        # self.timer = self.create_timer(1.0, self.publish_empty_map_and_metadata)
        self.publish_empty_map_and_metadata()


    def clock_callback(self, clock_msg):
        # /clock 토픽에서 받은 시간을 저장
        self.current_time = rclpy.time.Time.from_msg(clock_msg.clock)

    def create_map(self):
                # Create an empty grid map
        grid_map = OccupancyGrid()
        grid_map.header = Header()
        grid_map.header.stamp = rclpy.time.Time().to_msg()
        grid_map.header.frame_id = "map"

        # Define the map metadata
        grid_map.info.resolution = 0.05  # each cell is 5cm x 5cm
        grid_map.info.width = 6000  # map is 1000 cells wide
        grid_map.info.height = 6000 # map is 1000 cells tall
        grid_map.info.origin = Pose()
        grid_map.info.origin.position.x = -100.0
        grid_map.info.origin.position.y = -100.0
        grid_map.info.origin.position.z = 0.0
        grid_map.info.origin.orientation = Quaternion()
        grid_map.info.origin.orientation.w = 1.0  # No rotation

        # Initialize all cells in the grid map to -1 (unknown)
        grid_map.data = [-1] * (grid_map.info.width * grid_map.info.height)

        # Add walls to the edges of the map
        for i in range(grid_map.info.width):
            # Top edge
            grid_map.data[i] = 100
            # Bottom edge
            grid_map.data[i + grid_map.info.width * (grid_map.info.height - 1)] = 100
        for j in range(grid_map.info.height):
            # Left edge
            grid_map.data[j * grid_map.info.width] = 100
            # Right edge
            grid_map.data[(j+1) * grid_map.info.width - 1] = 100
        self.grid_map = grid_map

    def publish_empty_map_and_metadata(self):


        # Publish the empty grid map with walls on the edges
        self.publisher_.publish(self.grid_map)
        # self.publisher_local.publish(grid_map)
        
        # Publish the map metadata separately
        metadata = self.grid_map.info
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
