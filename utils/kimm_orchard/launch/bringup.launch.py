#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression

from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    utm_pub = Node(
        package='kimm_orchard',
        executable='utm_publisher.py',
        name='utm',
    )

    pcd_yaml_path = join(
        get_package_share_directory('pcd_cal'),
        'config',
        'tree_grid_real.yaml'
    )

    laser_scan = Node(
        package='pcd_cal',
        executable='laser_scan',
        name='laser_scan',
        output='screen'
    )
    
    cloud_crop = Node(
        package='pointcloud_crop',
        executable='crop_node',
        name='crop_node',
        output='screen'
    )
    
    local_path_pub = Node(
        package='kimm_orchard',
        executable='local_path_publisher',
        name='local_path',
    )

    # kimm_orchard_sim mappub 실행
    map_pub = Node(
        package='kimm_orchard',
        executable='mappub.py',
        name='mappub',
    )
    
    empty_map_pub = Node(
        package='kimm_orchard',
        executable='empty_map_publisher.py',
        name='mappub',
    )
    
    state_machine=Node(
            package='kimm_orchard', 
            executable='state_machine',
            name='state_machine'
    )
    
    state_machine_nav2=Node(
            package='kimm_orchard', 
            executable='state_machine_nav2',
            name='state_machine_nav2'
    )

    odom_baselink=Node(
            package='kimm_orchard', 
            executable='odom.py',
            name='odom_baselink'
    )

    return LaunchDescription([
        # Declare launch arguments
        cloud_crop,
        utm_pub,
        laser_scan,
        # map_pub,
        #empty_map_pub,
        # local_path_pub,
        state_machine,
        # state_machine_nav2,
        odom_baselink
    ])
