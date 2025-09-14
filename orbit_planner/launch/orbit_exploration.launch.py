#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    pkg_orbit_planner = get_package_share_directory('orbit_planner')
    
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orbit_planner'),
            'config',
            'orbit_planner_params.yaml'
        ]),
        description='Path to the config file'
    )
    
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orbit_planner'),
            'rviz',
            'orbit_planner.rviz'
        ]),
        description='Path to the RViz config file'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    # Voxblox Interface Node
    voxblox_interface_node = Node(
        package='orbit_planner',
        executable='orbit_voxblox_interface',
        name='orbit_voxblox_interface',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/lio_sam/mapping/cloudRegistered', '/pointcloud_map'),
        ]
    )
    
    # Orbit Planner Node
    planner_node = Node(
        package='orbit_planner',
        executable='orbit_planner_node',
        name='orbit_planner',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/orbit_planner/trajectory', '/trajectory'),
            ('/orbit_planner/goal', '/exploration_goal'),
        ]
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # Static Transform Publisher (for testing)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_rviz_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_file_arg)
    
    # Add nodes
    ld.add_action(voxblox_interface_node)
    ld.add_action(planner_node)
    ld.add_action(static_tf_publisher)
    ld.add_action(rviz_node)
    
    return ld
