#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='orbit_planner').find('orbit_planner')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orbit_planner'),
            'config',
            'orbit_planner_params.yaml'
        ]),
        description='Path to config file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('orbit_planner'),
            'rviz',
            'orbit_planner.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    # Launch LIO-SAM (assuming it's available)
    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lio_sam'),
                'launch',
                'run.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # Launch orbit planner node
    orbit_planner_node = Node(
        package='orbit_planner',
        executable='orbit_planner_node',
        name='orbit_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/lio_sam/mapping/cloudRegistered', '/pointcloud_map'),
            ('/lio_sam/odometry', '/robot_odometry')
        ]
    )
    
    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Launch control node (assuming control_4ws is available)
    control_node = Node(
        package='control_4ws',
        executable='control_4ws_node',
        name='control_4ws',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/orbit_planner/trajectory', '/path_to_follow'),
            ('/orbit_planner/goal', '/current_goal')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        rviz_config_arg,
        lio_sam_launch,
        orbit_planner_node,
        rviz_node,
        control_node
    ])