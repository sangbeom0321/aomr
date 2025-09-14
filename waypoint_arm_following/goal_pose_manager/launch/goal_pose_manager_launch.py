from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rover_pose_logger = Node(
        package='goal_pose_manager',
        executable='rover_pose_logger_node',
        name='rover_pose_logger',
        output='screen',  # Output logs to the screen
        parameters=[{
        }]
    )

    rover_pose_loader = Node(
        package='goal_pose_manager',
        executable='rover_pose_loader_node',
        name='rover_pose_loader',
        output='screen',  # Output logs to the screen
        parameters=[{
        }]
    )

    return LaunchDescription([
        rover_pose_logger,
        rover_pose_loader
        ])

