from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    #     config_file_path_controller = os.path.join(
    #         os.getcwd(), "config", "angle_params.yaml"
    #     )
    #     config_file_path_Trajectory_Gen = os.path.join(
    #         os.getcwd(), "config", "trajectory_params.yaml"
    #     )

    #     config_file_path_task_combination = os.path.join(
    #         os.getcwd(), "config", "task_combination_params.yaml"
    #     )

    return LaunchDescription(
        [
            # Node(
            #     package="pkg_rb10_apple",
            #     executable="Task_Manager",
            #     name="Task_Manager",
            #     output="screen",
            #     parameters=[config_file_path_task_combination],
            # ),
            Node(
                package="pkg_rb10_apple",
                executable="robot_info_publisher",
                name="robot_info_publisher",
                output="screen",
            ),
            Node(
                package="pkg_rb10_apple",
                executable="robot_tcp_control_server",
                name="robot_tcp_control_server",
                output="screen",
            ),
            Node(
                package="pkg_rb10_apple",
                executable="robot_mode_control_server",
                name="robot_mode_control_server",
                output="screen",
            ),
            Node(
                package="pkg_rb10_apple",
                executable="robot_mode_control_client",
                name="robot_mode_control_client",
                output="screen",
            ),
            Node(
                package="pkg_rb10_apple",
                executable="robot_tracking_aruco_server",
                name="robot_tracking_aruco_server",
                output="screen",
            ),
            Node(
                package="pkg_rb10_apple",
                executable="robot_joint_control_server",
                name="robot_joint_control_server",
                output="screen",
            ),
        ]
    )
