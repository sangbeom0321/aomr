ros2 action send_goal /robot_tcp_control interface_rb10_apple/action/RobotTcpControl "{x: 0.0, y: 30.0, z: 0.0, rx: 0.0, ry: 0.0, rz: 0.0}"
ros2 action send_goal /robot_mode_control interface_rb10_apple/action/RobotModeControl "mode: 4"

mode: 0 ~ 4

ros2 action send_goal /robot_joint_control interface_rb10_apple/action/RobotJointControl "{j0: 89.97, j1: -8.31, j2: -125.52, j3: -226.17, j4: -89.89, j5: 0.0}"
