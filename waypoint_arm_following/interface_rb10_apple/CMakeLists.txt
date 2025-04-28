cmake_minimum_required(VERSION 3.8)
project(interface_rb10_apple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/RobotJointPosition.msg"
  "msg/RobotTcpPose.msg"
  "msg/ArucoPose.msg"
)

set(srv_files
  "srv/RobotConnectControl.srv"
)

set(action_files
  "action/RobotTrackingControl.action"
  "action/RobotJointControl.action"
  "action/RobotModeControl.action"
  "action/RobotTcpControl.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()
