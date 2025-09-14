cmake_minimum_required(VERSION 3.8)
project(isaac_ml)

# 디버그 정보 추가
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(2d_map_graph src/2d_map_graph.gui.cpp)
target_link_libraries(2d_map_graph Qt5::Widgets)
ament_target_dependencies(2d_map_graph rclcpp)

add_executable(state_machine src/state_machine.cpp)
ament_target_dependencies(state_machine rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(topology_path 
  src/topology_path.cpp
)

target_include_directories(topology_path PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(topology_path
  rclcpp
  nav_msgs
  geometry_msgs
  yaml-cpp
  OpenCV
  tf2_geometry_msgs
)
target_link_libraries(topology_path
  yaml-cpp
  ${OpenCV_LIBS}
)

add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(path_planner_node rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(local_costmap_node src/local_costmap_node.cpp)
ament_target_dependencies(local_costmap_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  Eigen3)

add_executable(nav_goal_action_server src/nav_goal_action_server.cpp)
ament_target_dependencies(nav_goal_action_server 
  rclcpp
  rclcpp_action
  sensor_msgs 
  nav_msgs
  nav2_msgs 
  std_msgs
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  Eigen3)
  
install(TARGETS
  state_machine
  2d_map_graph
  topology_path
  path_planner_node
  nav_goal_action_server
  local_costmap_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  map
  DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/four_ws_control.py
        scripts/four_ws_control_pos.py
        scripts/keyboard_teleop.py
        scripts/state_machine.py
        scripts/drone_local_path.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()


