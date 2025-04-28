cmake_minimum_required(VERSION 3.8)
project(kimm_orchard)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(state_machine src/state_machine.cpp)
ament_target_dependencies(state_machine rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(state_machine_nav2 src/state_machine_nav2.cpp)
ament_target_dependencies(state_machine_nav2 rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(local_path_publisher src/local_path_publisher.cpp)
ament_target_dependencies(local_path_publisher 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  std_msgs
  tf2_eigen
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  state_machine
  state_machine_nav2
  local_path_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch scripts
    DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
