cmake_minimum_required(VERSION 3.8)
project(goal_pose_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(interface_rb10_apple REQUIRED)

add_executable(goal_pose_generator_node src/goal_pose_generator_node.cpp)

ament_target_dependencies(goal_pose_generator_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_srvs
  nlohmann_json
  interface_rb10_apple
)

add_executable(rover_pose_logger_node src/rover_pose_logger_node.cpp)

ament_target_dependencies(rover_pose_logger_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_srvs
  nlohmann_json
  interface_rb10_apple
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  rover_pose_logger_node
  goal_pose_generator_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
