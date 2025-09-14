cmake_minimum_required(VERSION 3.8)
project(orbit_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_visualization_tools REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Voxblox integration (assuming Voxblox is available)
# Note: In a real implementation, you would need to properly integrate Voxblox
# For now, we'll create placeholder headers
set(VOXBLOX_INCLUDE_DIRS "")
set(VOXBLOX_LIBRARIES "")

# Create the orbit_voxblox_interface library
add_library(orbit_voxblox_interface
  src/orbit_voxblox_interface.cpp
)

ament_target_dependencies(orbit_voxblox_interface
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
)

target_include_directories(orbit_voxblox_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the orbit_planner_node executable
add_executable(orbit_planner_node
  src/orbit_planner_node.cpp
)

ament_target_dependencies(orbit_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
  std_srvs
)

target_link_libraries(orbit_planner_node
  orbit_voxblox_interface
)

target_include_directories(orbit_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Create the RViz panel plugin
add_library(orbit_panel_plugin
  src/orbit_panel_plugin.cpp
)

ament_target_dependencies(orbit_panel_plugin
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  rviz_common
  rviz_rendering
  rviz_visualization_tools
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(orbit_panel_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Install executables
install(TARGETS
  orbit_planner_node
  orbit_voxblox_interface
  orbit_panel_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install RViz plugin description
install(FILES rviz/orbit_panel_plugin.xml
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  eigen3_cmake_module
  rviz_common
  rviz_rendering
  rviz_visualization_tools
  std_srvs
)

# Export targets
ament_export_targets(has_targets)

# Export include directories
ament_export_include_directories(include)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests
  ament_add_gtest(test_orbit_planner test/test_orbit_planner.cpp)
  target_link_libraries(test_orbit_planner
    orbit_voxblox_interface
    orbit_planner_node
  )
  target_include_directories(test_orbit_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  # Add performance tests
  ament_add_gtest(test_performance test/test_performance.cpp)
  target_link_libraries(test_performance
    orbit_voxblox_interface
    orbit_planner_node
  )
  
  # Add integration tests
  ament_add_gtest(test_integration test/test_integration.cpp)
  target_link_libraries(test_integration
    orbit_voxblox_interface
    orbit_planner_node
  )
endif()

ament_package()
