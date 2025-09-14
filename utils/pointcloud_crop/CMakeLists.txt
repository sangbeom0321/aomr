cmake_minimum_required(VERSION 3.8)
project(pointcloud_crop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(crop_node src/crop_node.cpp)

ament_target_dependencies(crop_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  Eigen3
)

target_link_libraries(crop_node
  ${PCL_LIBRARIES}
)

install(TARGETS crop_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()