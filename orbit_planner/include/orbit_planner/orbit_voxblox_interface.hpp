#ifndef ORBIT_PLANNER_ORBIT_VOXBLOX_INTERFACE_HPP
#define ORBIT_PLANNER_ORBIT_VOXBLOX_INTERFACE_HPP

#include <memory>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

namespace orbit_planner
{

/**
 * @brief Interface class for Voxblox TSDF/ESDF mapping integration
 * 
 * This class provides a ROS2-compatible interface to Voxblox mapping capabilities.
 * It maintains a TSDF map from point cloud data and generates ESDF for path planning.
 * 
 * Note: This is a simplified implementation. In a real deployment, this would
 * integrate with the actual Voxblox library for TSDF/ESDF generation.
 */
class OrbitVoxbloxInterface : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for ROS2
   */
  explicit OrbitVoxbloxInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~OrbitVoxbloxInterface();

  /**
   * @brief Check if a point is in free space
   * @param point 3D point to check
   * @return true if point is free (safe for robot)
   */
  bool isFree(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Get distance to nearest obstacle at a point
   * @param point 3D point to query
   * @return Distance to nearest obstacle (negative if inside obstacle)
   */
  double getDistance(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Get 2D occupancy grid for visualization
   * @return Occupancy grid message
   */
  nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;

  /**
   * @brief Check if the map has been updated recently
   * @return true if map was updated within the last update cycle
   */
  bool isMapUpdated() const;

  /**
   * @brief Get the current map bounds
   * @param min_bounds Output minimum bounds
   * @param max_bounds Output maximum bounds
   */
  void getMapBounds(Eigen::Vector3d & min_bounds, Eigen::Vector3d & max_bounds) const;

  /**
   * @brief Clear the map
   */
  void clearMap();

  /**
   * @brief Get visualization markers for debugging
   * @return Marker array for RViz visualization
   */
  visualization_msgs::msg::MarkerArray getVisualizationMarkers() const;

private:
  /**
   * @brief Point cloud callback
   * @param msg Point cloud message
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Update the internal map representation
   */
  void updateMap();

  /**
   * @brief Convert point cloud to internal format
   * @param cloud_msg ROS point cloud message
   * @param cloud PCL point cloud output
   */
  void convertPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /**
   * @brief Publish occupancy grid for visualization
   */
  void publishOccupancyGrid();

  /**
   * @brief Publish visualization markers
   */
  void publishVisualizationMarkers();

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Map data
  mutable std::mutex map_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map_;
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  
  // Map parameters
  double voxel_size_;
  double truncation_distance_;
  double esdf_max_distance_;
  double robot_radius_;
  double update_rate_;
  
  // Map state
  bool map_updated_;
  Eigen::Vector3d map_min_bounds_;
  Eigen::Vector3d map_max_bounds_;
  
  // Grid parameters
  double grid_resolution_;
  int grid_width_;
  int grid_height_;
  int grid_depth_;
  
  // Internal map representation (simplified)
  std::vector<std::vector<std::vector<double>>> distance_field_;
  std::vector<std::vector<std::vector<int>>> occupancy_grid_3d_;
};

} // namespace orbit_planner

#endif // ORBIT_PLANNER_ORBIT_VOXBLOX_INTERFACE_HPP
