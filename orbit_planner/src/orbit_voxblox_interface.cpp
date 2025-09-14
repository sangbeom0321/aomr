#include "orbit_planner/orbit_voxblox_interface.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace orbit_planner
{

OrbitVoxbloxInterface::OrbitVoxbloxInterface(const rclcpp::NodeOptions & options)
: Node("orbit_voxblox_interface", options)
, point_cloud_map_(new pcl::PointCloud<pcl::PointXYZ>)
, map_updated_(false)
, map_min_bounds_(Eigen::Vector3d::Zero())
, map_max_bounds_(Eigen::Vector3d::Zero())
, grid_resolution_(0.1)
, grid_width_(0)
, grid_height_(0)
, grid_depth_(0)
{
  // Declare parameters
  this->declare_parameter("voxel_size", 0.1);
  this->declare_parameter("truncation_distance", 0.2);
  this->declare_parameter("esdf_max_distance", 2.0);
  this->declare_parameter("robot_radius", 0.4);
  this->declare_parameter("update_rate", 2.0);
  this->declare_parameter("grid_resolution", 0.1);
  this->declare_parameter("point_cloud_topic", "/lio_sam/mapping/cloudRegistered");

  // Get parameters
  voxel_size_ = this->get_parameter("voxel_size").as_double();
  truncation_distance_ = this->get_parameter("truncation_distance").as_double();
  esdf_max_distance_ = this->get_parameter("esdf_max_distance").as_double();
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  update_rate_ = this->get_parameter("update_rate").as_double();
  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  std::string point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();

  // Initialize grid parameters
  grid_width_ = static_cast<int>(100.0 / grid_resolution_);  // 100m x 100m default
  grid_height_ = static_cast<int>(100.0 / grid_resolution_);
  grid_depth_ = static_cast<int>(10.0 / grid_resolution_);   // 10m height

  // Initialize 3D grids
  distance_field_.resize(grid_width_);
  occupancy_grid_3d_.resize(grid_width_);
  for (int i = 0; i < grid_width_; ++i) {
    distance_field_[i].resize(grid_height_);
    occupancy_grid_3d_[i].resize(grid_height_);
    for (int j = 0; j < grid_height_; ++j) {
      distance_field_[i][j].resize(grid_depth_, esdf_max_distance_);
      occupancy_grid_3d_[i][j].resize(grid_depth_, 0);  // 0 = unknown
    }
  }

  // Initialize occupancy grid
  occupancy_grid_.header.frame_id = "map";
  occupancy_grid_.info.resolution = grid_resolution_;
  occupancy_grid_.info.width = grid_width_;
  occupancy_grid_.info.height = grid_height_;
  occupancy_grid_.info.origin.position.x = -50.0;  // Center at origin
  occupancy_grid_.info.origin.position.y = -50.0;
  occupancy_grid_.info.origin.position.z = 0.0;
  occupancy_grid_.info.origin.orientation.w = 1.0;
  occupancy_grid_.data.resize(grid_width_ * grid_height_, -1);  // -1 = unknown

  // Initialize map bounds
  map_min_bounds_ = Eigen::Vector3d(-50.0, -50.0, -5.0);
  map_max_bounds_ = Eigen::Vector3d(50.0, 50.0, 5.0);

  // Create subscribers
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic, 10,
    std::bind(&OrbitVoxbloxInterface::pointCloudCallback, this, std::placeholders::_1));

  // Create publishers
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/orbit_planner/occupancy_grid", 10);
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/orbit_planner/map_markers", 10);

  // Create update timer
  update_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
    std::bind(&OrbitVoxbloxInterface::updateMap, this));

  RCLCPP_INFO(this->get_logger(), "OrbitVoxbloxInterface initialized");
}

OrbitVoxbloxInterface::~OrbitVoxbloxInterface()
{
}

void OrbitVoxbloxInterface::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  // Convert point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  convertPointCloud(msg, cloud);

  // Add to map
  *point_cloud_map_ += *cloud;

  // Downsample the map to prevent memory issues
  if (point_cloud_map_->size() > 100000) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(point_cloud_map_);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*point_cloud_map_);
  }

  map_updated_ = true;
}

void OrbitVoxbloxInterface::convertPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::fromROSMsg(*cloud_msg, *cloud);
}

void OrbitVoxbloxInterface::updateMap()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  if (!map_updated_ || point_cloud_map_->empty()) {
    return;
  }

  // Update 3D occupancy grid
  for (const auto & point : point_cloud_map_->points) {
    int x = static_cast<int>((point.x - map_min_bounds_.x()) / grid_resolution_);
    int y = static_cast<int>((point.y - map_min_bounds_.y()) / grid_resolution_);
    int z = static_cast<int>((point.z - map_min_bounds_.z()) / grid_resolution_);

    if (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_ && z >= 0 && z < grid_depth_) {
      occupancy_grid_3d_[x][y][z] = 1;  // 1 = occupied
    }
  }

  // Update 2D occupancy grid (project 3D to 2D)
  for (int x = 0; x < grid_width_; ++x) {
    for (int y = 0; y < grid_height_; ++y) {
      int occupied_count = 0;
      int total_count = 0;
      
      for (int z = 0; z < grid_depth_; ++z) {
        if (occupancy_grid_3d_[x][y][z] == 1) {
          occupied_count++;
        }
        total_count++;
      }
      
      int index = y * grid_width_ + x;
      if (total_count > 0) {
        double occupancy_ratio = static_cast<double>(occupied_count) / total_count;
        if (occupancy_ratio > 0.5) {
          occupancy_grid_.data[index] = 100;  // Occupied
        } else if (occupancy_ratio < 0.1) {
          occupancy_grid_.data[index] = 0;    // Free
        } else {
          occupancy_grid_.data[index] = -1;   // Unknown
        }
      }
    }
  }

  // Update distance field (simplified implementation)
  for (int x = 0; x < grid_width_; ++x) {
    for (int y = 0; y < grid_height_; ++y) {
      for (int z = 0; z < grid_depth_; ++z) {
        if (occupancy_grid_3d_[x][y][z] == 1) {
          distance_field_[x][y][z] = 0.0;  // Inside obstacle
        } else {
          // Find minimum distance to nearest obstacle
          double min_distance = esdf_max_distance_;
          for (int dx = -static_cast<int>(esdf_max_distance_ / grid_resolution_); 
               dx <= static_cast<int>(esdf_max_distance_ / grid_resolution_); ++dx) {
            for (int dy = -static_cast<int>(esdf_max_distance_ / grid_resolution_); 
                 dy <= static_cast<int>(esdf_max_distance_ / grid_resolution_); ++dy) {
              for (int dz = -static_cast<int>(esdf_max_distance_ / grid_resolution_); 
                   dz <= static_cast<int>(esdf_max_distance_ / grid_resolution_); ++dz) {
                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;
                
                if (nx >= 0 && nx < grid_width_ && ny >= 0 && ny < grid_height_ && 
                    nz >= 0 && nz < grid_depth_ && occupancy_grid_3d_[nx][ny][nz] == 1) {
                  double distance = std::sqrt(dx * dx + dy * dy + dz * dz) * grid_resolution_;
                  min_distance = std::min(min_distance, distance);
                }
              }
            }
          }
          distance_field_[x][y][z] = min_distance;
        }
      }
    }
  }

  // Update map bounds
  if (!point_cloud_map_->empty()) {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*point_cloud_map_, min_pt, max_pt);
    
    map_min_bounds_.x() = std::min(map_min_bounds_.x(), min_pt.x - 5.0);
    map_min_bounds_.y() = std::min(map_min_bounds_.y(), min_pt.y - 5.0);
    map_min_bounds_.z() = std::min(map_min_bounds_.z(), min_pt.z - 2.0);
    
    map_max_bounds_.x() = std::max(map_max_bounds_.x(), max_pt.x + 5.0);
    map_max_bounds_.y() = std::max(map_max_bounds_.y(), max_pt.y + 5.0);
    map_max_bounds_.z() = std::max(map_max_bounds_.z(), max_pt.z + 2.0);
  }

  map_updated_ = false;

  // Publish updated data
  publishOccupancyGrid();
  publishVisualizationMarkers();
}

void OrbitVoxbloxInterface::publishOccupancyGrid()
{
  occupancy_grid_.header.stamp = this->now();
  occupancy_grid_pub_->publish(occupancy_grid_);
}

void OrbitVoxbloxInterface::publishVisualizationMarkers()
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Create marker for point cloud
  visualization_msgs::msg::Marker point_cloud_marker;
  point_cloud_marker.header.frame_id = "map";
  point_cloud_marker.header.stamp = this->now();
  point_cloud_marker.ns = "point_cloud";
  point_cloud_marker.id = 0;
  point_cloud_marker.type = visualization_msgs::msg::Marker::POINTS;
  point_cloud_marker.action = visualization_msgs::msg::Marker::ADD;
  point_cloud_marker.scale.x = 0.05;
  point_cloud_marker.scale.y = 0.05;
  point_cloud_marker.color.r = 0.0;
  point_cloud_marker.color.g = 1.0;
  point_cloud_marker.color.b = 0.0;
  point_cloud_marker.color.a = 0.8;

  // Add points to marker
  for (const auto & point : point_cloud_map_->points) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    point_cloud_marker.points.push_back(p);
  }

  markers.markers.push_back(point_cloud_marker);
  markers_pub_->publish(markers);
}

bool OrbitVoxbloxInterface::isFree(const geometry_msgs::msg::Point & point) const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  int x = static_cast<int>((point.x - map_min_bounds_.x()) / grid_resolution_);
  int y = static_cast<int>((point.y - map_min_bounds_.y()) / grid_resolution_);
  int z = static_cast<int>((point.z - map_min_bounds_.z()) / grid_resolution_);

  if (x < 0 || x >= grid_width_ || y < 0 || y >= grid_height_ || z < 0 || z >= grid_depth_) {
    return false;  // Outside map bounds
  }

  return distance_field_[x][y][z] > robot_radius_;
}

double OrbitVoxbloxInterface::getDistance(const geometry_msgs::msg::Point & point) const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  int x = static_cast<int>((point.x - map_min_bounds_.x()) / grid_resolution_);
  int y = static_cast<int>((point.y - map_min_bounds_.y()) / grid_resolution_);
  int z = static_cast<int>((point.z - map_min_bounds_.z()) / grid_resolution_);

  if (x < 0 || x >= grid_width_ || y < 0 || y >= grid_height_ || z < 0 || z >= grid_depth_) {
    return -1.0;  // Outside map bounds
  }

  return distance_field_[x][y][z];
}

nav_msgs::msg::OccupancyGrid OrbitVoxbloxInterface::getOccupancyGrid() const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  return occupancy_grid_;
}

bool OrbitVoxbloxInterface::isMapUpdated() const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  return map_updated_;
}

void OrbitVoxbloxInterface::getMapBounds(Eigen::Vector3d & min_bounds, Eigen::Vector3d & max_bounds) const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  min_bounds = map_min_bounds_;
  max_bounds = map_max_bounds_;
}

void OrbitVoxbloxInterface::clearMap()
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  
  point_cloud_map_->clear();
  
  // Reset 3D grids
  for (int i = 0; i < grid_width_; ++i) {
    for (int j = 0; j < grid_height_; ++j) {
      std::fill(distance_field_[i][j].begin(), distance_field_[i][j].end(), esdf_max_distance_);
      std::fill(occupancy_grid_3d_[i][j].begin(), occupancy_grid_3d_[i][j].end(), 0);
    }
  }
  
  // Reset 2D occupancy grid
  std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);
  
  map_updated_ = false;
}

visualization_msgs::msg::MarkerArray OrbitVoxbloxInterface::getVisualizationMarkers() const
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Create marker for point cloud
  visualization_msgs::msg::Marker point_cloud_marker;
  point_cloud_marker.header.frame_id = "map";
  point_cloud_marker.header.stamp = this->now();
  point_cloud_marker.ns = "point_cloud";
  point_cloud_marker.id = 0;
  point_cloud_marker.type = visualization_msgs::msg::Marker::POINTS;
  point_cloud_marker.action = visualization_msgs::msg::Marker::ADD;
  point_cloud_marker.scale.x = 0.05;
  point_cloud_marker.scale.y = 0.05;
  point_cloud_marker.color.r = 0.0;
  point_cloud_marker.color.g = 1.0;
  point_cloud_marker.color.b = 0.0;
  point_cloud_marker.color.a = 0.8;

  // Add points to marker
  std::lock_guard<std::mutex> lock(map_mutex_);
  for (const auto & point : point_cloud_map_->points) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    point_cloud_marker.points.push_back(p);
  }

  markers.markers.push_back(point_cloud_marker);
  return markers;
}

} // namespace orbit_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(orbit_planner::OrbitVoxbloxInterface)
