#include "orbit_planner/orbit_voxblox_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orbit_planner {

OrbitVoxbloxInterface::OrbitVoxbloxInterface() 
    : voxel_size_(0.1), truncation_distance_(0.2), max_weight_(100.0),
      bounds_initialized_(false) {
    min_bounds_.x = min_bounds_.y = min_bounds_.z = std::numeric_limits<double>::max();
    max_bounds_.x = max_bounds_.y = max_bounds_.z = std::numeric_limits<double>::lowest();
}

void OrbitVoxbloxInterface::initialize(double voxel_size, double truncation_distance) {
    voxel_size_ = voxel_size;
    truncation_distance_ = truncation_distance;
}

void OrbitVoxbloxInterface::integratePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const geometry_msgs::msg::PoseStamped& pose) {
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (const auto& point : cloud->points) {
        geometry_msgs::msg::Point world_point;
        world_point.x = point.x;
        world_point.y = point.y;
        world_point.z = point.z;
        
        updateBounds(world_point);
        
        std::string key = pointToKey(world_point);
        auto& voxel = tsdf_map_[key];
        
        // Simple TSDF integration
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        voxel.tsdf_value = std::min(voxel.tsdf_value, distance);
        voxel.weight = std::min(voxel.weight + 1.0, max_weight_);
        voxel.is_occupied = (distance < truncation_distance_);
    }
}

void OrbitVoxbloxInterface::updateESDF() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    computeESDF();
}

double OrbitVoxbloxInterface::getDistance(const geometry_msgs::msg::Point& point) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return interpolateDistance(point);
}

bool OrbitVoxbloxInterface::isFree(const geometry_msgs::msg::Point& point, double radius) {
    return getDistance(point) >= radius;
}

nav_msgs::msg::OccupancyGrid OrbitVoxbloxInterface::generateOccupancyGrid(
    const geometry_msgs::msg::Point& origin,
    double resolution,
    int width,
    int height) {
    
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = rclcpp::Clock().now();
    
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position = origin;
    grid.info.origin.orientation.w = 1.0;
    
    grid.data.resize(width * height, -1);
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            geometry_msgs::msg::Point world_point;
            world_point.x = origin.x + x * resolution;
            world_point.y = origin.y + y * resolution;
            world_point.z = 0.0;
            
            double distance = interpolateDistance(world_point);
            int index = y * width + x;
            
            if (distance < 0.5) {
                grid.data[index] = 100; // Occupied
            } else if (distance < 2.0) {
                grid.data[index] = 0;   // Free
            } else {
                grid.data[index] = -1;  // Unknown
            }
        }
    }
    
    return grid;
}

void OrbitVoxbloxInterface::getMapBounds(geometry_msgs::msg::Point& min_point, 
                                        geometry_msgs::msg::Point& max_point) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    min_point = min_bounds_;
    max_point = max_bounds_;
}

void OrbitVoxbloxInterface::setVoxelSize(double voxel_size) {
    voxel_size_ = voxel_size;
}

void OrbitVoxbloxInterface::setTruncationDistance(double truncation_distance) {
    truncation_distance_ = truncation_distance;
}

void OrbitVoxbloxInterface::setMaxWeight(double max_weight) {
    max_weight_ = max_weight;
}

std::string OrbitVoxbloxInterface::pointToKey(const geometry_msgs::msg::Point& point) {
    int x = static_cast<int>(std::floor(point.x / voxel_size_));
    int y = static_cast<int>(std::floor(point.y / voxel_size_));
    int z = static_cast<int>(std::floor(point.z / voxel_size_));
    return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
}

geometry_msgs::msg::Point OrbitVoxbloxInterface::keyToPoint(const std::string& key) {
    geometry_msgs::msg::Point point;
    // Simple parsing - in real implementation, use proper parsing
    point.x = point.y = point.z = 0.0;
    return point;
}

void OrbitVoxbloxInterface::updateBounds(const geometry_msgs::msg::Point& point) {
    if (!bounds_initialized_) {
        min_bounds_ = max_bounds_ = point;
        bounds_initialized_ = true;
    } else {
        min_bounds_.x = std::min(min_bounds_.x, point.x);
        min_bounds_.y = std::min(min_bounds_.y, point.y);
        min_bounds_.z = std::min(min_bounds_.z, point.z);
        max_bounds_.x = std::max(max_bounds_.x, point.x);
        max_bounds_.y = std::max(max_bounds_.y, point.y);
        max_bounds_.z = std::max(max_bounds_.z, point.z);
    }
}

void OrbitVoxbloxInterface::computeESDF() {
    // Simple ESDF computation - in real implementation, use proper algorithm
    for (auto& [key, voxel] : tsdf_map_) {
        if (voxel.is_occupied) {
            esdf_map_[key] = 0.0;
        } else {
            esdf_map_[key] = voxel.tsdf_value;
        }
    }
}

double OrbitVoxbloxInterface::interpolateDistance(const geometry_msgs::msg::Point& point) {
    std::string key = pointToKey(point);
    
    if (esdf_map_.find(key) != esdf_map_.end()) {
        return esdf_map_[key];
    }
    
    // Simple interpolation - in real implementation, use proper interpolation
    return 5.0; // Default distance for unknown areas
}

} // namespace orbit_planner