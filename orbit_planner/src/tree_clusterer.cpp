#include "orbit_planner/tree_clusterer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace orbit_planner {

TreeClusterer::TreeClusterer() 
    : min_height_(0.4), max_height_(0.7), cluster_tolerance_(0.5),
      min_cluster_size_(10), voxel_size_(0.1), row_tolerance_(1.0),
      min_row_length_(5.0), min_trees_per_row_(3) {
    kdtree_ = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
}

void TreeClusterer::setParameters(double min_height, double max_height, 
                                 double cluster_tolerance, int min_cluster_size,
                                 double voxel_size, double row_tolerance,
                                 double min_row_length, int min_trees_per_row) {
    min_height_ = min_height;
    max_height_ = max_height;
    cluster_tolerance_ = cluster_tolerance;
    min_cluster_size_ = min_cluster_size;
    voxel_size_ = voxel_size;
    row_tolerance_ = row_tolerance;
    min_row_length_ = min_row_length;
    min_trees_per_row_ = min_trees_per_row;
}

std::vector<TreeCluster> TreeClusterer::detectTrees(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    
    if (!input_cloud || input_cloud->empty()) {
        return {};
    }
    
    // Filter height band
    auto filtered_cloud = filterHeightBand(input_cloud);
    
    // Cluster trees
    auto clusters = clusterTrees(filtered_cloud);
    
    last_tree_clusters_ = clusters;
    return clusters;
}

std::vector<TreeRow> TreeClusterer::detectRows(
    const std::vector<TreeCluster>& tree_clusters) {
    
    std::vector<TreeRow> rows;
    
    if (tree_clusters.empty()) {
        return rows;
    }
    
    // Simple row detection - group clusters by proximity
    std::vector<bool> processed(tree_clusters.size(), false);
    
    for (size_t i = 0; i < tree_clusters.size(); ++i) {
        if (processed[i]) continue;
        
        std::vector<TreeCluster> row_clusters;
        row_clusters.push_back(tree_clusters[i]);
        processed[i] = true;
        
        // Find nearby clusters
        for (size_t j = i + 1; j < tree_clusters.size(); ++j) {
            if (processed[j]) continue;
            
            double distance = calculateDistance(tree_clusters[i].center, tree_clusters[j].center);
            if (distance < row_tolerance_) {
                row_clusters.push_back(tree_clusters[j]);
                processed[j] = true;
            }
        }
        
        // Create row if enough trees
        if (row_clusters.size() >= min_trees_per_row_) {
            TreeRow row;
            row.trees = row_clusters;
            row.tree_count = row_clusters.size();
            row.start_point = calculateRowStart(row);
            row.end_point = calculateRowEnd(row);
            row.orientation = calculateRowOrientation(row);
            row.length = calculateDistance(row.start_point, row.end_point);
            row.width = 2.0; // Default width
            
            if (isValidRow(row)) {
                rows.push_back(row);
            }
        }
    }
    
    last_tree_rows_ = rows;
    return rows;
}

nav_msgs::msg::OccupancyGrid TreeClusterer::createOccupancyGrid(
    const std::vector<TreeCluster>& tree_clusters,
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
    
    grid.data.resize(width * height, 0);
    
    // Mark tree positions as occupied
    for (const auto& cluster : tree_clusters) {
        int x = static_cast<int>((cluster.center.x - origin.x) / resolution);
        int y = static_cast<int>((cluster.center.y - origin.y) / resolution);
        
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Mark tree and surrounding area as occupied
            for (int dx = -2; dx <= 2; ++dx) {
                for (int dy = -2; dy <= 2; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        grid.data[ny * width + nx] = 100;
                    }
                }
            }
        }
    }
    
    return grid;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TreeClusterer::filterHeightBand(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (const auto& point : input_cloud->points) {
        if (isPointInHeightBand(point)) {
            filtered_cloud->points.push_back(point);
        }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;
    
    return filtered_cloud;
}

std::vector<TreeCluster> TreeClusterer::clusterTrees(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
    
    std::vector<TreeCluster> clusters;
    
    if (filtered_cloud->empty()) {
        return clusters;
    }
    
    // Voxel grid filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_.setInputCloud(filtered_cloud);
    voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter_.filter(*voxel_cloud);
    
    // Euclidean clustering
    kdtree_->setInputCloud(voxel_cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extractor_.setClusterTolerance(cluster_tolerance_);
    cluster_extractor_.setMinClusterSize(min_cluster_size_);
    cluster_extractor_.setMaxClusterSize(1000);
    cluster_extractor_.setSearchMethod(kdtree_);
    cluster_extractor_.setInputCloud(voxel_cloud);
    cluster_extractor_.extract(cluster_indices);
    
    // Create tree clusters
    for (const auto& indices : cluster_indices) {
        TreeCluster cluster;
        cluster.point_count = indices.indices.size();
        
        // Calculate centroid
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (int idx : indices.indices) {
            sum_x += voxel_cloud->points[idx].x;
            sum_y += voxel_cloud->points[idx].y;
            sum_z += voxel_cloud->points[idx].z;
        }
        
        cluster.center.x = sum_x / cluster.point_count;
        cluster.center.y = sum_y / cluster.point_count;
        cluster.center.z = sum_z / cluster.point_count;
        
        // Calculate radius and height
        double max_radius = 0;
        double min_z = std::numeric_limits<double>::max();
        double max_z = std::numeric_limits<double>::lowest();
        
        for (int idx : indices.indices) {
            const auto& point = voxel_cloud->points[idx];
            double dist = std::sqrt(std::pow(point.x - cluster.center.x, 2) + 
                                  std::pow(point.y - cluster.center.y, 2));
            max_radius = std::max(max_radius, dist);
            min_z = std::min(min_z, static_cast<double>(point.z));
            max_z = std::max(max_z, static_cast<double>(point.z));
        }
        
        cluster.radius = max_radius;
        cluster.height = max_z - min_z;
        
        clusters.push_back(cluster);
    }
    
    return clusters;
}

bool TreeClusterer::isValidRow(const TreeRow& row) {
    return row.tree_count >= min_trees_per_row_ && row.length >= min_row_length_;
}

double TreeClusterer::calculateRowOrientation(const TreeRow& row) const {
    if (row.trees.size() < 2) return 0.0;
    
    // Calculate orientation using first and last trees
    const auto& first = row.trees.front().center;
    const auto& last = row.trees.back().center;
    
    return std::atan2(last.y - first.y, last.x - first.x);
}

geometry_msgs::msg::Point TreeClusterer::calculateRowStart(const TreeRow& row) const {
    if (row.trees.empty()) return {};
    
    geometry_msgs::msg::Point start = row.trees[0].center;
    for (const auto& tree : row.trees) {
        if (tree.center.x < start.x) {
            start = tree.center;
        }
    }
    return start;
}

geometry_msgs::msg::Point TreeClusterer::calculateRowEnd(const TreeRow& row) const {
    if (row.trees.empty()) return {};
    
    geometry_msgs::msg::Point end = row.trees[0].center;
    for (const auto& tree : row.trees) {
        if (tree.center.x > end.x) {
            end = tree.center;
        }
    }
    return end;
}

double TreeClusterer::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                       const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double TreeClusterer::calculateAngle(const geometry_msgs::msg::Point& p1, 
                                    const geometry_msgs::msg::Point& p2) {
    return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

bool TreeClusterer::isPointInHeightBand(const pcl::PointXYZ& point) {
    return point.z >= min_height_ && point.z <= max_height_;
}

} // namespace orbit_planner
