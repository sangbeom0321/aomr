/**
 * @file orbit_planner_node.hpp
 * @brief Main planner node for autonomous orchard exploration
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This node implements autonomous exploration for orchard environments using
 * frontier-based exploration with tree row detection and ESDF-based path planning.
 * 
 * Key features:
 * - Tree clustering and row detection
 * - Frontier detection and candidate generation
 * - Orbit anchor generation for systematic monitoring
 * - Receding-horizon scheduling
 * - ESDF-based path planning
 * - RViz integration for area specification
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <opencv2/opencv.hpp>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include "orbit_voxblox_interface.hpp"
#include "tree_clusterer.hpp"
#include "frontier_detector.hpp"
#include "path_planner.hpp"
#include "orbit_anchor_generator.hpp"

namespace orbit_planner {

struct ExplorationState {
    bool is_exploring = false;
    bool area_defined = false;
    geometry_msgs::msg::PolygonStamped exploration_area;
    geometry_msgs::msg::PoseStamped start_pose;
    std::vector<geometry_msgs::msg::Point> visited_goals;
    std::vector<geometry_msgs::msg::Point> current_frontiers;
    std::vector<geometry_msgs::msg::Point> current_orbit_anchors;
    geometry_msgs::msg::PoseStamped current_goal;
    nav_msgs::msg::Path current_path;
};

class OrbitPlannerNode : public rclcpp::Node {
public:
    OrbitPlannerNode();
    ~OrbitPlannerNode();

private:
    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr area_sub_;
    
    // ROS2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontiers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr orbit_anchors_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visited_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    
    // ROS2 Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_exploration_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_exploration_srv_;
    
    // TF2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Core Components
    std::unique_ptr<OrbitVoxbloxInterface> voxblox_interface_;
    std::unique_ptr<TreeClusterer> tree_clusterer_;
    std::unique_ptr<FrontierDetector> frontier_detector_;
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<OrbitAnchorGenerator> orbit_anchor_generator_;
    
    // State Management
    ExplorationState state_;
    std::mutex state_mutex_;
    
    // Threading
    std::thread exploration_thread_;
    std::atomic<bool> should_explore_;
    
    // Parameters
    struct Parameters {
        // Update rates
        double map_update_rate = 2.0;
        double planning_rate = 1.0;
        
        // Robot parameters
        double robot_radius = 0.4;
        double safety_margin = 0.1;
        
        // Exploration parameters
        double max_planning_distance = 50.0;
        double frontier_cluster_min_size = 5.0;
        double frontier_cluster_max_distance = 10.0;
        double goal_tolerance = 1.0;
        
        // Cost weights
        double yaw_change_weight = 0.5;
        double frontier_gain_weight = 1.0;
        double distance_weight = 1.0;
        
        // Tree detection parameters
        double tree_height_min = 0.4;
        double tree_height_max = 0.7;
        double tree_cluster_tolerance = 0.5;
        int tree_min_cluster_size = 10;
        
        // Orbit anchor parameters
        double orbit_radius = 2.0;
        double orbit_spacing = 1.0;
        
        // Path planning parameters
        double path_resolution = 0.1;
        double path_smoothing_factor = 0.5;
    } params_;
    
    // Callbacks
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void areaCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    
    void startExplorationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void stopExplorationCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    // Core Functions
    void explorationLoop();
    void updateMap();
    void generateCandidates();
    void selectNextGoal();
    void planPath();
    void publishVisualization();
    
    // Utility Functions
    bool isPointInPolygon(const geometry_msgs::msg::Point& point, 
                         const geometry_msgs::msg::PolygonStamped& polygon);
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2);
    double calculateYawChange(const geometry_msgs::msg::PoseStamped& from, 
                            const geometry_msgs::msg::PoseStamped& to);
    
    // Visualization
    void publishFrontiers();
    void publishOrbitAnchors();
    void publishVisitedGoals();
    void publishOccupancyGrid();
    
    // Parameter Loading
    void loadParameters();
};

} // namespace orbit_planner