#include "orbit_planner/orbit_planner_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace orbit_planner {

OrbitPlannerNode::OrbitPlannerNode() 
    : Node("orbit_planner_node"),
      should_explore_(false) {
    
    // Load parameters
    loadParameters();
    
    // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize core components
    voxblox_interface_ = std::make_unique<OrbitVoxbloxInterface>();
    tree_clusterer_ = std::make_unique<TreeClusterer>();
    frontier_detector_ = std::make_unique<FrontierDetector>();
    path_planner_ = std::make_unique<PathPlanner>();
    orbit_anchor_generator_ = std::make_unique<OrbitAnchorGenerator>();
    
    // Initialize voxblox interface
    voxblox_interface_->initialize(params_.robot_radius, params_.safety_margin);
    
    // Set parameters for components
    tree_clusterer_->setParameters(
        params_.tree_height_min, params_.tree_height_max,
        params_.tree_cluster_tolerance, params_.tree_min_cluster_size,
        0.1, 1.0, 5.0, 3);
    
    frontier_detector_->setParameters(
        params_.robot_radius, static_cast<int>(params_.frontier_cluster_min_size),
        params_.max_planning_distance, params_.frontier_gain_weight);
    
    path_planner_->setParameters(
        params_.robot_radius, params_.safety_margin,
        params_.path_resolution, params_.max_planning_distance);
    
    orbit_anchor_generator_->setParameters(
        params_.orbit_radius, params_.orbit_spacing,
        5.0, params_.max_planning_distance);
    
    // Create subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio_sam/mapping/cloudRegistered", 10,
        std::bind(&OrbitPlannerNode::pointCloudCallback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/lio_sam/odometry", 10,
        std::bind(&OrbitPlannerNode::poseCallback, this, std::placeholders::_1));
    
    area_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/orbit_planner/exploration_area", 10,
        std::bind(&OrbitPlannerNode::areaCallback, this, std::placeholders::_1));

  // Create publishers
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/orbit_planner/trajectory", 10);
    
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/orbit_planner/goal", 10);
    
    frontiers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/frontiers", 10);
    
    orbit_anchors_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/orbit_anchors", 10);
    
    visited_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/visited", 10);
    
    occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/orbit_planner/occupancy", 10);

  // Create services
  start_exploration_srv_ = this->create_service<std_srvs::srv::Empty>(
    "/orbit_planner/start_exploration",
        std::bind(&OrbitPlannerNode::startExplorationCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
  
  stop_exploration_srv_ = this->create_service<std_srvs::srv::Empty>(
    "/orbit_planner/stop_exploration",
        std::bind(&OrbitPlannerNode::stopExplorationCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    // Start exploration thread
    should_explore_ = true;
    exploration_thread_ = std::thread(&OrbitPlannerNode::explorationLoop, this);
    
    RCLCPP_INFO(this->get_logger(), "Orbit Planner Node initialized");
}

OrbitPlannerNode::~OrbitPlannerNode() {
    should_explore_ = false;
    if (exploration_thread_.joinable()) {
        exploration_thread_.join();
    }
}

void OrbitPlannerNode::loadParameters() {
    // Update rates
    this->declare_parameter("map_update_rate", params_.map_update_rate);
    this->declare_parameter("planning_rate", params_.planning_rate);
    
    // Robot parameters
    this->declare_parameter("robot_radius", params_.robot_radius);
    this->declare_parameter("safety_margin", params_.safety_margin);
    
    // Exploration parameters
    this->declare_parameter("max_planning_distance", params_.max_planning_distance);
    this->declare_parameter("frontier_cluster_min_size", params_.frontier_cluster_min_size);
    this->declare_parameter("frontier_cluster_max_distance", params_.frontier_cluster_max_distance);
    this->declare_parameter("goal_tolerance", params_.goal_tolerance);
    
    // Cost weights
    this->declare_parameter("yaw_change_weight", params_.yaw_change_weight);
    this->declare_parameter("frontier_gain_weight", params_.frontier_gain_weight);
    this->declare_parameter("distance_weight", params_.distance_weight);
    
    // Tree detection parameters
    this->declare_parameter("tree_height_min", params_.tree_height_min);
    this->declare_parameter("tree_height_max", params_.tree_height_max);
    this->declare_parameter("tree_cluster_tolerance", params_.tree_cluster_tolerance);
    this->declare_parameter("tree_min_cluster_size", params_.tree_min_cluster_size);
    
    // Orbit anchor parameters
    this->declare_parameter("orbit_radius", params_.orbit_radius);
    this->declare_parameter("orbit_spacing", params_.orbit_spacing);
    
    // Path planning parameters
    this->declare_parameter("path_resolution", params_.path_resolution);
    this->declare_parameter("path_smoothing_factor", params_.path_smoothing_factor);
    
    // Load values
    params_.map_update_rate = this->get_parameter("map_update_rate").as_double();
    params_.planning_rate = this->get_parameter("planning_rate").as_double();
    params_.robot_radius = this->get_parameter("robot_radius").as_double();
    params_.safety_margin = this->get_parameter("safety_margin").as_double();
    params_.max_planning_distance = this->get_parameter("max_planning_distance").as_double();
    params_.frontier_cluster_min_size = this->get_parameter("frontier_cluster_min_size").as_double();
    params_.frontier_cluster_max_distance = this->get_parameter("frontier_cluster_max_distance").as_double();
    params_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    params_.yaw_change_weight = this->get_parameter("yaw_change_weight").as_double();
    params_.frontier_gain_weight = this->get_parameter("frontier_gain_weight").as_double();
    params_.distance_weight = this->get_parameter("distance_weight").as_double();
    params_.tree_height_min = this->get_parameter("tree_height_min").as_double();
    params_.tree_height_max = this->get_parameter("tree_height_max").as_double();
    params_.tree_cluster_tolerance = this->get_parameter("tree_cluster_tolerance").as_double();
    params_.tree_min_cluster_size = this->get_parameter("tree_min_cluster_size").as_int();
    params_.orbit_radius = this->get_parameter("orbit_radius").as_double();
    params_.orbit_spacing = this->get_parameter("orbit_spacing").as_double();
    params_.path_resolution = this->get_parameter("path_resolution").as_double();
    params_.path_smoothing_factor = this->get_parameter("path_smoothing_factor").as_double();
}

void OrbitPlannerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // Get robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_link", msg->header.stamp);
        robot_pose.header = msg->header;
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    return;
  }

    // Integrate point cloud
    voxblox_interface_->integratePointCloud(cloud, robot_pose);
}

void OrbitPlannerNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Update robot pose in state if needed
}

void OrbitPlannerNode::areaCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.exploration_area = *msg;
    state_.area_defined = true;
    RCLCPP_INFO(this->get_logger(), "Exploration area received");
}

void OrbitPlannerNode::startExplorationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_.area_defined) {
        RCLCPP_WARN(this->get_logger(), "Exploration area not defined");
    return;
  }

    state_.is_exploring = true;
    RCLCPP_INFO(this->get_logger(), "Exploration started");
}

void OrbitPlannerNode::stopExplorationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.is_exploring = false;
    RCLCPP_INFO(this->get_logger(), "Exploration stopped");
}

void OrbitPlannerNode::explorationLoop() {
    rclcpp::Rate rate(params_.planning_rate);
    
    while (rclcpp::ok() && should_explore_) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!state_.is_exploring || !state_.area_defined) {
                rate.sleep();
                continue;
            }
        }
        
        // Update map
        updateMap();
        
        // Generate candidates
        generateCandidates();
        
        // Select next goal
        selectNextGoal();
        
        // Plan path
        planPath();
        
        // Publish visualization
        publishVisualization();
        
        rate.sleep();
    }
}

void OrbitPlannerNode::updateMap() {
    // Update ESDF
    voxblox_interface_->updateESDF();
    
    // Generate occupancy grid
    geometry_msgs::msg::Point origin;
    origin.x = -50.0;
    origin.y = -50.0;
    origin.z = 0.0;
    
    auto occupancy_grid = voxblox_interface_->generateOccupancyGrid(origin, 0.1, 1000, 1000);
    occupancy_pub_->publish(occupancy_grid);
}

void OrbitPlannerNode::generateCandidates() {
    // Get current robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
        robot_pose.header.frame_id = "map";
        robot_pose.header.stamp = this->now();
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
        return;
    }
    
    // Generate occupancy grid for frontier detection
    geometry_msgs::msg::Point origin;
    origin.x = -50.0;
    origin.y = -50.0;
    origin.z = 0.0;
    
    auto occupancy_grid = voxblox_interface_->generateOccupancyGrid(origin, 0.1, 1000, 1000);
    
    // Detect frontiers
    auto frontiers = frontier_detector_->detectFrontiers(occupancy_grid, robot_pose);
    
    // Filter frontiers within exploration area
    std::vector<Frontier> valid_frontiers;
    for (const auto& frontier : frontiers) {
        if (isPointInPolygon(frontier.center, state_.exploration_area)) {
            valid_frontiers.push_back(frontier);
        }
    }
    
    // Update state
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.current_frontiers.clear();
    for (const auto& frontier : valid_frontiers) {
        state_.current_frontiers.push_back(frontier.center);
    }
}

void OrbitPlannerNode::selectNextGoal() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (state_.current_frontiers.empty()) {
        RCLCPP_INFO(this->get_logger(), "No more frontiers to explore");
        state_.is_exploring = false;
        return;
    }
    
    // Simple selection: choose closest frontier
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
        robot_pose.header.frame_id = "map";
        robot_pose.header.stamp = this->now();
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
        return;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point best_goal;
    
    for (const auto& frontier : state_.current_frontiers) {
        double distance = calculateDistance(robot_pose.pose.position, frontier);
        if (distance < min_distance) {
            min_distance = distance;
            best_goal = frontier;
        }
    }
    
    // Create goal pose
    state_.current_goal.header.frame_id = "map";
    state_.current_goal.header.stamp = this->now();
    state_.current_goal.pose.position = best_goal;
    state_.current_goal.pose.orientation.w = 1.0;
    
    // Mark as visited
    state_.visited_goals.push_back(best_goal);
}

void OrbitPlannerNode::planPath() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (state_.current_goal.pose.position.x == 0.0 && 
        state_.current_goal.pose.position.y == 0.0) {
        return;
    }
    
    // Get robot pose
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
        robot_pose.header.frame_id = "map";
        robot_pose.header.stamp = this->now();
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
        return;
    }
    
    // Generate occupancy grid for path planning
    geometry_msgs::msg::Point origin;
    origin.x = -50.0;
    origin.y = -50.0;
    origin.z = 0.0;
    
    auto occupancy_grid = voxblox_interface_->generateOccupancyGrid(origin, 0.1, 1000, 1000);
    
    // Plan path
    auto path = path_planner_->planPath(robot_pose, state_.current_goal, occupancy_grid);
    
    if (!path.poses.empty()) {
        state_.current_path = path;
        trajectory_pub_->publish(path);
        goal_pub_->publish(state_.current_goal);
    }
}

void OrbitPlannerNode::publishVisualization() {
    publishFrontiers();
    publishVisitedGoals();
}

void OrbitPlannerNode::publishFrontiers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    for (size_t i = 0; i < state_.current_frontiers.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "frontiers";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = state_.current_frontiers[i];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker_array.markers.push_back(marker);
    }
    
    frontiers_pub_->publish(marker_array);
}

void OrbitPlannerNode::publishVisitedGoals() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    for (size_t i = 0; i < state_.visited_goals.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "visited";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = state_.visited_goals[i];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.6;
        
        marker_array.markers.push_back(marker);
    }
    
    visited_pub_->publish(marker_array);
}

bool OrbitPlannerNode::isPointInPolygon(const geometry_msgs::msg::Point& point, 
                                       const geometry_msgs::msg::PolygonStamped& polygon) {
    // Simple point-in-polygon test using ray casting
  int crossings = 0;
    const auto& points = polygon.polygon.points;
    
    for (size_t i = 0; i < points.size(); ++i) {
        size_t j = (i + 1) % points.size();
        
        if (((points[i].y <= point.y) && (point.y < points[j].y)) ||
            ((points[j].y <= point.y) && (point.y < points[i].y))) {
            
            double x = points[i].x + (point.y - points[i].y) / (points[j].y - points[i].y) * 
                      (points[j].x - points[i].x);
            
            if (point.x < x) {
        crossings++;
      }
    }
  }
  
  return (crossings % 2) == 1;
}

double OrbitPlannerNode::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                          const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double OrbitPlannerNode::calculateYawChange(const geometry_msgs::msg::PoseStamped& from, 
                                           const geometry_msgs::msg::PoseStamped& to) {
    // Calculate yaw change between two poses
    double yaw_from = std::atan2(2.0 * (from.pose.orientation.w * from.pose.orientation.z + 
                                       from.pose.orientation.x * from.pose.orientation.y),
                                1.0 - 2.0 * (from.pose.orientation.y * from.pose.orientation.y + 
                                           from.pose.orientation.z * from.pose.orientation.z));
    
    double yaw_to = std::atan2(2.0 * (to.pose.orientation.w * to.pose.orientation.z + 
                                     to.pose.orientation.x * to.pose.orientation.y),
                              1.0 - 2.0 * (to.pose.orientation.y * to.pose.orientation.y + 
                                         to.pose.orientation.z * to.pose.orientation.z));
    
    double yaw_diff = yaw_to - yaw_from;
    
    // Normalize to [-π, π]
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    
    return std::abs(yaw_diff);
}

} // namespace orbit_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<orbit_planner::OrbitPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
