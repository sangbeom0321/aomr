#include "orbit_planner/orbit_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <map>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace orbit_planner
{

OrbitPlannerNode::OrbitPlannerNode(const rclcpp::NodeOptions & options)
: Node("orbit_planner", options)
, exploration_active_(false)
, polygon_mode_active_(false)
, start_point_selected_(false)
, map_frame_("map")
, base_frame_("base_link")
{
  // Declare parameters
  this->declare_parameter("robot_radius", 0.4);
  this->declare_parameter("goal_tolerance", 1.0);
  this->declare_parameter("frontier_cluster_min_size", 5);
  this->declare_parameter("frontier_cluster_max_distance", 2.0);
  this->declare_parameter("yaw_change_weight", 0.5);
  this->declare_parameter("frontier_gain_weight", 1.0);
  this->declare_parameter("distance_weight", 0.3);
  this->declare_parameter("exploration_rate", 1.0);
  this->declare_parameter("max_planning_distance", 50.0);
  this->declare_parameter("visited_radius", 3.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_frame", "base_link");

  // Get parameters
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  frontier_cluster_min_size_ = this->get_parameter("frontier_cluster_min_size").as_int();
  frontier_cluster_max_distance_ = this->get_parameter("frontier_cluster_max_distance").as_double();
  yaw_change_weight_ = this->get_parameter("yaw_change_weight").as_double();
  frontier_gain_weight_ = this->get_parameter("frontier_gain_weight").as_double();
  distance_weight_ = this->get_parameter("distance_weight").as_double();
  exploration_rate_ = this->get_parameter("exploration_rate").as_double();
  max_planning_distance_ = this->get_parameter("max_planning_distance").as_double();
  visited_radius_ = this->get_parameter("visited_radius").as_double();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize Voxblox interface
  voxblox_interface_ = std::make_shared<OrbitVoxbloxInterface>();

  // Create publishers
  trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orbit_planner/trajectory", 10);
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orbit_planner/goal", 10);
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/orbit_planner/markers", 10);

  // Create services
  start_exploration_srv_ = this->create_service<std_srvs::srv::Empty>(
    "/orbit_planner/start_exploration",
    std::bind(&OrbitPlannerNode::startExplorationCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  stop_exploration_srv_ = this->create_service<std_srvs::srv::Empty>(
    "/orbit_planner/stop_exploration",
    std::bind(&OrbitPlannerNode::stopExplorationCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Create exploration timer
  exploration_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / exploration_rate_)),
    std::bind(&OrbitPlannerNode::explorationLoop, this));

  RCLCPP_INFO(this->get_logger(), "OrbitPlannerNode initialized");
}

OrbitPlannerNode::~OrbitPlannerNode()
{
}

void OrbitPlannerNode::initialize()
{
  RCLCPP_INFO(this->get_logger(), "OrbitPlannerNode initialized");
}

void OrbitPlannerNode::startExploration(
  const geometry_msgs::msg::PolygonStamped & exploration_polygon,
  const geometry_msgs::msg::PoseStamped & start_pose)
{
  std::lock_guard<std::mutex> lock(exploration_mutex_);
  
  exploration_polygon_ = exploration_polygon;
  start_pose_ = start_pose;
  exploration_active_ = true;
  
  // Clear visited areas
  visited_positions_.clear();
  visited_cluster_ids_.clear();
  
  RCLCPP_INFO(this->get_logger(), "Exploration started in polygon with %zu points", 
              exploration_polygon.polygon.points.size());
}

void OrbitPlannerNode::stopExploration()
{
  std::lock_guard<std::mutex> lock(exploration_mutex_);
  
  exploration_active_ = false;
  
  // Clear current path and goal
  current_path_.poses.clear();
  current_goal_.pose.position.x = 0.0;
  current_goal_.pose.position.y = 0.0;
  current_goal_.pose.position.z = 0.0;
  
  RCLCPP_INFO(this->get_logger(), "Exploration stopped");
}

bool OrbitPlannerNode::isExplorationActive() const
{
  std::lock_guard<std::mutex> lock(exploration_mutex_);
  return exploration_active_;
}

void OrbitPlannerNode::explorationLoop()
{
  std::lock_guard<std::mutex> lock(exploration_mutex_);
  
  if (!exploration_active_) {
    return;
  }

  // Update robot pose
  if (!updateRobotPose()) {
    RCLCPP_WARN(this->get_logger(), "Failed to update robot pose");
    return;
  }

  // Check if current goal is reached
  if (!current_goal_.pose.position.x == 0.0 && !current_goal_.pose.position.y == 0.0) {
    if (isGoalReached(current_goal_)) {
      RCLCPP_INFO(this->get_logger(), "Goal reached, selecting next goal");
      markAreaAsVisited(current_goal_.pose.position, visited_radius_);
    } else {
      return;  // Continue to current goal
    }
  }

  // Generate frontier candidates
  std::vector<FrontierCandidate> candidates = generateFrontierCandidates();
  
  if (candidates.empty()) {
    RCLCPP_INFO(this->get_logger(), "No more frontier candidates, exploration complete");
    stopExploration();
    return;
  }

  // Cluster frontiers
  candidates = clusterFrontiers(candidates);

  // Select best goal
  FrontierCandidate best_goal = selectBestGoal(candidates);
  
  if (best_goal.visited) {
    RCLCPP_INFO(this->get_logger(), "All frontiers visited, exploration complete");
    stopExploration();
    return;
  }

  // Plan path to goal
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = map_frame_;
  goal_pose.header.stamp = this->now();
  goal_pose.pose.position = best_goal.position;
  goal_pose.pose.orientation.w = 1.0;

  PathResult path_result = planPath(goal_pose);
  
  if (!path_result.success) {
    RCLCPP_WARN(this->get_logger(), "Failed to plan path to goal");
    return;
  }

  // Update current goal and path
  current_goal_ = goal_pose;
  current_path_ = path_result.path;

  // Publish results
  publishTrajectory(current_path_);
  publishGoal(current_goal_);
  publishVisualizationMarkers(candidates, best_goal);

  RCLCPP_INFO(this->get_logger(), "Selected goal at (%.2f, %.2f), path length: %.2f", 
              best_goal.position.x, best_goal.position.y, path_result.path_length);
}

bool OrbitPlannerNode::updateRobotPose()
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero);
    
    current_pose_.header.frame_id = map_frame_;
    current_pose_.header.stamp = this->now();
    current_pose_.pose.position.x = transform.transform.translation.x;
    current_pose_.pose.position.y = transform.transform.translation.y;
    current_pose_.pose.position.z = transform.transform.translation.z;
    current_pose_.pose.orientation = transform.transform.rotation;
    
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", 
                base_frame_.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }
}

std::vector<FrontierCandidate> OrbitPlannerNode::generateFrontierCandidates()
{
  std::vector<FrontierCandidate> candidates;
  
  if (!voxblox_interface_) {
    return candidates;
  }

  // Get occupancy grid
  nav_msgs::msg::OccupancyGrid occupancy_grid = voxblox_interface_->getOccupancyGrid();
  
  if (occupancy_grid.data.empty()) {
    return candidates;
  }

  // Find frontier cells
  for (int x = 1; x < occupancy_grid.info.width - 1; ++x) {
    for (int y = 1; y < occupancy_grid.info.height - 1; ++y) {
      int index = y * occupancy_grid.info.width + x;
      
      // Check if current cell is free
      if (occupancy_grid.data[index] != 0) {
        continue;
      }
      
      // Check if any neighbor is unknown
      bool has_unknown_neighbor = false;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          
          int nx = x + dx;
          int ny = y + dy;
          int nindex = ny * occupancy_grid.info.width + nx;
          
          if (nindex >= 0 && nindex < static_cast<int>(occupancy_grid.data.size()) &&
              occupancy_grid.data[nindex] == -1) {
            has_unknown_neighbor = true;
            break;
          }
        }
        if (has_unknown_neighbor) break;
      }
      
      if (has_unknown_neighbor) {
        // Convert grid coordinates to world coordinates
        geometry_msgs::msg::Point world_point;
        world_point.x = occupancy_grid.info.origin.position.x + x * occupancy_grid.info.resolution;
        world_point.y = occupancy_grid.info.origin.position.y + y * occupancy_grid.info.resolution;
        world_point.z = 0.0;
        
        // Check if point is in exploration area
        if (!isPointInExplorationArea(world_point)) {
          continue;
        }
        
        // Check if point is reachable
        if (!voxblox_interface_->isFree(world_point)) {
          continue;
        }
        
        // Create frontier candidate
        FrontierCandidate candidate;
        candidate.position = world_point;
        candidate.information_gain = calculateInformationGain(candidate);
        candidate.travel_cost = calculateTravelCost(candidate);
        candidate.total_utility = frontier_gain_weight_ * candidate.information_gain - 
                                 distance_weight_ * candidate.travel_cost;
        candidate.visited = false;
        candidate.cluster_id = -1;
        
        candidates.push_back(candidate);
      }
    }
  }
  
  return candidates;
}

std::vector<FrontierCandidate> OrbitPlannerNode::clusterFrontiers(
  const std::vector<FrontierCandidate> & candidates)
{
  std::vector<FrontierCandidate> clustered_candidates = candidates;
  
  if (candidates.empty()) {
    return clustered_candidates;
  }
  
  // Simple clustering based on distance
  int cluster_id = 0;
  std::vector<bool> processed(candidates.size(), false);
  
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (processed[i]) continue;
    
    clustered_candidates[i].cluster_id = cluster_id;
    processed[i] = true;
    
    // Find nearby candidates
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (processed[j]) continue;
      
      double distance = std::sqrt(
        std::pow(candidates[i].position.x - candidates[j].position.x, 2) +
        std::pow(candidates[i].position.y - candidates[j].position.y, 2));
      
      if (distance < frontier_cluster_max_distance_) {
        clustered_candidates[j].cluster_id = cluster_id;
        processed[j] = true;
      }
    }
    
    cluster_id++;
  }
  
  // Filter clusters by minimum size
  std::map<int, int> cluster_sizes;
  for (const auto & candidate : clustered_candidates) {
    cluster_sizes[candidate.cluster_id]++;
  }
  
  std::vector<FrontierCandidate> filtered_candidates;
  for (const auto & candidate : clustered_candidates) {
    if (cluster_sizes[candidate.cluster_id] >= frontier_cluster_min_size_) {
      filtered_candidates.push_back(candidate);
    }
  }
  
  return filtered_candidates;
}

FrontierCandidate OrbitPlannerNode::selectBestGoal(const std::vector<FrontierCandidate> & candidates)
{
  if (candidates.empty()) {
    return FrontierCandidate();
  }
  
  // Find candidate with highest utility
  auto best_candidate = std::max_element(candidates.begin(), candidates.end(),
    [](const FrontierCandidate & a, const FrontierCandidate & b) {
      return a.total_utility < b.total_utility;
    });
  
  return *best_candidate;
}

PathResult OrbitPlannerNode::planPath(const geometry_msgs::msg::PoseStamped & goal)
{
  PathResult result;
  
  if (!voxblox_interface_) {
    return result;
  }
  
  // Simple straight-line path planning (in real implementation, use A* or RRT)
  geometry_msgs::msg::Point start = current_pose_.pose.position;
  geometry_msgs::msg::Point end = goal.pose.position;
  
  // Check if direct path is collision-free
  int num_samples = static_cast<int>(std::sqrt(
    std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2)) / 0.5) + 1;
  
  bool collision_free = true;
  for (int i = 0; i <= num_samples; ++i) {
    double t = static_cast<double>(i) / num_samples;
    geometry_msgs::msg::Point point;
    point.x = start.x + t * (end.x - start.x);
    point.y = start.y + t * (end.y - start.y);
    point.z = start.z + t * (end.z - start.z);
    
    if (!voxblox_interface_->isFree(point)) {
      collision_free = false;
      break;
    }
  }
  
  if (!collision_free) {
    RCLCPP_WARN(this->get_logger(), "Direct path is not collision-free");
    return result;
  }
  
  // Create path
  result.path.header.frame_id = map_frame_;
  result.path.header.stamp = this->now();
  
  for (int i = 0; i <= num_samples; ++i) {
    double t = static_cast<double>(i) / num_samples;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.path.header;
    pose.pose.position.x = start.x + t * (end.x - start.x);
    pose.pose.position.y = start.y + t * (end.y - start.y);
    pose.pose.position.z = start.z + t * (end.z - start.z);
    pose.pose.orientation.w = 1.0;
    
    result.path.poses.push_back(pose);
  }
  
  result.success = true;
  result.path_length = std::sqrt(
    std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2));
  result.clearance = robot_radius_;
  
  return result;
}

bool OrbitPlannerNode::isGoalReached(const geometry_msgs::msg::PoseStamped & goal)
{
  double distance = std::sqrt(
    std::pow(current_pose_.pose.position.x - goal.pose.position.x, 2) +
    std::pow(current_pose_.pose.position.y - goal.pose.position.y, 2));
  
  return distance < goal_tolerance_;
}

void OrbitPlannerNode::markAreaAsVisited(const geometry_msgs::msg::Point & position, double radius)
{
  visited_positions_.push_back(position);
  
  // Mark nearby cluster IDs as visited
  for (const auto & pos : visited_positions_) {
    double distance = std::sqrt(
      std::pow(position.x - pos.x, 2) + std::pow(position.y - pos.y, 2));
    
    if (distance < radius) {
      // In a real implementation, you would mark the cluster ID as visited
      // For now, we'll just store the position
    }
  }
}

bool OrbitPlannerNode::isPointInExplorationArea(const geometry_msgs::msg::Point & point) const
{
  if (exploration_polygon_.polygon.points.size() < 3) {
    return true;  // No polygon defined, explore everywhere
  }
  
  // Simple point-in-polygon test (ray casting algorithm)
  int crossings = 0;
  for (size_t i = 0; i < exploration_polygon_.polygon.points.size(); ++i) {
    size_t j = (i + 1) % exploration_polygon_.polygon.points.size();
    
    const auto & p1 = exploration_polygon_.polygon.points[i];
    const auto & p2 = exploration_polygon_.polygon.points[j];
    
    if (((p1.y <= point.y) && (point.y < p2.y)) || ((p2.y <= point.y) && (point.y < p1.y))) {
      double x_intersect = p1.x + (point.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y);
      if (point.x < x_intersect) {
        crossings++;
      }
    }
  }
  
  return (crossings % 2) == 1;
}

double OrbitPlannerNode::calculateInformationGain(const FrontierCandidate & candidate)
{
  // Simple information gain based on distance to robot
  double distance = std::sqrt(
    std::pow(candidate.position.x - current_pose_.pose.position.x, 2) +
    std::pow(candidate.position.y - current_pose_.pose.position.y, 2));
  
  // Closer frontiers have higher gain (simplified)
  return 1.0 / (1.0 + distance);
}

double OrbitPlannerNode::calculateTravelCost(const FrontierCandidate & candidate)
{
  // Calculate Euclidean distance
  double distance = std::sqrt(
    std::pow(candidate.position.x - current_pose_.pose.position.x, 2) +
    std::pow(candidate.position.y - current_pose_.pose.position.y, 2));
  
  // Add turning cost (simplified)
  double yaw_cost = 0.0;
  if (!current_path_.poses.empty()) {
    // Calculate angle to candidate
    double dx = candidate.position.x - current_pose_.pose.position.x;
    double dy = candidate.position.y - current_pose_.pose.position.y;
    double target_yaw = std::atan2(dy, dx);
    
    // Get current yaw from pose
    tf2::Quaternion q(
      current_pose_.pose.orientation.x,
      current_pose_.pose.orientation.y,
      current_pose_.pose.orientation.z,
      current_pose_.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    double yaw_diff = std::abs(target_yaw - yaw);
    if (yaw_diff > M_PI) {
      yaw_diff = 2 * M_PI - yaw_diff;
    }
    
    yaw_cost = yaw_diff * yaw_change_weight_;
  }
  
  return distance + yaw_cost;
}

void OrbitPlannerNode::publishTrajectory(const nav_msgs::msg::Path & path)
{
  trajectory_pub_->publish(path);
}

void OrbitPlannerNode::publishGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  goal_pub_->publish(goal);
}

void OrbitPlannerNode::publishVisualizationMarkers(
  const std::vector<FrontierCandidate> & candidates,
  const FrontierCandidate & selected_goal)
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Create marker for frontier candidates
  visualization_msgs::msg::Marker frontier_marker;
  frontier_marker.header.frame_id = map_frame_;
  frontier_marker.header.stamp = this->now();
  frontier_marker.ns = "frontiers";
  frontier_marker.id = 0;
  frontier_marker.type = visualization_msgs::msg::Marker::POINTS;
  frontier_marker.action = visualization_msgs::msg::Marker::ADD;
  frontier_marker.scale.x = 0.2;
  frontier_marker.scale.y = 0.2;
  frontier_marker.color.r = 1.0;
  frontier_marker.color.g = 0.0;
  frontier_marker.color.b = 0.0;
  frontier_marker.color.a = 0.8;
  
  for (const auto & candidate : candidates) {
    if (candidate.visited) continue;
    
    frontier_marker.points.push_back(candidate.position);
  }
  
  markers.markers.push_back(frontier_marker);
  
  // Create marker for selected goal
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = map_frame_;
  goal_marker.header.stamp = this->now();
  goal_marker.ns = "selected_goal";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position = selected_goal.position;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 0.5;
  goal_marker.scale.y = 0.5;
  goal_marker.scale.z = 0.5;
  goal_marker.color.r = 0.0;
  goal_marker.color.g = 1.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 1.0;
  
  markers.markers.push_back(goal_marker);
  
  markers_pub_->publish(markers);
}

void OrbitPlannerNode::startExplorationCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;
  
  // In a real implementation, you would get the polygon and start pose from the request
  // For now, we'll use default values
  geometry_msgs::msg::PolygonStamped polygon;
  polygon.header.frame_id = map_frame_;
  polygon.header.stamp = this->now();
  
  // Create a simple square exploration area
  geometry_msgs::msg::Point32 point;
  point.x = -10.0; point.y = -10.0; point.z = 0.0;
  polygon.polygon.points.push_back(point);
  point.x = 10.0; point.y = -10.0; point.z = 0.0;
  polygon.polygon.points.push_back(point);
  point.x = 10.0; point.y = 10.0; point.z = 0.0;
  polygon.polygon.points.push_back(point);
  point.x = -10.0; point.y = 10.0; point.z = 0.0;
  polygon.polygon.points.push_back(point);
  
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = map_frame_;
  start_pose.header.stamp = this->now();
  start_pose.pose.position.x = 0.0;
  start_pose.pose.position.y = 0.0;
  start_pose.pose.position.z = 0.0;
  start_pose.pose.orientation.w = 1.0;
  
  startExploration(polygon, start_pose);
}

void OrbitPlannerNode::stopExplorationCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;
  
  stopExploration();
}

} // namespace orbit_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(orbit_planner::OrbitPlannerNode)
