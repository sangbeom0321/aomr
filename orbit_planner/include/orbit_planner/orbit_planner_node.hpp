#ifndef ORBIT_PLANNER_ORBIT_PLANNER_NODE_HPP
#define ORBIT_PLANNER_ORBIT_PLANNER_NODE_HPP

#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "orbit_voxblox_interface.hpp"

namespace orbit_planner
{

/**
 * @brief Structure representing a frontier candidate point
 */
struct FrontierCandidate
{
  geometry_msgs::msg::Point position;
  double information_gain;
  double travel_cost;
  double total_utility;
  bool visited;
  int cluster_id;
  
  FrontierCandidate()
    : information_gain(0.0)
    , travel_cost(0.0)
    , total_utility(0.0)
    , visited(false)
    , cluster_id(-1)
  {}
};

/**
 * @brief Structure representing a path planning result
 */
struct PathResult
{
  nav_msgs::msg::Path path;
  bool success;
  double path_length;
  double clearance;
  
  PathResult()
    : success(false)
    , path_length(0.0)
    , clearance(0.0)
  {}
};

/**
 * @brief Main exploration planner node
 * 
 * This node implements the core exploration logic including:
 * - Frontier detection and candidate generation
 * - Goal selection and scheduling
 * - Path planning using ESDF
 * - Integration with mapping and control systems
 */
class OrbitPlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options for ROS2
   */
  explicit OrbitPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~OrbitPlannerNode();

  /**
   * @brief Initialize the planner
   */
  void initialize();

  /**
   * @brief Start exploration in the specified region
   * @param exploration_polygon Polygon defining exploration area
   * @param start_pose Starting pose for exploration
   */
  void startExploration(
    const geometry_msgs::msg::PolygonStamped & exploration_polygon,
    const geometry_msgs::msg::PoseStamped & start_pose);

  /**
   * @brief Stop exploration
   */
  void stopExploration();

  /**
   * @brief Check if exploration is active
   * @return true if exploration is currently running
   */
  bool isExplorationActive() const;

private:
  /**
   * @brief Main exploration loop
   */
  void explorationLoop();

  /**
   * @brief Update robot pose from TF
   * @return true if pose was successfully updated
   */
  bool updateRobotPose();

  /**
   * @brief Generate frontier candidates
   * @return Vector of frontier candidates
   */
  std::vector<FrontierCandidate> generateFrontierCandidates();

  /**
   * @brief Cluster frontier points
   * @param candidates Input frontier candidates
   * @return Clustered candidates with cluster IDs
   */
  std::vector<FrontierCandidate> clusterFrontiers(
    const std::vector<FrontierCandidate> & candidates);

  /**
   * @brief Select the best goal from candidates
   * @param candidates Vector of frontier candidates
   * @return Selected goal candidate
   */
  FrontierCandidate selectBestGoal(const std::vector<FrontierCandidate> & candidates);

  /**
   * @brief Plan path to goal
   * @param goal Goal pose
   * @return Path planning result
   */
  PathResult planPath(const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Check if goal is reached
   * @param goal Goal pose
   * @return true if robot is close enough to goal
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Mark area as visited
   * @param position Position to mark as visited
   * @param radius Radius around position to mark
   */
  void markAreaAsVisited(const geometry_msgs::msg::Point & position, double radius);

  /**
   * @brief Check if point is within exploration polygon
   * @param point Point to check
   * @return true if point is within exploration area
   */
  bool isPointInExplorationArea(const geometry_msgs::msg::Point & point) const;

  /**
   * @brief Calculate information gain for a candidate
   * @param candidate Frontier candidate
   * @return Information gain value
   */
  double calculateInformationGain(const FrontierCandidate & candidate);

  /**
   * @brief Calculate travel cost to a candidate
   * @param candidate Frontier candidate
   * @return Travel cost value
   */
  double calculateTravelCost(const FrontierCandidate & candidate);

  /**
   * @brief Publish current trajectory
   * @param path Path to publish
   */
  void publishTrajectory(const nav_msgs::msg::Path & path);

  /**
   * @brief Publish current goal
   * @param goal Goal pose to publish
   */
  void publishGoal(const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Publish visualization markers
   * @param candidates Frontier candidates to visualize
   * @param selected_goal Currently selected goal
   */
  void publishVisualizationMarkers(
    const std::vector<FrontierCandidate> & candidates,
    const FrontierCandidate & selected_goal);

  /**
   * @brief Service callback for starting exploration
   * @param request Service request
   * @param response Service response
   */
  void startExplorationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /**
   * @brief Service callback for stopping exploration
   * @param request Service request
   * @param response Service response
   */
  void stopExplorationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // ROS2 components
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_exploration_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_exploration_srv_;
  rclcpp::TimerBase::SharedPtr exploration_timer_;

  // TF components
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Voxblox interface
  std::shared_ptr<OrbitVoxbloxInterface> voxblox_interface_;

  // Exploration state
  mutable std::mutex exploration_mutex_;
  bool exploration_active_;
  geometry_msgs::msg::PolygonStamped exploration_polygon_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped current_goal_;
  nav_msgs::msg::Path current_path_;

  // Visited areas tracking
  std::vector<geometry_msgs::msg::Point> visited_positions_;
  std::unordered_set<int> visited_cluster_ids_;

  // Planner parameters
  double robot_radius_;
  double goal_tolerance_;
  double frontier_cluster_min_size_;
  double frontier_cluster_max_distance_;
  double yaw_change_weight_;
  double frontier_gain_weight_;
  double distance_weight_;
  double exploration_rate_;
  double max_planning_distance_;
  double visited_radius_;

  // Map frame
  std::string map_frame_;
  std::string base_frame_;
  
  // Performance optimization
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread planning_thread_;
  std::atomic<bool> planning_active_;
  std::chrono::steady_clock::time_point last_planning_time_;
  
  // Adaptive update rates
  double base_exploration_rate_;
  double adaptive_exploration_rate_;
  int consecutive_failed_plans_;
  double planning_time_threshold_;
  
  // Caching for performance
  std::unordered_map<std::string, std::vector<FrontierCandidate>> frontier_cache_;
  std::chrono::steady_clock::time_point last_frontier_update_;
  double frontier_cache_duration_;
};

} // namespace orbit_planner

#endif // ORBIT_PLANNER_ORBIT_PLANNER_NODE_HPP
