#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "orbit_planner/orbit_planner_node.hpp"
#include "orbit_planner/orbit_voxblox_interface.hpp"
#include "orbit_planner/advanced_path_planner.hpp"
#include "orbit_planner/orchard_specialized_planner.hpp"

class OrbitPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_orbit_planner");
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(OrbitPlannerTest, TestFrontierDetection)
{
  // Create a simple occupancy grid for testing
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 10;
  grid.info.height = 10;
  grid.info.resolution = 0.5;
  grid.info.origin.position.x = -2.5;
  grid.info.origin.position.y = -2.5;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  
  // Initialize with unknown (-1)
  grid.data.resize(100, -1);
  
  // Create a simple L-shaped obstacle
  for (int i = 0; i < 5; ++i) {
    grid.data[i * 10 + 5] = 100; // Occupied
    grid.data[5 * 10 + i] = 100; // Occupied
  }
  
  // Mark some areas as free
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      grid.data[i * 10 + j] = 0; // Free
    }
  }
  
  // Test frontier detection logic
  std::vector<geometry_msgs::msg::Point> frontiers;
  
  for (int x = 1; x < grid.info.width - 1; ++x) {
    for (int y = 1; y < grid.info.height - 1; ++y) {
      int index = y * grid.info.width + x;
      
      // Check if current cell is free
      if (grid.data[index] != 0) continue;
      
      // Check if any neighbor is unknown
      bool has_unknown_neighbor = false;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          
          int nx = x + dx;
          int ny = y + dy;
          int nindex = ny * grid.info.width + nx;
          
          if (nindex >= 0 && nindex < static_cast<int>(grid.data.size()) &&
              grid.data[nindex] == -1) {
            has_unknown_neighbor = true;
            break;
          }
        }
        if (has_unknown_neighbor) break;
      }
      
      if (has_unknown_neighbor) {
        geometry_msgs::msg::Point frontier;
        frontier.x = grid.info.origin.position.x + x * grid.info.resolution;
        frontier.y = grid.info.origin.position.y + y * grid.info.resolution;
        frontier.z = 0.0;
        frontiers.push_back(frontier);
      }
    }
  }
  
  // Should find frontiers at the boundary between free and unknown
  EXPECT_GT(frontiers.size(), 0);
  
  // Check that frontiers are at expected locations
  bool found_expected_frontier = false;
  for (const auto & frontier : frontiers) {
    if (std::abs(frontier.x - 0.0) < 0.1 && std::abs(frontier.y - 0.0) < 0.1) {
      found_expected_frontier = true;
      break;
    }
  }
  EXPECT_TRUE(found_expected_frontier);
}

TEST_F(OrbitPlannerTest, TestPathPlanning)
{
  // Create a simple test scenario
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "map";
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.position.z = 0.0;
  start.pose.orientation.w = 1.0;
  
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 5.0;
  goal.pose.position.y = 5.0;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;
  
  // Create a simple distance function (no obstacles)
  auto distance_function = [](const geometry_msgs::msg::Point & point) -> double {
    return 1.0; // Always free space
  };
  
  // Test path planning
  orbit_planner::AdvancedPathPlanner::PlanningConfig config;
  config.robot_radius = 0.4;
  config.safety_margin = 0.1;
  config.path_resolution = 0.5;
  
  orbit_planner::AdvancedPathPlanner planner(config);
  auto result = planner.planPath(start, goal, distance_function);
  
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path_length, 0.0);
  EXPECT_FALSE(result.path.poses.empty());
  
  // Check that path starts and ends at correct positions
  EXPECT_NEAR(result.path.poses.front().pose.position.x, start.pose.position.x, 0.1);
  EXPECT_NEAR(result.path.poses.front().pose.position.y, start.pose.position.y, 0.1);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, goal.pose.position.x, 0.1);
  EXPECT_NEAR(result.path.poses.back().pose.position.y, goal.pose.position.y, 0.1);
}

TEST_F(OrbitPlannerTest, TestOrchardRowDetection)
{
  // Create test point cloud data for orchard rows
  std::vector<geometry_msgs::msg::Point> point_cloud;
  
  // Create two parallel rows
  for (int i = 0; i < 10; ++i) {
    // Row 1
    geometry_msgs::msg::Point p1;
    p1.x = i * 2.0;
    p1.y = 0.0;
    p1.z = 1.0;
    point_cloud.push_back(p1);
    
    // Row 2
    geometry_msgs::msg::Point p2;
    p2.x = i * 2.0;
    p2.y = 3.0;
    p2.z = 1.0;
    point_cloud.push_back(p2);
  }
  
  // Test row detection
  orbit_planner::OrchardSpecializedPlanner::OrchardConfig config;
  config.expected_row_spacing = 3.0;
  config.expected_tree_spacing = 2.0;
  config.enable_row_detection = true;
  
  orbit_planner::OrchardSpecializedPlanner planner(config);
  auto rows = planner.detectOrchardRows(point_cloud);
  
  // Should detect at least one row
  EXPECT_GT(rows.size(), 0);
  
  // Check row properties
  if (!rows.empty()) {
    EXPECT_GT(rows[0].length, 0.0);
    EXPECT_GT(rows[0].width, 0.0);
  }
}

TEST_F(OrbitPlannerTest, TestTreeDetection)
{
  // Create test point cloud data for trees
  std::vector<geometry_msgs::msg::Point> point_cloud;
  
  // Create circular tree patterns
  for (int i = 0; i < 5; ++i) {
    double center_x = i * 3.0;
    double center_y = 0.0;
    double radius = 0.25;
    
    for (int j = 0; j < 20; ++j) {
      double angle = j * 2.0 * M_PI / 20.0;
      geometry_msgs::msg::Point p;
      p.x = center_x + radius * std::cos(angle);
      p.y = center_y + radius * std::sin(angle);
      p.z = 1.0;
      point_cloud.push_back(p);
    }
  }
  
  // Test tree detection
  orbit_planner::OrchardSpecializedPlanner::OrchardConfig config;
  config.enable_tree_detection = true;
  config.tree_diameter = 0.5;
  config.min_tree_confidence = 0.7;
  
  orbit_planner::OrchardSpecializedPlanner planner(config);
  auto trees = planner.detectTrees(point_cloud);
  
  // Should detect trees
  EXPECT_GT(trees.size(), 0);
  
  // Check tree properties
  for (const auto & tree : trees) {
    EXPECT_GT(tree.confidence, 0.0);
    EXPECT_GT(tree.estimated_diameter, 0.0);
  }
}

TEST_F(OrbitPlannerTest, TestLearningBasedExplorer)
{
  // Test learning-based explorer initialization
  orbit_planner::LearningBasedExplorer::LearningConfig config;
  config.enable_learning = true;
  config.learning_rate = 0.001;
  config.discount_factor = 0.99;
  
  orbit_planner::LearningBasedExplorer explorer(config);
  explorer.initialize(10, 2); // 10-dimensional state, 2-dimensional action
  
  // Test action selection
  orbit_planner::LearningBasedExplorer::ExplorationState state;
  state.features = Eigen::VectorXd::Random(10);
  state.reward = 0.0;
  state.terminal = false;
  state.state_id = "test_state";
  
  std::vector<orbit_planner::LearningBasedExplorer::ExplorationAction> actions;
  for (int i = 0; i < 5; ++i) {
    orbit_planner::LearningBasedExplorer::ExplorationAction action;
    action.target.x = i * 1.0;
    action.target.y = i * 1.0;
    action.target.z = 0.0;
    action.confidence = 0.5;
    action.action_type = "explore";
    actions.push_back(action);
  }
  
  auto selected_action = explorer.selectAction(state, actions);
  
  // Should select a valid action
  EXPECT_GE(selected_action.confidence, 0.0);
  EXPECT_FALSE(selected_action.action_type.empty());
}

TEST_F(OrbitPlannerTest, TestPerformanceMetrics)
{
  // Test performance monitoring
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Simulate some computation
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  // Should measure time correctly
  EXPECT_GE(duration.count(), 100);
  EXPECT_LE(duration.count(), 200); // Allow some tolerance
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
