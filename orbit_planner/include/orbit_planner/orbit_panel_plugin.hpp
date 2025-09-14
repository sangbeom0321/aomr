#ifndef ORBIT_PLANNER_ORBIT_PANEL_PLUGIN_HPP
#define ORBIT_PLANNER_ORBIT_PANEL_PLUGIN_HPP

#include <memory>
#include <vector>

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QGroupBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QProgressBar>
#include <QListWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QMessageBox>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>

#include <rviz_common/panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>

namespace orbit_planner
{

/**
 * @brief RViz2 panel plugin for orbit planner control
 * 
 * This panel provides a graphical user interface for:
 * - Defining exploration areas
 * - Setting start positions
 * - Starting/stopping exploration
 * - Monitoring exploration progress
 * - Visualizing exploration data
 */
class OrbitPanelPlugin : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor
   * @param parent Parent widget
   */
  explicit OrbitPanelPlugin(QWidget * parent = nullptr);

  /**
   * @brief Destructor
   */
  ~OrbitPanelPlugin();

  /**
   * @brief Initialize the panel
   * @param context RViz context
   */
  void onInitialize() override;

  /**
   * @brief Save panel configuration
   * @param config Config object
   */
  void save(rviz_common::Config config) const override;

  /**
   * @brief Load panel configuration
   * @param config Config object
   */
  void load(const rviz_common::Config & config) override;

protected:
  /**
   * @brief Handle mouse click events
   * @param event Mouse event
   */
  void mousePressEvent(QMouseEvent * event) override;

private slots:
  /**
   * @brief Handle start point selection
   */
  void onSelectStartPoint();

  /**
   * @brief Handle polygon drawing mode toggle
   */
  void onTogglePolygonMode();

  /**
   * @brief Handle start exploration button
   */
  void onStartExploration();

  /**
   * @brief Handle stop exploration button
   */
  void onStopExploration();

  /**
   * @brief Handle clear polygon button
   */
  void onClearPolygon();

  /**
   * @brief Handle parameter changes
   */
  void onParameterChanged();

  /**
   * @brief Handle status update timer
   */
  void updateStatus();

private:
  /**
   * @brief Initialize ROS2 components
   */
  void initializeROS();

  /**
   * @brief Create UI elements
   */
  void createUI();

  /**
   * @brief Setup signal-slot connections
   */
  void setupConnections();

  /**
   * @brief Update UI state based on exploration status
   */
  void updateUIState();

  /**
   * @brief Handle clicked point from RViz
   * @param point Clicked point
   */
  void handleClickedPoint(const geometry_msgs::msg::PointStamped & point);

  /**
   * @brief Add point to exploration polygon
   * @param point Point to add
   */
  void addPolygonPoint(const geometry_msgs::msg::Point & point);

  /**
   * @brief Validate exploration setup
   * @return true if setup is valid
   */
  bool validateExplorationSetup();

  /**
   * @brief Publish exploration polygon
   */
  void publishExplorationPolygon();

  /**
   * @brief Publish start pose
   */
  void publishStartPose();

  /**
   * @brief Update exploration status display
   */
  void updateExplorationStatus();

  /**
   * @brief Update frontier candidates display
   */
  void updateFrontierDisplay();

  /**
   * @brief Update path display
   */
  void updatePathDisplay();

  // ROS2 components
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_exploration_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_exploration_client_;

  // UI components
  QVBoxLayout * main_layout_;
  QGroupBox * exploration_group_;
  QGroupBox * polygon_group_;
  QGroupBox * parameters_group_;
  QGroupBox * status_group_;

  // Exploration controls
  QPushButton * select_start_btn_;
  QPushButton * toggle_polygon_btn_;
  QPushButton * start_exploration_btn_;
  QPushButton * stop_exploration_btn_;
  QPushButton * clear_polygon_btn_;

  // Start point display
  QLabel * start_point_label_;
  QLineEdit * start_x_edit_;
  QLineEdit * start_y_edit_;
  QLineEdit * start_yaw_edit_;

  // Polygon display
  QListWidget * polygon_points_list_;
  QLabel * polygon_area_label_;

  // Parameters
  QDoubleSpinBox * robot_radius_spin_;
  QDoubleSpinBox * goal_tolerance_spin_;
  QDoubleSpinBox * exploration_rate_spin_;
  QDoubleSpinBox * max_planning_distance_spin_;
  QSpinBox * frontier_cluster_min_size_spin_;
  QDoubleSpinBox * frontier_cluster_max_distance_spin_;

  // Status display
  QLabel * exploration_status_label_;
  QLabel * current_goal_label_;
  QLabel * path_length_label_;
  QLabel * frontiers_count_label_;
  QProgressBar * exploration_progress_;
  QTextEdit * status_log_;
  
  // Advanced status display
  QLabel * exploration_percentage_label_;
  QLabel * trees_detected_label_;
  QLabel * rows_detected_label_;
  QLabel * planning_time_label_;
  QLabel * learning_metrics_label_;
  QProgressBar * learning_progress_;
  QLabel * performance_metrics_label_;

  // Data
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PolygonStamped exploration_polygon_;
  nav_msgs::msg::Path current_trajectory_;
  geometry_msgs::msg::PoseStamped current_goal_;
  visualization_msgs::msg::MarkerArray current_markers_;

  // State
  bool polygon_mode_active_;
  bool exploration_active_;
  bool start_point_selected_;
  std::vector<geometry_msgs::msg::Point> polygon_points_;

  // Timers
  QTimer * status_update_timer_;
  QTimer * ui_update_timer_;

  // RViz tools
  rviz_common::Tool * point_tool_;
  rviz_common::Tool * previous_tool_;
};

} // namespace orbit_planner

#endif // ORBIT_PLANNER_ORBIT_PANEL_PLUGIN_HPP
