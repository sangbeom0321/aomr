#include "orbit_planner/orbit_panel_plugin.hpp"

#include <QApplication>
#include <QGridLayout>
#include <QMessageBox>
#include <QFormLayout>
#include <QMouseEvent>
#include <QGroupBox>
#include <QOverload>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QTimer>
#include <QTextEdit>
#include <QProgressBar>
#include <QCheckBox>
#include <QListWidget>
#include <QSpinBox>
#include <QTableWidget>
#include <QDoubleSpinBox>
#include <QHeaderView>
#include <QComboBox>
#include <QString>
#include <QProgressBar>
#include <QApplication>
#include <QListWidget>
#include <QWidget>
#include <QTableWidget>
#include <QGroupBox>
#include <QHeaderView>
#include <QPushButton>
#include <QMessageBox>
#include <QLabel>
#include <QTimer>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QMouseEvent>
#include <QSpinBox>
#include <QOverload>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QProgressBar>
#include <QListWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QString>
#include <QApplication>
#include <QWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QMouseEvent>
#include <QSpinBox>
#include <QOverload>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QProgressBar>
#include <QListWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QString>
#include <QApplication>
#include <QWidget>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QMouseEvent>
#include <QSpinBox>
#include <QOverload>
#include <QDoubleSpinBox>

#include <rviz_common/display_context.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <thread>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <QApplication>

namespace orbit_planner
{

OrbitPanelPlugin::OrbitPanelPlugin(QWidget * parent)
: rviz_common::Panel(parent)
, main_layout_(nullptr)
, exploration_group_(nullptr)
, polygon_group_(nullptr)
, parameters_group_(nullptr)
, status_group_(nullptr)
, select_start_btn_(nullptr)
, toggle_polygon_btn_(nullptr)
, start_exploration_btn_(nullptr)
, stop_exploration_btn_(nullptr)
, clear_polygon_btn_(nullptr)
, start_point_label_(nullptr)
, start_x_edit_(nullptr)
, start_y_edit_(nullptr)
, start_yaw_edit_(nullptr)
, polygon_points_list_(nullptr)
, polygon_area_label_(nullptr)
, robot_radius_spin_(nullptr)
, goal_tolerance_spin_(nullptr)
, exploration_rate_spin_(nullptr)
, max_planning_distance_spin_(nullptr)
, frontier_cluster_min_size_spin_(nullptr)
, frontier_cluster_max_distance_spin_(nullptr)
, exploration_status_label_(nullptr)
, current_goal_label_(nullptr)
, path_length_label_(nullptr)
, frontiers_count_label_(nullptr)
, exploration_progress_(nullptr)
, status_log_(nullptr)
, polygon_mode_active_(false)
, exploration_active_(false)
, start_point_selected_(false)
, status_update_timer_(nullptr)
, ui_update_timer_(nullptr)
, point_tool_(nullptr)
, previous_tool_(nullptr)
{
  createUI();
  setupConnections();
}

OrbitPanelPlugin::~OrbitPanelPlugin()
{
  if (status_update_timer_) {
    status_update_timer_->stop();
  }
  if (ui_update_timer_) {
    ui_update_timer_->stop();
  }
}

void OrbitPanelPlugin::onInitialize()
{
  rviz_common::Panel::onInitialize();
  
  initializeROS();
  
  // Start timers
  status_update_timer_ = new QTimer(this);
  connect(status_update_timer_, &QTimer::timeout, this, &OrbitPanelPlugin::updateStatus);
  status_update_timer_->start(1000);  // Update every second
  
  ui_update_timer_ = new QTimer(this);
  connect(ui_update_timer_, &QTimer::timeout, this, &OrbitPanelPlugin::updateUIState);
  ui_update_timer_->start(100);  // Update every 100ms
}

void OrbitPanelPlugin::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  
  // Save parameters
  config.mapSetValue("robot_radius", robot_radius_spin_->value());
  config.mapSetValue("goal_tolerance", goal_tolerance_spin_->value());
  config.mapSetValue("exploration_rate", exploration_rate_spin_->value());
  config.mapSetValue("max_planning_distance", max_planning_distance_spin_->value());
  config.mapSetValue("frontier_cluster_min_size", frontier_cluster_min_size_spin_->value());
  config.mapSetValue("frontier_cluster_max_distance", frontier_cluster_max_distance_spin_->value());
}

void OrbitPanelPlugin::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  
  // Load parameters
  double robot_radius, goal_tolerance, exploration_rate, max_planning_distance, frontier_cluster_max_distance;
  int frontier_cluster_min_size;
  
  if (config.mapGetDouble("robot_radius", &robot_radius)) {
    robot_radius_spin_->setValue(robot_radius);
  }
  if (config.mapGetDouble("goal_tolerance", &goal_tolerance)) {
    goal_tolerance_spin_->setValue(goal_tolerance);
  }
  if (config.mapGetDouble("exploration_rate", &exploration_rate)) {
    exploration_rate_spin_->setValue(exploration_rate);
  }
  if (config.mapGetDouble("max_planning_distance", &max_planning_distance)) {
    max_planning_distance_spin_->setValue(max_planning_distance);
  }
  if (config.mapGetInt("frontier_cluster_min_size", &frontier_cluster_min_size)) {
    frontier_cluster_min_size_spin_->setValue(frontier_cluster_min_size);
  }
  if (config.mapGetDouble("frontier_cluster_max_distance", &frontier_cluster_max_distance)) {
    frontier_cluster_max_distance_spin_->setValue(frontier_cluster_max_distance);
  }
}

void OrbitPanelPlugin::mousePressEvent(QMouseEvent * event)
{
  if (polygon_mode_active_) {
    // Handle polygon point selection
    // This would typically involve converting mouse coordinates to world coordinates
    // For now, we'll just add a placeholder point
    geometry_msgs::msg::Point point;
    point.x = 0.0;  // Would be calculated from mouse position
    point.y = 0.0;
    point.z = 0.0;
    addPolygonPoint(point);
  }
  
  QWidget::mousePressEvent(event);
}

void OrbitPanelPlugin::createUI()
{
  main_layout_ = new QVBoxLayout(this);
  
  // Exploration Controls Group
  exploration_group_ = new QGroupBox("Exploration Controls", this);
  QVBoxLayout * exploration_layout = new QVBoxLayout(exploration_group_);
  
  // Start point selection
  QHBoxLayout * start_layout = new QHBoxLayout();
  select_start_btn_ = new QPushButton("Select Start Point", this);
  start_layout->addWidget(select_start_btn_);
  
  start_point_label_ = new QLabel("Not selected", this);
  start_layout->addWidget(start_point_label_);
  exploration_layout->addLayout(start_layout);
  
  // Start point coordinates
  QHBoxLayout * start_coords_layout = new QHBoxLayout();
  start_coords_layout->addWidget(new QLabel("X:", this));
  start_x_edit_ = new QLineEdit("0.0", this);
  start_x_edit_->setEnabled(false);
  start_coords_layout->addWidget(start_x_edit_);
  
  start_coords_layout->addWidget(new QLabel("Y:", this));
  start_y_edit_ = new QLineEdit("0.0", this);
  start_y_edit_->setEnabled(false);
  start_coords_layout->addWidget(start_y_edit_);
  
  start_coords_layout->addWidget(new QLabel("Yaw:", this));
  start_yaw_edit_ = new QLineEdit("0.0", this);
  start_yaw_edit_->setEnabled(false);
  start_coords_layout->addWidget(start_yaw_edit_);
  
  exploration_layout->addLayout(start_coords_layout);
  
  // Polygon controls
  QHBoxLayout * polygon_layout = new QHBoxLayout();
  toggle_polygon_btn_ = new QPushButton("Start Polygon Mode", this);
  polygon_layout->addWidget(toggle_polygon_btn_);
  
  clear_polygon_btn_ = new QPushButton("Clear Polygon", this);
  clear_polygon_btn_->setEnabled(false);
  polygon_layout->addWidget(clear_polygon_btn_);
  
  exploration_layout->addLayout(polygon_layout);
  
  // Polygon points list
  polygon_points_list_ = new QListWidget(this);
  polygon_points_list_->setMaximumHeight(100);
  exploration_layout->addWidget(polygon_points_list_);
  
  // Polygon area
  polygon_area_label_ = new QLabel("Area: Not calculated", this);
  exploration_layout->addWidget(polygon_area_label_);
  
  // Exploration buttons
  QHBoxLayout * button_layout = new QHBoxLayout();
  start_exploration_btn_ = new QPushButton("Start Exploration", this);
  start_exploration_btn_->setEnabled(false);
  button_layout->addWidget(start_exploration_btn_);
  
  stop_exploration_btn_ = new QPushButton("Stop Exploration", this);
  stop_exploration_btn_->setEnabled(false);
  button_layout->addWidget(stop_exploration_btn_);
  
  exploration_layout->addLayout(button_layout);
  
  main_layout_->addWidget(exploration_group_);
  
  // Parameters Group
  parameters_group_ = new QGroupBox("Parameters", this);
  QFormLayout * params_layout = new QFormLayout(parameters_group_);
  
  robot_radius_spin_ = new QDoubleSpinBox(this);
  robot_radius_spin_->setRange(0.1, 2.0);
  robot_radius_spin_->setSingleStep(0.1);
  robot_radius_spin_->setValue(0.4);
  params_layout->addRow("Robot Radius (m):", robot_radius_spin_);
  
  goal_tolerance_spin_ = new QDoubleSpinBox(this);
  goal_tolerance_spin_->setRange(0.1, 5.0);
  goal_tolerance_spin_->setSingleStep(0.1);
  goal_tolerance_spin_->setValue(1.0);
  params_layout->addRow("Goal Tolerance (m):", goal_tolerance_spin_);
  
  exploration_rate_spin_ = new QDoubleSpinBox(this);
  exploration_rate_spin_->setRange(0.1, 10.0);
  exploration_rate_spin_->setSingleStep(0.1);
  exploration_rate_spin_->setValue(1.0);
  params_layout->addRow("Exploration Rate (Hz):", exploration_rate_spin_);
  
  max_planning_distance_spin_ = new QDoubleSpinBox(this);
  max_planning_distance_spin_->setRange(1.0, 100.0);
  max_planning_distance_spin_->setSingleStep(1.0);
  max_planning_distance_spin_->setValue(50.0);
  params_layout->addRow("Max Planning Distance (m):", max_planning_distance_spin_);
  
  frontier_cluster_min_size_spin_ = new QSpinBox(this);
  frontier_cluster_min_size_spin_->setRange(1, 50);
  frontier_cluster_min_size_spin_->setValue(5);
  params_layout->addRow("Min Cluster Size:", frontier_cluster_min_size_spin_);
  
  frontier_cluster_max_distance_spin_ = new QDoubleSpinBox(this);
  frontier_cluster_max_distance_spin_->setRange(0.1, 10.0);
  frontier_cluster_max_distance_spin_->setSingleStep(0.1);
  frontier_cluster_max_distance_spin_->setValue(2.0);
  params_layout->addRow("Max Cluster Distance (m):", frontier_cluster_max_distance_spin_);
  
  main_layout_->addWidget(parameters_group_);
  
  // Status Group
  status_group_ = new QGroupBox("Status", this);
  QVBoxLayout * status_layout = new QVBoxLayout(status_group_);
  
  exploration_status_label_ = new QLabel("Status: Idle", this);
  status_layout->addWidget(exploration_status_label_);
  
  current_goal_label_ = new QLabel("Current Goal: None", this);
  status_layout->addWidget(current_goal_label_);
  
  path_length_label_ = new QLabel("Path Length: 0.0 m", this);
  status_layout->addWidget(path_length_label_);
  
  frontiers_count_label_ = new QLabel("Frontiers: 0", this);
  status_layout->addWidget(frontiers_count_label_);
  
  exploration_progress_ = new QProgressBar(this);
  exploration_progress_->setRange(0, 100);
  exploration_progress_->setValue(0);
  status_layout->addWidget(exploration_progress_);
  
  status_log_ = new QTextEdit(this);
  status_log_->setMaximumHeight(100);
  status_log_->setReadOnly(true);
  status_layout->addWidget(status_log_);
  
  main_layout_->addWidget(status_group_);
  
  setLayout(main_layout_);
}

void OrbitPanelPlugin::setupConnections()
{
  // Button connections
  connect(select_start_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onSelectStartPoint);
  connect(toggle_polygon_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onTogglePolygonMode);
  connect(start_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStartExploration);
  connect(stop_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStopExploration);
  connect(clear_polygon_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onClearPolygon);
  
  // Parameter connections
  connect(robot_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
          this, &OrbitPanelPlugin::onParameterChanged);
  connect(goal_tolerance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
          this, &OrbitPanelPlugin::onParameterChanged);
  connect(exploration_rate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
          this, &OrbitPanelPlugin::onParameterChanged);
  connect(max_planning_distance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
          this, &OrbitPanelPlugin::onParameterChanged);
  connect(frontier_cluster_min_size_spin_, QOverload<int>::of(&QSpinBox::valueChanged), 
          this, &OrbitPanelPlugin::onParameterChanged);
  connect(frontier_cluster_max_distance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
          this, &OrbitPanelPlugin::onParameterChanged);
}

void OrbitPanelPlugin::initializeROS()
{
  // Create ROS2 node
  node_ = rclcpp::Node::make_shared("orbit_panel_plugin");
  
  // Create publishers
  polygon_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
    "/orbit_planner/exploration_polygon", 10);
  start_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/orbit_planner/start_pose", 10);
  
  // Create subscribers
  trajectory_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    "/orbit_planner/trajectory", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      current_trajectory_ = *msg;
      updatePathDisplay();
    });
  
  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/orbit_planner/goal", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      current_goal_ = *msg;
      updateExplorationStatus();
    });
  
  markers_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/orbit_planner/markers", 10,
    [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      current_markers_ = *msg;
      updateFrontierDisplay();
    });
  
  // Create service clients
  start_exploration_client_ = node_->create_client<std_srvs::srv::Empty>("/orbit_planner/start_exploration");
  stop_exploration_client_ = node_->create_client<std_srvs::srv::Empty>("/orbit_planner/stop_exploration");
  
  // Start ROS2 executor in a separate thread
  std::thread([this]() {
    rclcpp::spin(node_);
  }).detach();
}

void OrbitPanelPlugin::onSelectStartPoint()
{
  // In a real implementation, this would enable a point selection tool in RViz
  // For now, we'll just set a default start point
  start_pose_.header.frame_id = "map";
  start_pose_.header.stamp = node_->now();
  start_pose_.pose.position.x = 0.0;
  start_pose_.pose.position.y = 0.0;
  start_pose_.pose.position.z = 0.0;
  start_pose_.pose.orientation.w = 1.0;
  
  start_point_selected_ = true;
  start_point_label_->setText("Selected");
  start_x_edit_->setText(QString::number(start_pose_.pose.position.x));
  start_y_edit_->setText(QString::number(start_pose_.pose.position.y));
  start_yaw_edit_->setText("0.0");
  
  updateUIState();
}

void OrbitPanelPlugin::onTogglePolygonMode()
{
  polygon_mode_active_ = !polygon_mode_active_;
  
  if (polygon_mode_active_) {
    toggle_polygon_btn_->setText("Exit Polygon Mode");
    status_log_->append("Polygon mode activated. Click on the map to add points.");
  } else {
    toggle_polygon_btn_->setText("Start Polygon Mode");
    status_log_->append("Polygon mode deactivated.");
  }
  
  updateUIState();
}

void OrbitPanelPlugin::onStartExploration()
{
  if (!validateExplorationSetup()) {
    QMessageBox::warning(this, "Invalid Setup", "Please select a start point and define an exploration area.");
    return;
  }
  
  // Publish exploration polygon and start pose
  publishExplorationPolygon();
  publishStartPose();
  
  // Call start exploration service
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = start_exploration_client_->async_send_request(request);
  
  // Wait for response
  if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
    exploration_active_ = true;
    status_log_->append("Exploration started successfully.");
  } else {
    status_log_->append("Failed to start exploration.");
  }
  
  updateUIState();
}

void OrbitPanelPlugin::onStopExploration()
{
  // Call stop exploration service
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = stop_exploration_client_->async_send_request(request);
  
  // Wait for response
  if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
    exploration_active_ = false;
    status_log_->append("Exploration stopped.");
  } else {
    status_log_->append("Failed to stop exploration.");
  }
  
  updateUIState();
}

void OrbitPanelPlugin::onClearPolygon()
{
  polygon_points_.clear();
  polygon_points_list_->clear();
  polygon_area_label_->setText("Area: Not calculated");
  status_log_->append("Polygon cleared.");
  
  updateUIState();
}

void OrbitPanelPlugin::onParameterChanged()
{
  // In a real implementation, you would send parameter updates to the planner
  status_log_->append("Parameters updated.");
}

void OrbitPanelPlugin::updateStatus()
{
  // Update status display
  if (exploration_active_) {
    exploration_status_label_->setText("Status: Exploring");
  } else {
    exploration_status_label_->setText("Status: Idle");
  }
  
  // Update goal display
  if (current_goal_.pose.position.x != 0.0 || current_goal_.pose.position.y != 0.0) {
    QString goal_text = QString("Current Goal: (%.2f, %.2f)")
                        .arg(current_goal_.pose.position.x)
                        .arg(current_goal_.pose.position.y);
    current_goal_label_->setText(goal_text);
  } else {
    current_goal_label_->setText("Current Goal: None");
  }
  
  // Update path length
  if (!current_trajectory_.poses.empty()) {
    double path_length = 0.0;
    for (size_t i = 1; i < current_trajectory_.poses.size(); ++i) {
      const auto & p1 = current_trajectory_.poses[i-1].pose.position;
      const auto & p2 = current_trajectory_.poses[i].pose.position;
      path_length += std::sqrt(
        std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    }
    path_length_label_->setText(QString("Path Length: %.2f m").arg(path_length));
  } else {
    path_length_label_->setText("Path Length: 0.0 m");
  }
}

void OrbitPanelPlugin::updateUIState()
{
  // Update button states
  start_exploration_btn_->setEnabled(start_point_selected_ && !polygon_points_.empty() && !exploration_active_);
  stop_exploration_btn_->setEnabled(exploration_active_);
  clear_polygon_btn_->setEnabled(!polygon_points_.empty());
  
  // Update polygon mode button
  if (polygon_mode_active_) {
    toggle_polygon_btn_->setText("Exit Polygon Mode");
  } else {
    toggle_polygon_btn_->setText("Start Polygon Mode");
  }
}

void OrbitPanelPlugin::handleClickedPoint(const geometry_msgs::msg::PointStamped & point)
{
  if (polygon_mode_active_) {
    addPolygonPoint(point.point);
  }
}

void OrbitPanelPlugin::addPolygonPoint(const geometry_msgs::msg::Point & point)
{
  polygon_points_.push_back(point);
  
  // Add to list widget
  QString point_text = QString("Point %1: (%.2f, %.2f)")
                       .arg(polygon_points_.size())
                       .arg(point.x)
                       .arg(point.y);
  polygon_points_list_->addItem(point_text);
  
  // Calculate polygon area (simplified)
  if (polygon_points_.size() >= 3) {
    double area = 0.0;
    for (size_t i = 0; i < polygon_points_.size(); ++i) {
      size_t j = (i + 1) % polygon_points_.size();
      area += polygon_points_[i].x * polygon_points_[j].y;
      area -= polygon_points_[j].x * polygon_points_[i].y;
    }
    area = std::abs(area) / 2.0;
    polygon_area_label_->setText(QString("Area: %.2f m²").arg(area));
  }
  
  updateUIState();
}

bool OrbitPanelPlugin::validateExplorationSetup()
{
  return start_point_selected_ && !polygon_points_.empty();
}

void OrbitPanelPlugin::publishExplorationPolygon()
{
  exploration_polygon_.header.frame_id = "map";
  exploration_polygon_.header.stamp = node_->now();
  exploration_polygon_.polygon.points.clear();
  
  for (const auto & point : polygon_points_) {
    geometry_msgs::msg::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    point32.z = point.z;
    exploration_polygon_.polygon.points.push_back(point32);
  }
  
  polygon_pub_->publish(exploration_polygon_);
}

void OrbitPanelPlugin::publishStartPose()
{
  start_pose_pub_->publish(start_pose_);
}

void OrbitPanelPlugin::updateExplorationStatus()
{
  // This method is called when the goal is updated
  // Implementation would update the status display
}

void OrbitPanelPlugin::updateFrontierDisplay()
{
  // This method is called when frontier markers are updated
  // Implementation would update the frontier count display
  int frontier_count = 0;
  for (const auto & marker : current_markers_.markers) {
    if (marker.ns == "frontiers") {
      frontier_count = marker.points.size();
      break;
    }
  }
  frontiers_count_label_->setText(QString("Frontiers: %1").arg(frontier_count));
}

void OrbitPanelPlugin::updatePathDisplay()
{
  // This method is called when the trajectory is updated
  // Implementation would update the path display
}

} // namespace orbit_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(orbit_planner::OrbitPanelPlugin, rviz_common::Panel)
