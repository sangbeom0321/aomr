/**
 * @file orbit_panel_plugin.hpp
 * @brief RViz2 Panel plugin for orbit planner control
 * 
 * @author Sangbeom Woo, Duksu Kim
 * @date 2025-01-15
 * @version 1.0
 * 
 * @details
 * This class implements an RViz2 panel plugin that provides a GUI for
 * defining exploration areas and controlling the orbit planner.
 */

#pragma once

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

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
#include <QSlider>
#include <QProgressBar>
#include <QListWidget>
#include <QTableWidget>
#include <QTabWidget>
#include <QComboBox>
#include <QSlider>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>

#include <memory>
#include <vector>

namespace orbit_planner {

class OrbitPanelPlugin : public rviz_common::Panel {
    Q_OBJECT

public:
    OrbitPanelPlugin(QWidget* parent = nullptr);
    ~OrbitPanelPlugin() = default;

    // RViz panel interface
    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

private slots:
    // UI event handlers
    void onStartPointClicked();
    void onAddPolygonPoint();
    void onClearPolygon();
    void onStartExploration();
    void onStopExploration();
    void onResetExploration();
    void onParameterChanged();
    void onUpdateStatus();

private:
    // ROS2 communication
    rclcpp::Node::SharedPtr ros_node_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_pub_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr frontiers_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr orbit_anchors_sub_;
    
    // Services
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_exploration_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_exploration_client_;
    
    // UI Components
    QVBoxLayout* main_layout_;
    QTabWidget* tab_widget_;
    
    // Control Tab
    QWidget* control_tab_;
    QPushButton* start_point_btn_;
    QPushButton* add_point_btn_;
    QPushButton* clear_polygon_btn_;
    QPushButton* start_exploration_btn_;
    QPushButton* stop_exploration_btn_;
    QPushButton* reset_btn_;
    
    // Status Tab
    QWidget* status_tab_;
    QLabel* status_label_;
    QLabel* exploration_area_label_;
    QLabel* current_goal_label_;
    QLabel* frontiers_count_label_;
    QLabel* orbit_anchors_count_label_;
    QProgressBar* exploration_progress_;
    QTextEdit* log_text_;
    
    // Visualization Tab
    QWidget* visualization_tab_;
    QCheckBox* show_frontiers_cb_;
    QCheckBox* show_orbit_anchors_cb_;
    QCheckBox* show_visited_cb_;
    QCheckBox* show_trajectory_cb_;
    QCheckBox* show_occupancy_grid_cb_;
    
    // Parameters Tab
    QWidget* parameters_tab_;
    QDoubleSpinBox* robot_radius_spin_;
    QDoubleSpinBox* safety_margin_spin_;
    QDoubleSpinBox* max_planning_distance_spin_;
    QDoubleSpinBox* frontier_cluster_min_size_spin_;
    QDoubleSpinBox* yaw_change_weight_spin_;
    QDoubleSpinBox* frontier_gain_weight_spin_;
    QDoubleSpinBox* orbit_radius_spin_;
    QDoubleSpinBox* orbit_spacing_spin_;
    
    // Data
    std::vector<geometry_msgs::msg::Point> polygon_points_;
    geometry_msgs::msg::PointStamped start_point_;
    bool start_point_selected_;
    bool exploration_active_;
    
    // Status
    std::string current_status_;
    geometry_msgs::msg::PoseStamped current_goal_;
    int frontiers_count_;
    int orbit_anchors_count_;
    double exploration_progress_value_;
    
    // Timer for status updates
    QTimer* status_timer_;
    
    // Callbacks
    void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void frontiersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void orbitAnchorsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    // UI update functions
    void updateStatus();
    void updatePolygonDisplay();
    void updateParameterDisplay();
    void logMessage(const std::string& message);
    
    // Utility functions
    void setupUI();
    void setupControlTab();
    void setupStatusTab();
    void setupVisualizationTab();
    void setupParametersTab();
    void connectSignals();
    void loadParameters();
    void saveParameters();
    
    // ROS2 setup
    void setupROS2();
    void publishPolygon();
    void publishStartPose();
    void spin();
};

} // namespace orbit_planner