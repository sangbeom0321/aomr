#include "orbit_planner/orbit_panel_plugin.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QTimer>
#include <QTime>
#include <rclcpp/rclcpp.hpp>

namespace orbit_planner {

OrbitPanelPlugin::OrbitPanelPlugin(QWidget* parent) 
    : rviz_common::Panel(parent) {
    
    // Create main layout
    auto main_layout = new QVBoxLayout(this);
    
    // Create control buttons
    auto button_layout = new QHBoxLayout();
    
    start_exploration_btn_ = new QPushButton("Start Exploration");
    stop_exploration_btn_ = new QPushButton("Stop Exploration");
    reset_btn_ = new QPushButton("Reset");
    
    button_layout->addWidget(start_exploration_btn_);
    button_layout->addWidget(stop_exploration_btn_);
    button_layout->addWidget(reset_btn_);
    
    // Create status labels
    status_label_ = new QLabel("Status: Idle");
    exploration_area_label_ = new QLabel("Exploration Area: Not defined");
    current_goal_label_ = new QLabel("Current Goal: None");
    frontiers_count_label_ = new QLabel("Frontiers: 0");
    orbit_anchors_count_label_ = new QLabel("Orbit Anchors: 0");
    
    // Create log text area
    log_text_ = new QTextEdit();
    log_text_->setMaximumHeight(100);
    log_text_->setReadOnly(true);
    
    // Add widgets to layout
    main_layout->addLayout(button_layout);
    main_layout->addWidget(status_label_);
    main_layout->addWidget(exploration_area_label_);
    main_layout->addWidget(current_goal_label_);
    main_layout->addWidget(frontiers_count_label_);
    main_layout->addWidget(orbit_anchors_count_label_);
    main_layout->addWidget(log_text_);
    
    // Connect signals
    connect(start_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStartExploration);
    connect(stop_exploration_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onStopExploration);
    connect(reset_btn_, &QPushButton::clicked, this, &OrbitPanelPlugin::onResetExploration);
    
    // Initialize ROS2 node
    ros_node_ = rclcpp::Node::make_shared("orbit_panel_plugin");
    
    // Create publishers and subscribers
    polygon_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "/orbit_planner/exploration_area", 10);
    
    start_exploration_client_ = ros_node_->create_client<std_srvs::srv::Empty>(
        "/orbit_planner/start_exploration");
    
    stop_exploration_client_ = ros_node_->create_client<std_srvs::srv::Empty>(
        "/orbit_planner/stop_exploration");
    
    trajectory_sub_ = ros_node_->create_subscription<nav_msgs::msg::Path>(
        "/orbit_planner/trajectory", 10,
        std::bind(&OrbitPanelPlugin::trajectoryCallback, this, std::placeholders::_1));
    
    goal_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/orbit_planner/goal", 10,
        std::bind(&OrbitPanelPlugin::goalCallback, this, std::placeholders::_1));
    
    frontiers_sub_ = ros_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/frontiers", 10,
        std::bind(&OrbitPanelPlugin::frontiersCallback, this, std::placeholders::_1));
    
    orbit_anchors_sub_ = ros_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/orbit_planner/orbit_anchors", 10,
        std::bind(&OrbitPanelPlugin::orbitAnchorsCallback, this, std::placeholders::_1));
    
    // Initialize variables
    exploration_active_ = false;
    frontiers_count_ = 0;
    orbit_anchors_count_ = 0;
    
    // Create timer for ROS2 spinning
    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout, this, &OrbitPanelPlugin::spin);
    status_timer_->start(10); // 100 Hz
    
    logMessage("Orbit Panel Plugin initialized");
}

// OrbitPanelPlugin::~OrbitPanelPlugin() {
//     if (status_timer_) {
//         status_timer_->stop();
//     }
// }

void OrbitPanelPlugin::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
}

void OrbitPanelPlugin::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
}

void OrbitPanelPlugin::onStartExploration() {
    if (!start_exploration_client_->service_is_ready()) {
        logMessage("Start exploration service not available");
        return;
    }
    
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    start_exploration_client_->async_send_request(request);
    
    exploration_active_ = true;
    logMessage("Exploration started");
}

void OrbitPanelPlugin::onStopExploration() {
    if (!stop_exploration_client_->service_is_ready()) {
        logMessage("Stop exploration service not available");
        return;
    }
    
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    stop_exploration_client_->async_send_request(request);
    
    exploration_active_ = false;
    logMessage("Exploration stopped");
}

void OrbitPanelPlugin::onResetExploration() {
    exploration_active_ = false;
    logMessage("Exploration reset");
}

void OrbitPanelPlugin::trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    // Update trajectory display if needed
}

void OrbitPanelPlugin::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_goal_ = *msg;
    updateStatus();
}

void OrbitPanelPlugin::frontiersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    frontiers_count_ = msg->markers.size();
    updateStatus();
}

void OrbitPanelPlugin::orbitAnchorsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    orbit_anchors_count_ = msg->markers.size();
    updateStatus();
}

void OrbitPanelPlugin::updateStatus() {
    if (exploration_active_) {
        status_label_->setText("Status: Exploring");
    } else {
        status_label_->setText("Status: Idle");
    }
    
    frontiers_count_label_->setText(QString("Frontiers: %1").arg(frontiers_count_));
    orbit_anchors_count_label_->setText(QString("Orbit Anchors: %1").arg(orbit_anchors_count_));
    
    if (current_goal_.pose.position.x != 0.0 || current_goal_.pose.position.y != 0.0) {
        QString goal_text = QString("Current Goal: (%.2f, %.2f)")
            .arg(current_goal_.pose.position.x)
            .arg(current_goal_.pose.position.y);
        current_goal_label_->setText(goal_text);
    } else {
        current_goal_label_->setText("Current Goal: None");
    }
}

void OrbitPanelPlugin::logMessage(const std::string& message) {
    QString timestamp = QTime::currentTime().toString("hh:mm:ss");
    log_text_->append(QString("[%1] %2")
        .arg(timestamp)
        .arg(QString::fromStdString(message)));
}

void OrbitPanelPlugin::spin() {
    rclcpp::spin_some(ros_node_);
}

} // namespace orbit_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(orbit_planner::OrbitPanelPlugin, rviz_common::Panel)