#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <mutex>
#include "interface_rb10_apple/msg/robot_joint_position.hpp"


using json = nlohmann::json;

class RoverPoseLogger : public rclcpp::Node {
public:
    RoverPoseLogger()
        : Node("rover_pose_logger"), arm_pose_received_(false), base_pose_received_(false), log_count_(0) {
        // Subscriber for joint_states topic
        arm_pose_sub_ = this->create_subscription<interface_rb10_apple::msg::RobotJointPosition>(
            "robot_joint_position", 10,
            std::bind(&RoverPoseLogger::armPoseCallback, this, std::placeholders::_1)
        );

        base_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_baselink", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&RoverPoseLogger::basePoseCallback, this, std::placeholders::_1)
        );

        // Service for triggering data logging
        log_service_ = this->create_service<std_srvs::srv::Trigger>(
            "log_rover_pose",
            std::bind(&RoverPoseLogger::logRoverPoseCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "RoverPoseLogger node initialized.");
    }

private:
    rclcpp::Subscription<interface_rb10_apple::msg::RobotJointPosition>::SharedPtr arm_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_pose_sub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr log_service_;

    interface_rb10_apple::msg::RobotJointPosition::SharedPtr last_arm_pose_;
    nav_msgs::msg::Odometry::SharedPtr last_base_pose_;

    std::mutex arm_pose_mutex_;
    std::mutex base_pose_mutex_;

    bool arm_pose_received_;
    bool base_pose_received_;
    uint16_t log_count_;

    void armPoseCallback(const interface_rb10_apple::msg::RobotJointPosition::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(arm_pose_mutex_);
        last_arm_pose_ = msg;
        arm_pose_received_ = true;
    }

    void basePoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(base_pose_mutex_);
        last_base_pose_ = msg;
        base_pose_received_ = true;
    }

    void logRoverPoseCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // To avoid unused parameter warning

        if (!arm_pose_received_ || !base_pose_received_) {
            response->success = false;
            response->message = "No rover_poses received yet.";
            return;
        }

        // Convert joint state to JSON
        json rover_pose_json;

	rover_pose_json["index"] = log_count_++;

	rover_pose_json["arm"]["j0"] = last_arm_pose_->j0;
	rover_pose_json["arm"]["j1"] = last_arm_pose_->j1;
	rover_pose_json["arm"]["j2"] = last_arm_pose_->j2;
	rover_pose_json["arm"]["j3"] = last_arm_pose_->j3;
	rover_pose_json["arm"]["j4"] = last_arm_pose_->j4;
	rover_pose_json["arm"]["j5"] = last_arm_pose_->j5;

	rover_pose_json["base"]["x"] = last_base_pose_->pose.pose.position.x;
	rover_pose_json["base"]["y"] = last_base_pose_->pose.pose.position.y;
	rover_pose_json["base"]["z"] = last_base_pose_->pose.pose.position.z;
	rover_pose_json["base"]["qx"] = last_base_pose_->pose.pose.orientation.x;
	rover_pose_json["base"]["qy"] = last_base_pose_->pose.pose.orientation.y;
	rover_pose_json["base"]["qz"] = last_base_pose_->pose.pose.orientation.z;
	rover_pose_json["base"]["qw"] = last_base_pose_->pose.pose.orientation.w;

        const std::string filename = "/root/ros2_ws/src/goal_pose_manager/json/rover_pose_log.json";
        json log_data;

        // Read existing log file if available
        std::ifstream infile(filename);
        if (infile.is_open()) {
            try {
                infile >> log_data;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse existing JSON file: %s", e.what());
            }
            infile.close();
        }

        // Initialize the file if empty or invalid
        if (!log_data.contains("count") || !log_data.contains("rover_poses")) {
            log_data["count"] = 0;
            log_data["rover_poses"] = json::array();
        }

        // Append new joint state and update count
        log_data["rover_poses"].push_back(rover_pose_json);
        log_data["count"] = log_data["rover_poses"].size();

        // Write updated JSON to file
        std::ofstream outfile(filename);
        if (!outfile.is_open()) {
            response->success = false;
            response->message = "Failed to open log file.";
            return;
        }

        outfile << log_data.dump(4) << std::endl; // Pretty print with indentation
        outfile.close();

        response->success = true;
        response->message = "Rover pose logged successfully.";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPoseLogger>());
    rclcpp::shutdown();
    return 0;
}

