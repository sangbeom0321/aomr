#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include "interface_rb10_apple/msg/robot_tcp_pose.hpp"

using json = nlohmann::json;

class RoverPoseLoader : public rclcpp::Node {
public:
    RoverPoseLoader()
        : Node("rover_pose_loader") {
        // Create a service to load joint states
        load_service_ = this->create_service<std_srvs::srv::Trigger>(
            "load_rover_pose",
            std::bind(&RoverPoseLoader::loadRoverStateCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "RoverPoseLoader node initialized. Ready to load rover poses.");
    }

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
    std::vector<interface_rb10_apple::msg::RobotTcpPose> arm_poses_;
    std::vector<interface_rb10_apple::msg::RobotTcpPose> base_poses_;

    void loadRoverStateCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;  // Unused parameter

        const std::string filename = "/root/ros2_ws/src/goal_pose_manager/json/rover_pose_log.json";

        if (loadRoverPosesFromFile(filename)) {
            response->success = true;
            response->message = "Loaded rover poses successfully. Total states: " + std::to_string(arm_poses_.size());
            RCLCPP_INFO(this->get_logger(), "Loaded %zu poses from file.", arm_poses_.size());
        } else {
            response->success = false;
            response->message = "Failed to load rover pose from file.";
            RCLCPP_ERROR(this->get_logger(), "Failed to load rover poses from file.");
        }
    }

    bool loadRoverPosesFromFile(const std::string &filename) {
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
            return false;
        }

        json log_data;
        try {
            infile >> log_data;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON file: %s", e.what());
            return false;
        }

        // Verify JSON structure
        if (!log_data.contains("rover_poses") || !log_data["rover_poses"].is_array()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid JSON structure: Missing or invalid 'rover_poses' array.");
            return false;
        }

        arm_poses_.clear();
        base_poses_.clear();

        // Parse joint_states array
        for (const auto &entry : log_data["rover_poses"]) {
            try {
		interface_rb10_apple::msg::RobotTcpPose arm_pose;
		interface_rb10_apple::msg::RobotTcpPose base_pose;

                // Parse joint names, positions, velocities, and efforts
                arm_pose.x = entry["arm"]["x"].get<double>();
                arm_pose.y = entry["arm"]["y"].get<double>();
                arm_pose.z = entry["arm"]["z"].get<double>();
                arm_pose.rx = entry["arm"]["rx"].get<double>();
                arm_pose.ry = entry["arm"]["ry"].get<double>();
                arm_pose.rz = entry["arm"]["rz"].get<double>();

                arm_poses_.push_back(arm_pose);

                base_pose.x = entry["base"]["x"].get<double>();
                base_pose.y = entry["base"]["y"].get<double>();
                base_pose.z = entry["base"]["z"].get<double>();
                base_pose.rx = entry["base"]["rx"].get<double>();
                base_pose.ry = entry["base"]["ry"].get<double>();
                base_pose.rz = entry["base"]["rz"].get<double>();
                
		base_poses_.push_back(base_pose);

            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid rover pose entry: %s", e.what());
                continue;
            }
        }

        infile.close();
        return true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPoseLoader>());
    rclcpp::shutdown();
    return 0;
}


