#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/qos.hpp>
#include <cmath>

class ControlInputNode : public rclcpp::Node {
public:
    ControlInputNode() : Node("control_input_node") {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Publishers
        mod_pub_ = this->create_publisher<std_msgs::msg::Int32>("/Control/mod", 10);
        goal_point_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/goal_point", 10);

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path_drone", 10, std::bind(&ControlInputNode::pathCallback, this, std::placeholders::_1));
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odom_gt", qos, std::bind(&ControlInputNode::odomCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/odom_gt", qos, std::bind(&ControlInputNode::odomCallback, this, std::placeholders::_1));
        heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/heading", qos, std::bind(&ControlInputNode::headingCallback, this, std::placeholders::_1));

        // Initialize parameters
        current_mode_ = 0;
        is_path_received = false;
        is_precise_task = true;
        skipping_hz_ = 5;
        odom_cnt_ = 0;
        heading_euler = 0.0;

        // Initialize current_pose_
        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
        current_pose_.position.z = 0.0;
        current_pose_.orientation.x = 0.0;
        current_pose_.orientation.y = 0.0;
        current_pose_.orientation.z = 0.0;
        current_pose_.orientation.w = 1.0;
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg->poses.empty()) {
            goal_point_ = msg->poses.back().pose;  
            is_path_received = true;
        }
    }

    void odomCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        odom_cnt_++;
        if ((odom_cnt_ % skipping_hz_) == 0) {
            odom_cnt_ = 0;
            
            // Update current_pose_ with the received data
            current_pose_.position.x = msg->data[0];
            current_pose_.position.y = msg->data[1];
            
            // Create quaternion from yaw (assuming the drone only rotates around z-axis)
            tf2::Quaternion q;
            q.setRPY(0, 0, heading_euler);  // Set rotation only around z-axis
            current_pose_.orientation = tf2::toMsg(q);
            
            if (is_heading_received && is_path_received) {
                updateControlMode();
            }
            else if (is_heading_received) {
                std_msgs::msg::Int32 mod_msg;
                mod_msg.data = 3;
                mod_pub_->publish(mod_msg);
            }
        }
    }

    void headingCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        heading_euler = msg->data;
        is_heading_received = true;
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat) {
        tf2::Quaternion tf2_quat;
        tf2::fromMsg(quat, tf2_quat);
        double roll, pitch, yaw;

        tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger(), "Goal orientation euler: x=%.6f, y=%.6f, z=%.6f",roll, pitch, yaw);

        return yaw;
    }

    void updateControlMode() {
        // Calculate the Euclidean distance and yaw difference to the goal
        double distance_to_goal = std::hypot(
            goal_point_.position.x - current_pose_.position.x,
            goal_point_.position.y - current_pose_.position.y);

        // RCLCPP_INFO(this->get_logger(), 
        //             "Goal orientation quaternion: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
        //             goal_point_.orientation.x,
        //             goal_point_.orientation.y,
        //             goal_point_.orientation.z,
        //             goal_point_.orientation.w);

        double yaw_goal_ = getYawFromQuaternion(goal_point_.orientation);
        double yaw_current = getYawFromQuaternion(current_pose_.orientation);
        double yaw_diff = std::fabs(yaw_goal_ - yaw_current);

        // Determine the mode based on the conditions
        if ((distance_to_goal < 0.05 && yaw_diff < 0.0524) && current_mode_ == 1) {  // ~3 degrees in radians
            current_mode_ = 3;  // Mod3: Stop mode
        } else if ((distance_to_goal < 0.1 && yaw_diff < 0.0873) && current_mode_ == 2) {  // ~5 degrees in radians
            current_mode_ = 3;  // Mod3: Stop mode (for Mod2 conditions)
        } else if (distance_to_goal < 0.4) {
            current_mode_ = is_precise_task ? 1 : 2;  // Mod1: Precise, Mod2: Semi-Precise
        } else if (current_mode_ != 2 && current_mode_ != 1) {
            current_mode_ = 0;  // Mod0: Path Follower
        }

        RCLCPP_INFO(this->get_logger(), "Current mode: %d, distance_to_goal: %.6f, yaw_diff: %.6f", current_mode_, distance_to_goal, yaw_diff);
        // Publish control mode (mod)
        std_msgs::msg::Int32 mod_msg;
        mod_msg.data = current_mode_;
        mod_pub_->publish(mod_msg);

        // Publish goal point
        std_msgs::msg::Float64MultiArray goal_point_msg;
        goal_point_msg.data = {goal_point_.position.x, goal_point_.position.y, yaw_goal_};
        goal_point_pub_->publish(goal_point_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mod_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_point_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
    // rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose goal_point_;
    int current_mode_;
    int skipping_hz_;
    int odom_cnt_;
    double yaw_goal_;
    double heading_euler;
    bool is_path_received;
    bool is_precise_task;
    bool is_heading_received;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlInputNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
