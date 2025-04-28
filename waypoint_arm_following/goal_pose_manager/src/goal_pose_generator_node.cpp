#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "interface_rb10_apple/action/robot_joint_control.hpp"
#include "interface_rb10_apple/msg/robot_joint_position.hpp"

using json = nlohmann::json;

class GoalPoseGenerator : public rclcpp::Node {
public:
	GoalPoseGenerator() : Node("goal_pose_generator"), current_pose_index_(0) {
	
			
		load_pose_from_json("/root/ros2_ws/src/goal_pose_manager/json/rover_pose_log.json");
		
		navigate_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
		manipulate_action_client_ = rclcpp_action::create_client<interface_rb10_apple::action::RobotJointControl>(this, "robot_joint_control");
		inspector_srv_client_ = this->create_client<std_srvs::srv::Trigger>("trigger_inspector");

		start_inspection();
	}

private:
	std::vector<geometry_msgs::msg::Pose> base_pose_goals_;
	std::vector<interface_rb10_apple::msg::RobotJointPosition> arm_pose_goals_;
	uint16_t current_pose_index_;

	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_action_client_;
	rclcpp_action::Client<interface_rb10_apple::action::RobotJointControl>::SharedPtr manipulate_action_client_;
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr inspector_srv_client_;
		
	// Read json file
	void load_pose_from_json(const std::string &file_path) 
	{
		std::ifstream file(file_path);

		if(!file.is_open()) {
			throw std::runtime_error("Unable to open JSON file");
		}

		json log_data;
		file >> log_data;

		for(const auto &pose_data : log_data["rover_poses"])
		{
			geometry_msgs::msg::Pose pose;
			interface_rb10_apple::msg::RobotJointPosition joint;

			pose.position.x = pose_data["base"]["x"].get<double>();
			pose.position.y = pose_data["base"]["y"].get<double>();
			pose.position.z	= pose_data["base"]["z"].get<double>();
			pose.orientation.x = pose_data["base"]["qx"].get<double>();
			pose.orientation.y = pose_data["base"]["qy"].get<double>();
			pose.orientation.z = pose_data["base"]["qz"].get<double>();
			pose.orientation.w = pose_data["base"]["qw"].get<double>();
			
			base_pose_goals_.push_back(pose);

			joint.j0 = pose_data["arm"]["j0"].get<double>();
			joint.j1 = pose_data["arm"]["j1"].get<double>();
			joint.j2 = pose_data["arm"]["j2"].get<double>();
			joint.j3 = pose_data["arm"]["j3"].get<double>();
			joint.j4 = pose_data["arm"]["j4"].get<double>();
			joint.j5 = pose_data["arm"]["j5"].get<double>();

			arm_pose_goals_.push_back(joint);
		}
	}

	std::string to_string(rclcpp_action::ResultCode result_code) 
	{
   		switch (result_code) 
		{
		case rclcpp_action::ResultCode::SUCCEEDED:
            		return "SUCCEEDED";
        	case rclcpp_action::ResultCode::ABORTED:
            		return "ABORTED";
        	case rclcpp_action::ResultCode::CANCELED:
            		return "CANCELED";
        	default:
            		return "UNKNOWN";
		}
    	}

	// Step 1 : Pose the manipulator to its home position
	void start_inspection()
	{	
		if(current_pose_index_ >= base_pose_goals_.size())
		{
			RCLCPP_INFO(this->get_logger(), "Sequence Completed!.");
			return;
		}

		if(!manipulate_action_client_->wait_for_action_server(std::chrono::seconds(10)))
		{
			RCLCPP_ERROR(this->get_logger(), "Manipulate action server not available !.");
		}

		auto goal_msg = interface_rb10_apple::action::RobotJointControl::Goal();
		goal_msg.j0 = 180.0;
		goal_msg.j1 = -70.0;
		goal_msg.j2 = 160.0;
		goal_msg.j3 = -90.0;
		goal_msg.j4 = 180.0;
		goal_msg.j5 =   0.0;

		auto send_goal_options = rclcpp_action::Client<interface_rb10_apple::action::RobotJointControl>::SendGoalOptions();
		send_goal_options.result_callback = [this](auto result)
		{
			if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
			{
				RCLCPP_INFO(this->get_logger(), "Manipulator homing completed !");
				RCLCPP_INFO(this->get_logger(), "Starting Navigation to Pose %d", current_pose_index_);

				navigate_to_goal(base_pose_goals_[current_pose_index_]);
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "Manipulator homing failed!");
			}
		};

		send_goal_options.goal_response_callback = [this](auto goal_handle)
		{
			if(!goal_handle)
			{
				RCLCPP_INFO(this->get_logger(), "Goal Rejected");
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "Goal Accepted");
			}
		};

		rclcpp::sleep_for(std::chrono::seconds(2));	
		manipulate_action_client_->async_send_goal(goal_msg, send_goal_options);
	}

	// Step 2 : Navigate to the goal position
	void navigate_to_goal(const geometry_msgs::msg::Pose &target_pose)
	{
		if(!navigate_action_client_->wait_for_action_server(std::chrono::seconds(10))) 
		{
			RCLCPP_ERROR(this->get_logger(), "Navigate action server not available!.");
		}

		auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
		goal_msg.pose.header.frame_id = "map";
		goal_msg.pose.pose.position = target_pose.position;
		goal_msg.pose.pose.orientation = target_pose.orientation;

		auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
		send_goal_options.result_callback = [this](auto result) 
		{
			if(result.code == rclcpp_action::ResultCode::SUCCEEDED) 
			{
				RCLCPP_INFO(this->get_logger(), "Navigation goal reached !");
				RCLCPP_INFO(this->get_logger(), "Moving Arm to Pose %d", current_pose_index_);

				move_arm(arm_pose_goals_[current_pose_index_]);
			}
			else 
			{
				RCLCPP_INFO(this->get_logger(), "Navigation failed with code %s", to_string(result.code).c_str());
			}
		};

		rclcpp::sleep_for(std::chrono::seconds(2));	
		navigate_action_client_->async_send_goal(goal_msg, send_goal_options);
	}

	// Step 3 : Pose the manipulator to the inspection position 
	void move_arm(const interface_rb10_apple::msg::RobotJointPosition &joint)
	{
		
		if(!manipulate_action_client_->wait_for_action_server(std::chrono::seconds(10)))
		{
			RCLCPP_ERROR(this->get_logger(), "Manipulate action server not available !.");
		}
		
		auto goal_msg = interface_rb10_apple::action::RobotJointControl::Goal();
		goal_msg.j0 = joint.j0;
		goal_msg.j1 = joint.j1;
		goal_msg.j2 = joint.j2;
		goal_msg.j3 = joint.j3;
		goal_msg.j4 = joint.j4;
		goal_msg.j5 = joint.j5;

		auto send_goal_options = rclcpp_action::Client<interface_rb10_apple::action::RobotJointControl>::SendGoalOptions();
		send_goal_options.result_callback = [this](auto result)
		{
			if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
			{
				RCLCPP_INFO(this->get_logger(), "Manipulation goal reached !");

				current_pose_index_++;
				start_inspection();
			}
		};

		rclcpp::sleep_for(std::chrono::seconds(2));	
		manipulate_action_client_->async_send_goal(goal_msg, send_goal_options);
	}

	// Step 4 : Request for the manipulator pose correction value (TCP)
	void request_arm_correction() 
	{
		;
	}
	// Step 5 : Adjust the manipulator pose  
	void adjust_arm_pose()
	{
		;
	}

	// Step 6 : Trigger the inspection device
	void trigger_inspector()
	{
		if(!inspector_srv_client_->wait_for_service(std::chrono::seconds(5)))
		{
			RCLCPP_ERROR(this->get_logger(), "Service not available !");
			return;
		}

		auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
		
		inspector_srv_client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
		{
			try	{
				auto response = future.get();
				current_pose_index_++;
				start_inspection();
			}
			catch(const std::exception &e) {
				RCLCPP_ERROR(this->get_logger(), "Service call failed: %s !", e.what());
			}
		});

	}

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GoalPoseGenerator>());
	rclcpp::shutdown();
	return 0;
}

