// joint_state_to_actuator_state.cpp

#include <memory>
#include <string>
#include <vector>
#include <algorithm>  // std::find

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rover_msgs/msg/actuator_state_array.hpp"
#include "rover_msgs/msg/actuator_state.hpp"
#include "rover_msgs/msg/motor_state.hpp"
#include "rover_msgs/msg/driver_state.hpp"

class JointStateToActuatorState : public rclcpp::Node
{
public:
  JointStateToActuatorState()
  : Node("joint_state_to_actuator_state")
  {
    // 필요한 조인트 이름 설정
    target_joints_ = {
      "l_front_wheel_rotate_joint",
      "r_front_wheel_rotate_joint",
      "l_rear_wheel_rotate_joint",
      "r_rear_wheel_rotate_joint",
      "l_front_wheel_joint",
      "r_front_wheel_joint",
      "l_rear_wheel_joint",
      "r_rear_wheel_joint"
    };

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointStateToActuatorState::joint_state_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<rover_msgs::msg::ActuatorStateArray>("/ranger_states", 10);
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto actuator_state_array_msg = rover_msgs::msg::ActuatorStateArray();
    actuator_state_array_msg.header = msg->header; // Header를 그대로 복사

    // target_joints_ 순서대로 ActuatorStateArray 메시지에 추가
    for (const auto & joint_name : target_joints_) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);

      if (it != msg->name.end()) {
        // 조인트 이름이 메시지에서 발견된 경우 처리
        size_t i = std::distance(msg->name.begin(), it);

        // ActuatorState 메시지 생성
        rover_msgs::msg::ActuatorState actuator_state;
        actuator_state.id = static_cast<uint32_t>(i); // id는 조인트 인덱스로 설정

        // MotorState와 DriverState 설정
        rover_msgs::msg::MotorState motor_state;

        // 특정 조인트의 값에 따라 motor_state.driver_state 설정
        if (joint_name.find("rotate_joint") != std::string::npos) {
          motor_state.driver_state = msg->position[i]; // rotate_joint의 position 값을 사용
        } else if (joint_name.find("wheel_joint") != std::string::npos) {
          motor_state.driver_state = msg->velocity[i]; // wheel_joint의 velocity 값을 사용
        }

        actuator_state.motor = motor_state;
        
        // ActuatorStateArray 메시지에 추가
        actuator_state_array_msg.states.push_back(actuator_state);
      }
    }

    publisher_->publish(actuator_state_array_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<rover_msgs::msg::ActuatorStateArray>::SharedPtr publisher_;
  std::vector<std::string> target_joints_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateToActuatorState>());
  rclcpp::shutdown();
  return 0;
}
