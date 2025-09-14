#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class goal_pose_flag: public rclcpp::Node
{
  public:
    goal_pose_flag()
    : Node("goal_pose_flag")
    {
        sub_path= this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&goal_pose_flag::path_callback, this, std::placeholders::_1));

        sub_local=this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/Local/utm", 10, std::bind(&goal_pose_flag::local_callback, this, std::placeholders::_1));

        pub_flag = this->create_publisher<std_msgs::msg::Bool>("/Planning/Control_SW", 10);
        flag_pub=this->create_wall_timer(
        500ms, std::bind(&goal_pose_flag::pub_callback, this));
    }

  private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        int size = msg->poses.size()-1;
        this->x_goal_pose=msg->poses[size].pose.position.x;
        this->y_goal_pose=msg->poses[size].pose.position.y;
    }

    void local_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        this->x_utm_pose=msg->data[0];
        this->y_utm_pose=msg->data[1];
        cal_distance();
    }

    void cal_distance()
    {
        this->distance=std::sqrt(std::pow(x_utm_pose-x_goal_pose,2)+std::pow(y_utm_pose-y_goal_pose,2));
    }

    void pub_callback() 
    {
        std_msgs::msg::Bool flag;
        flag.data=false;
        
        // 얼마나 들어와야 flag True로 할건지 하이퍼 파라미터 설정 필요 임시로 0.4//
        if(distance<0.4) flag.data=true;
        else flag.data=false;
        pub_flag->publish(flag);
    }

    float x_goal_pose=0;
    float y_goal_pose=0;
    float x_utm_pose=0;
    float y_utm_pose=0;
    float distance=0;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_local;

    rclcpp::TimerBase::SharedPtr flag_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_flag;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<goal_pose_flag>());
  rclcpp::shutdown();
  return 0;
}