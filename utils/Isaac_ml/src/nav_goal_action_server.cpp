#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <memory>

class NavigateToGoalServer : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    NavigateToGoalServer() : Node("navigate_to_goal_server") {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&NavigateToGoalServer::handle_goal, this, _1, _2),
            std::bind(&NavigateToGoalServer::handle_cancel, this, _1),
            std::bind(&NavigateToGoalServer::handle_accepted, this, _1));

        // 목표 포즈를 발행할 퍼블리셔
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        
        // Control/mod 토픽 구독
        control_mod_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/Control/mod", 10, 
            std::bind(&NavigateToGoalServer::control_mod_callback, this, _1));
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10, std::bind(&NavigateToGoalServer::pathCallback, this, std::placeholders::_1));
 
        RCLCPP_INFO(this->get_logger(), "Navigation Goal Action Server is ready");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_mod_sub_;
    
    std::shared_ptr<GoalHandleNavigate> current_goal_handle_;
    bool is_path_received = false;
    bool goal_reached_ = false;
    int cnt =0;

    geometry_msgs::msg::Pose last_goal_point_;
    geometry_msgs::msg::Pose goal_point_;

    void control_mod_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // mode가 3이면 goal에 도달한 것
        
        if ((msg->data == 3 && current_goal_handle_ != nullptr) && is_path_received) {
            cnt++;
            if (cnt > 100) {
                cnt = 0;
                goal_reached_ = true;
            }
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg->poses.empty()) {
            last_goal_point_ = goal_point_;
            if (last_goal_point_ != msg->poses.back().pose) {
                goal_point_ = msg->poses.back().pose;  
                is_path_received = true;
            }
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        using namespace std::placeholders;
        current_goal_handle_ = goal_handle;  // 현재 처리 중인 goal 저장
        goal_reached_ = false;  // goal 도달 상태 초기화
        
        // 비동기 실행
        std::thread{std::bind(&NavigateToGoalServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();

        // 목표 포즈 발행
        goal_pub_->publish(goal->pose);

        // 피드백 주기 설정 (1Hz)
        rclcpp::Rate loop_rate(1);
        
        while (rclcpp::ok()) {
            // 취소 요청 확인
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // 피드백 업데이트
            feedback->distance_remaining = 0.0;  // 실제 거리 계산 필요
            feedback->current_pose = goal->pose;  // 실제 로봇의 현재 위치로 업데이트 필요
            feedback->navigation_time = this->get_clock()->now() - goal->pose.header.stamp;
            feedback->number_of_recoveries = 0;
            goal_handle->publish_feedback(feedback);

            // Control/mod 토픽을 통해 goal 도달 여부 확인
            if (goal_reached_ && is_path_received) {
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                current_goal_handle_ = nullptr;
                is_path_received = false;
                return;
            }

            loop_rate.sleep();
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToGoalServer>());
    rclcpp::shutdown();
    return 0;
}
