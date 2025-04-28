#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

rmw_qos_profile_t qos_profile_costmap{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

rclcpp::QoS qos_costmap = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_costmap.history,
        qos_profile_costmap.depth
    ),
    qos_profile_costmap
);

class LocalCostMapNode : public rclcpp::Node {
public:
    LocalCostMapNode() : Node("local_costmap_node") {
        // LaserScan 구독
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_hy", 10, std::bind(&LocalCostMapNode::scan_callback, this, std::placeholders::_1));

        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", qos_costmap);

        map_size_m_ = 12.0;  // 10m x 10m
        resolution_ = 0.05;   // 5cm resolution
        map_size_ = static_cast<int>(map_size_m_ / resolution_);
        inflation_radius_ = 2.0;  // 1m inflation
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;

    double map_size_m_;
    double resolution_;
    int map_size_;
    double inflation_radius_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<Eigen::Vector2f> points = laserscan_to_points(msg);
        std::vector<int8_t> costmap(map_size_ * map_size_, 0);

        int robot_grid_x = map_size_ / 2;
        int robot_grid_y = map_size_ / 2;

        // LaserScan 포인트를 costmap에 변환
        for (const auto& p : points) {
            if (std::abs(p.x()) > map_size_m_ / 2 || std::abs(p.y()) > map_size_m_ / 2) continue;

            int grid_x = static_cast<int>(robot_grid_x + (p.x()) / resolution_);
            int grid_y = static_cast<int>(robot_grid_y + (p.y()) / resolution_);

            if (grid_x >= 0 && grid_x < map_size_ && grid_y >= 0 && grid_y < map_size_) {
                costmap[grid_y * map_size_ + grid_x] = 100;

                // Inflation 적용
                int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                        int nx = grid_x + dx;
                        int ny = grid_y + dy;
                        if (nx >= 0 && nx < map_size_ && ny >= 0 && ny < map_size_) {
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= inflation_radius_) {
                                int cost = static_cast<int>(100 * (1.0 - (distance / inflation_radius_)));
                                if (cost > costmap[ny * map_size_ + nx]) {
                                    costmap[ny * map_size_ + nx] = cost;
                                }
                            }
                        }
                    }
                }
            }
        }

        publish_costmap(costmap);
    }

    std::vector<Eigen::Vector2f> laserscan_to_points(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        std::vector<Eigen::Vector2f> points;
        float angle = scan->angle_min;
        
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            
            // 유효한 거리값인지 확인
            if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
                // 극좌표를 직교좌표로 변환
                float x = range * std::cos(angle);
                float y = range * std::sin(angle);
                points.emplace_back(x, y);
            }
            angle += scan->angle_increment;
        }
        return points;
    }

    void publish_costmap(const std::vector<int8_t>& costmap) {
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = this->now();
        grid_msg.header.frame_id = "os_sensor";

        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = map_size_;
        grid_msg.info.height = map_size_;

        grid_msg.info.origin.position.x = -(map_size_m_ / 2);
        grid_msg.info.origin.position.y = -(map_size_m_ / 2);
        grid_msg.info.origin.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        grid_msg.info.origin.orientation.x = q.x();
        grid_msg.info.origin.orientation.y = q.y();
        grid_msg.info.origin.orientation.z = q.z();
        grid_msg.info.origin.orientation.w = q.w();

        grid_msg.data = costmap;

        costmap_publisher_->publish(grid_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalCostMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


