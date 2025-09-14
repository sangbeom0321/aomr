#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <cfloat>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

struct ThetaStarNode {
    int x, y;
    double g_cost = DBL_MAX;
    double h_cost = DBL_MAX;
    double f_cost = DBL_MAX;
    int parent_x = -1;  // 포인터 대신 인덱스 사용
    int parent_y = -1;
    bool in_queue = false;
};

rmw_qos_profile_t qos_profile_imu{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    2000,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

rclcpp::QoS qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu
);

// 방향 벡터 추가
const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() 
    : Node("path_planner_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
     {
        // subscription
        costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap", 10, std::bind(&PathPlannerNode::costmap_callback, this, std::placeholders::_1));
        global_path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/topology_path", 10, std::bind(&PathPlannerNode::global_path_callback, this, std::placeholders::_1));
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/imu", 
                rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                std::bind(&PathPlannerNode::odom_callback, this, std::placeholders::_1)
        );

        // publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        

        RCLCPP_INFO(this->get_logger(), "Path Planner Node Initialized");
    }

    

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Path::SharedPtr global_path_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    std::map<int, std::pair<int, int>> vertex_positions_;

    bool received_pose_ = false;

    void global_path_callback(const nav_msgs::msg::Path::SharedPtr global_path) {
        if (global_path->poses.empty()) {
            return;
        }
        global_path_ = global_path;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        received_pose_ = true;  // 위치 데이터를 받았음을 표시
    }

    geometry_msgs::msg::Pose getLocalGoalPose(
        const nav_msgs::msg::Path::SharedPtr& global_path,
        const geometry_msgs::msg::Pose& current_pose,
        const nav_msgs::msg::OccupancyGrid& local_costmap,
        tf2_ros::Buffer& tf_buffer) 
    {
        geometry_msgs::msg::Pose local_goal;

        if (global_path->poses.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("path_planner"), "Global path is empty!");
            return current_pose;  // 경로가 없으면 현재 위치 반환
        }

        nav_msgs::msg::Path transformed_path;
        transformed_path.header.frame_id = "base_link";
        transformed_path.header.stamp = rclcpp::Clock().now();

        std::string global_frame = global_path->header.frame_id;

        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer.lookupTransform("base_link", global_frame, tf2::TimePointZero);

            for (const auto& pose : global_path->poses) {
                geometry_msgs::msg::PoseStamped transformed_pose;

                tf2::doTransform(pose, transformed_pose, transform_stamped);
                transformed_path.poses.push_back(transformed_pose);
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("path_planner"), "TF transform failed: %s", ex.what());
            return current_pose;
        }

        if (transformed_path.poses.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("path_planner"), "Transformed path is empty!");
            return current_pose;
        }

        std::vector<std::pair<int, double>> candidates; // {index, distance_to_goal}

        double local_goal_x = transformed_path.poses.back().pose.position.x;
        double local_goal_y = transformed_path.poses.back().pose.position.y;

        double costmap_x_min = - (local_costmap.info.width * local_costmap.info.resolution / 2);
        double costmap_y_min = - (local_costmap.info.height * local_costmap.info.resolution / 2);
        double costmap_x_max = (local_costmap.info.width * local_costmap.info.resolution / 2);
        double costmap_y_max = (local_costmap.info.height * local_costmap.info.resolution / 2);

        for (size_t i = 0; i < transformed_path.poses.size(); i++) {
            double local_x = transformed_path.poses[i].pose.position.x;
            double local_y = transformed_path.poses[i].pose.position.y;

            // Local Cost Map 바운더리 안에 있는지 확인
            if (local_x >= costmap_x_min && local_x <= costmap_x_max &&
                local_y >= costmap_y_min && local_y <= costmap_y_max) {

                double distance_to_goal = std::hypot(local_x - local_goal_x, local_y - local_goal_y);
                candidates.push_back({i, distance_to_goal});
            }
        }

        // 후보 점 중 Global Goal과 가장 가까운 점 선택
        int best_index = -1;
        if (!candidates.empty()) {
            double min_distance_to_goal = std::numeric_limits<double>::max();

            for (const auto& candidate : candidates) {
                if (candidate.second < min_distance_to_goal) {
                    min_distance_to_goal = candidate.second;
                    best_index = candidate.first;
                }
            }
        }

        if (best_index != -1) {
            local_goal.position.x = transformed_path.poses[best_index].pose.position.x;
            local_goal.position.y = transformed_path.poses[best_index].pose.position.y;
            local_goal.orientation = transformed_path.poses[best_index].pose.orientation;
        } else {
            local_goal.position.x = local_goal_x;
            local_goal.position.y = local_goal_y;
            local_goal.orientation = transformed_path.poses.back().pose.orientation;
        }
        return local_goal;
    }


    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
        if (!received_pose_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry data...");
            return;
        }

        if (!costmap) {
            RCLCPP_ERROR(this->get_logger(), "Received an empty cost map!");
            return;
        }
        map_msg_ = *costmap; 
        
        planAndPublishPath();
    }

    void planAndPublishPath() {
        if (!received_pose_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry data...");
            return;
        }

        if (global_path_ == nullptr || global_path_->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Global path is not available.");
            return;
        }

        geometry_msgs::msg::Pose start_pose, local_start_pose;
        start_pose.position.x = current_odom_.pose.pose.position.x;
        start_pose.position.y = current_odom_.pose.pose.position.y;

        // 로컬 좌표계에서 시작 (로봇 기준)
        local_start_pose.position.x = 0;
        local_start_pose.position.y = 0;

        // 로컬 Goal Pose 계산
        geometry_msgs::msg::Pose goal_pose = getLocalGoalPose(global_path_, start_pose, map_msg_, tf_buffer_);



        // Theta* 알고리즘을 이용하여 경로 생성
        std::vector<geometry_msgs::msg::PoseStamped> planned_path = calculateThetaStarPath(map_msg_, local_start_pose, goal_pose);

        // 경로 퍼블리시
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "base_link";
        path_msg.header.stamp = this->now();
        path_msg.poses = planned_path;

        path_publisher_->publish(path_msg);

    }

    std::vector<geometry_msgs::msg::PoseStamped> calculateThetaStarPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal)
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        // Convert world coordinates to map coordinates
        unsigned int start_x, start_y, goal_x, goal_y;
        if (!worldToMap(map, start.position.x, start.position.y, start_x, start_y) ||
            !worldToMap(map, goal.position.x, goal.position.y, goal_x, goal_y)) {
            RCLCPP_ERROR(this->get_logger(), "Start or goal position outside map bounds");
            return path;
        }

        RCLCPP_INFO(this->get_logger(), "Planning path from (%d,%d) to (%d,%d)", 
            start_x, start_y, goal_x, goal_y);

        // 탐색 영역 제한
        const int max_search_area = 10000;
        int search_min_x = std::max(0, (int)start_x - max_search_area);
        int search_max_x = std::min((int)map.info.width - 1, (int)start_x + max_search_area);
        int search_min_y = std::max(0, (int)start_y - max_search_area);
        int search_max_y = std::min((int)map.info.height - 1, (int)start_y + max_search_area);

        // Initialize nodes grid with limited size
        std::vector<std::vector<ThetaStarNode>> nodes(
            search_max_x - search_min_x + 1,
            std::vector<ThetaStarNode>(search_max_y - search_min_y + 1));

        RCLCPP_INFO(this->get_logger(), "Initialized nodes grid of size %zu x %zu",
                    nodes.size(), nodes[0].size());


        // Initialize start node
        int local_start_x = start_x - search_min_x;
        int local_start_y = start_y - search_min_y;
        nodes[local_start_x][local_start_y].x = start_x;
        nodes[local_start_x][local_start_y].y = start_y;
        nodes[local_start_x][local_start_y].g_cost = 0;
        nodes[local_start_x][local_start_y].h_cost = getHeuristic(start_x, start_y, goal_x, goal_y);
        nodes[local_start_x][local_start_y].f_cost = nodes[local_start_x][local_start_y].h_cost;
        nodes[local_start_x][local_start_y].in_queue = true;

        // Priority queue for open list
        std::priority_queue<std::pair<double, std::pair<int, int>>,
                          std::vector<std::pair<double, std::pair<int, int>>>,
                          std::greater<>> open_list;
        
        open_list.push({nodes[local_start_x][local_start_y].f_cost, {local_start_x, local_start_y}});
        RCLCPP_INFO(this->get_logger(), "Added start node to open list with f_cost: %.2f", 
            nodes[local_start_x][local_start_y].f_cost);

        int iterations = 0;
        while (!open_list.empty()) {
            iterations++;
            if (iterations % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(), "Iteration %d, open list size: %ld", 
                    iterations, open_list.size());
            }

            auto current = open_list.top().second;
            open_list.pop();

            int current_x = current.first;
            int current_y = current.second;

            // Goal found
            if (nodes[current_x][current_y].x == (int)goal_x && 
                nodes[current_x][current_y].y == (int)goal_y) {
                RCLCPP_INFO(this->get_logger(), "Goal found after %d iterations!", iterations);
                path = reconstructPath(map, nodes, current_x, current_y);
                RCLCPP_INFO(this->get_logger(), "Path reconstructed with %ld points", path.size());
                break;
            }

            nodes[current_x][current_y].in_queue = false;

            // Check line of sight with parent's parent
            if (nodes[current_x][current_y].parent_x != -1) {
                int parent_x = nodes[current_x][current_y].parent_x;
                int parent_y = nodes[current_x][current_y].parent_y;
                
                if (nodes[parent_x][parent_y].parent_x != -1) {
                    int grandparent_x = nodes[parent_x][parent_y].parent_x;
                    int grandparent_y = nodes[parent_x][parent_y].parent_y;
                    
                    if (hasLineOfSight(map, 
                        nodes[grandparent_x][grandparent_y].x, nodes[grandparent_x][grandparent_y].y,
                        nodes[current_x][current_y].x, nodes[current_x][current_y].y)) {
                        
                        double new_cost = nodes[grandparent_x][grandparent_y].g_cost + 
                            getDistance(nodes[grandparent_x][grandparent_y].x, nodes[grandparent_x][grandparent_y].y,
                                      nodes[current_x][current_y].x, nodes[current_x][current_y].y);
                        
                        if (new_cost < nodes[current_x][current_y].g_cost) {
                            RCLCPP_DEBUG(this->get_logger(), "Line of sight optimization at (%d,%d)", 
                                nodes[current_x][current_y].x, nodes[current_x][current_y].y);
                            nodes[current_x][current_y].parent_x = grandparent_x;
                            nodes[current_x][current_y].parent_y = grandparent_y;
                            nodes[current_x][current_y].g_cost = new_cost;
                            nodes[current_x][current_y].f_cost = new_cost + nodes[current_x][current_y].h_cost;
                        }
                    }
                }
            }

            // Expand neighbors
            int expanded = 0;
            for (int i = 0; i < 8; i++) {
                int nx = nodes[current_x][current_y].x + dx[i];
                int ny = nodes[current_x][current_y].y + dy[i];

                // 맵 범위 체크
                if (nx < 0 || nx >= (int)map.info.width || 
                    ny < 0 || ny >= (int)map.info.height) {
                    continue;
                }

                // 장애물 체크 (비용이 50 이상이면 장애물)
                if (map.data[ny * map.info.width + nx] >= 50) {
                    RCLCPP_DEBUG(this->get_logger(), "Cell (%d,%d) is obstacle with cost %d", 
                        nx, ny, map.data[ny * map.info.width + nx]);
                    continue;
                }

                // 로컬 좌표계로 변환
                int local_nx = nx - search_min_x;
                int local_ny = ny - search_min_y;

                // 로컬 좌표가 탐색 영역을 벗어나면 스킵
                if (local_nx < 0 || local_nx >= nodes.size() ||
                    local_ny < 0 || local_ny >= nodes[0].size()) {
                    continue;
                }

                // 이미 방문한 노드는 스킵 (이 부분 추가)
                if (!nodes[local_nx][local_ny].in_queue && 
                    nodes[local_nx][local_ny].g_cost != DBL_MAX) {
                    continue;
                }

                // 비용 계산
                double new_g_cost = nodes[current_x][current_y].g_cost + 
                    getDistance(nodes[current_x][current_y].x, nodes[current_x][current_y].y, nx, ny);

                // 더 나은 경로를 찾았거나 아직 방문하지 않은 노드인 경우
                if (!nodes[local_nx][local_ny].in_queue || new_g_cost < nodes[local_nx][local_ny].g_cost) {
                    expanded++;
                    nodes[local_nx][local_ny].x = nx;
                    nodes[local_nx][local_ny].y = ny;
                    nodes[local_nx][local_ny].g_cost = new_g_cost;
                    nodes[local_nx][local_ny].h_cost = getHeuristic(nx, ny, goal_x, goal_y);
                    nodes[local_nx][local_ny].f_cost = new_g_cost + nodes[local_nx][local_ny].h_cost;
                    nodes[local_nx][local_ny].parent_x = current_x;
                    nodes[local_nx][local_ny].parent_y = current_y;

                    RCLCPP_DEBUG(this->get_logger(), 
                        "Node (%d,%d) updated with g=%.2f, h=%.2f, f=%.2f", 
                        nx, ny, new_g_cost, nodes[local_nx][local_ny].h_cost, nodes[local_nx][local_ny].f_cost);

                    if (!nodes[local_nx][local_ny].in_queue) {
                        nodes[local_nx][local_ny].in_queue = true;
                        open_list.push({nodes[local_nx][local_ny].f_cost, {local_nx, local_ny}});
                        RCLCPP_DEBUG(this->get_logger(), "Added node (%d,%d) to open list", nx, ny);
                    }
                }
            }
            
            if (expanded == 0) {
                RCLCPP_WARN(this->get_logger(), "No neighbors expanded at (%d,%d)", 
                    nodes[current_x][current_y].x, nodes[current_x][current_y].y);
            }

            if (iterations > 100000) {
                RCLCPP_WARN(this->get_logger(), "Exceeded maximum iterations (100000)");
                break;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Path planning completed with %ld points", path.size());
        return path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const std::vector<std::vector<ThetaStarNode>>& nodes,
        int current_x, int current_y)
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        while (current_x != -1 && current_y != -1) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "base_link";
            pose.header.stamp = this->now();
            
            double wx, wy;
            mapToWorld(map, nodes[current_x][current_y].x, nodes[current_x][current_y].y, wx, wy);
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path.push_back(pose);
            
            int next_x = nodes[current_x][current_y].parent_x;
            int next_y = nodes[current_x][current_y].parent_y;
            current_x = next_x;
            current_y = next_y;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    // Helper functions for the Theta* algorithm
    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid& map,
        double wx, double wy,
        unsigned int& mx, unsigned int& my)
    {
        mx = static_cast<unsigned int>((wx - map.info.origin.position.x) / map.info.resolution);
        my = static_cast<unsigned int>((wy - map.info.origin.position.y) / map.info.resolution);
        return mx < map.info.width && my < map.info.height;
    }

    void mapToWorld(
        const nav_msgs::msg::OccupancyGrid& map,
        unsigned int mx, unsigned int my,
        double& wx, double& wy)
    {
        wx = map.info.origin.position.x + (mx + 0.5) * map.info.resolution;
        wy = map.info.origin.position.y + (my + 0.5) * map.info.resolution;
    }

    double getHeuristic(int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);
    }

    double getDistance(int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);
    }

    bool isValidCell(const nav_msgs::msg::OccupancyGrid& map, int x, int y) {
        if (x < 0 || x >= (int)map.info.width || y < 0 || y >= (int)map.info.height) {
            return false;
        }
        return map.data[y * map.info.width + x] < 50;  // Assuming 0-100 occupancy values
    }

    bool hasLineOfSight(
        const nav_msgs::msg::OccupancyGrid& map,
        int x1, int y1, int x2, int y2)
    {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int x = x1;
        int y = y1;
        int n = 1 + dx + dy;
        int x_inc = (x2 > x1) ? 1 : -1;
        int y_inc = (y2 > y1) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n) {
            if (!isValidCell(map, x, y)) {
                return false;
            }

            if (error > 0) {
                x += x_inc;
                error -= dy;
            } else {
                y += y_inc;
                error += dx;
            }
        }
        return true;
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




