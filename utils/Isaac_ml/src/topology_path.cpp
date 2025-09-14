#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

using namespace std;

struct Edge {
    int target;
    double weight;
};

struct PathNode {
    int vertex;
    double distance;
    bool operator>(const PathNode& other) const {
        return distance > other.distance;
    }
};

struct ThetaStarNode {
    int x, y;
    double g_cost = DBL_MAX;
    double h_cost = DBL_MAX;
    double f_cost = DBL_MAX;
    int parent_x = -1;  // 포인터 대신 인덱스 사용
    int parent_y = -1;
    bool in_queue = false;
};

struct PathInfo {
    std::vector<int> path;
    double heading;  // 첫 번째 vertex에서의 heading (다음 vertex를 향하는)
};

// 방향 벡터 추가
const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

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

rclcpp::QoS qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu
);

rclcpp::QoS qos_costmap = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_costmap.history,
        qos_profile_costmap.depth
    ),
    qos_profile_costmap
);

// Dijkstra 함수 전방 선언
vector<int> dijkstra(const unordered_map<int, vector<Edge>>& graph, int start, int end);

class TopologyPathPlanner : public rclcpp::Node {
public:
    TopologyPathPlanner() 
        : Node("topology_path_planner"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)  // tf_buffer를 사용하여 listener 초기화
    {
        // 파일 경로 파라미터
        this->declare_parameter("map_yaml_path", "/root/ros2_ws/src/pcd_cal/maps/map_tree.yaml");
        // this->declare_parameter("map_yaml_path", "/root/map/map.yaml");
        this->declare_parameter("vertex_yaml_path", "/root/ros2_ws/src/Isaac_ml/map/map_vertex.yaml");
        
        // 맵 변환 파라미터
        this->declare_parameter("flip_x", false);
        this->declare_parameter("flip_y", false);
        
        // Inflation 파라미터
        this->declare_parameter("inflation_radius", 2.0);  // meters
        
        // 경로 계획 파라미터
        this->declare_parameter("max_search_area", 10000);  // cells
        this->declare_parameter("path_resolution", 0.05);   // meters (B-spline interval)
        this->declare_parameter("min_points_for_spline", 4);
        this->declare_parameter("points_per_segment", 30);  // minimum points per B-spline segment
        
        // 토픽 이름 파라미터
        this->declare_parameter("map_topic", "topology_map");
        this->declare_parameter("path_topic", "plan");
        this->declare_parameter("odom_topic", "/odom_baselink");
        this->declare_parameter("goal_topic", "/goal_pose");

        // 추가 파라미터
        this->declare_parameter("occupancy_threshold", 70);  // isValidCell에서 사용
        this->declare_parameter("map_threshold", 128);       // 이진화 임계값
        this->declare_parameter("occupied_value", 100);      // 점유 셀 값
        this->declare_parameter("free_value", 0);           // 자유 셀 값

        // Local Path Planning을 위한 추가 파라미터
        this->declare_parameter("local_planner_frequency", 10.0);  // Hz
        this->declare_parameter("local_costmap_topic", "/local_costmap");
        
        // 파라미터 값 읽기
        flip_x_ = this->get_parameter("flip_x").as_bool();
        flip_y_ = this->get_parameter("flip_y").as_bool();
        inflation_radius_ = this->get_parameter("inflation_radius").as_double();
        max_search_area_ = this->get_parameter("max_search_area").as_int();
        path_resolution_ = this->get_parameter("path_resolution").as_double();
        min_points_for_spline_ = this->get_parameter("min_points_for_spline").as_int();
        min_points_per_segment_ = this->get_parameter("points_per_segment").as_int();
        occupancy_threshold_ = this->get_parameter("occupancy_threshold").as_int();
        map_threshold_ = this->get_parameter("map_threshold").as_int();
        occupied_value_ = this->get_parameter("occupied_value").as_int();
        free_value_ = this->get_parameter("free_value").as_int();
        
        // 퍼블리셔 생성
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_topic").as_string(), qos_costmap);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            this->get_parameter("path_topic").as_string(), 10);
            
        // 서브스크라이버 생성
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), qos_imu, 
            std::bind(&TopologyPathPlanner::odom_callback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("goal_topic").as_string(), 10,
            std::bind(&TopologyPathPlanner::goal_callback, this, std::placeholders::_1));
            
        // Local Path Planning을 위한 추가 Publisher/Subscriber
        local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("local_costmap_topic").as_string(), 
            qos_costmap,  // 이미 정의된 best effort QoS 프로파일 사용
            std::bind(&TopologyPathPlanner::local_costmap_callback, this, std::placeholders::_1));
            
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);

        // Local Path Planning Timer
        double frequency = this->get_parameter("local_planner_frequency").as_double();
        local_planner_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/frequency),
            std::bind(&TopologyPathPlanner::local_planner_callback, this));

        loadMap();
        loadGraph();
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    geometry_msgs::msg::PoseStamped current_goal_;
    bool goal_received_ = false;

    // 파라미터 멤버 변수
    bool flip_x_;
    bool flip_y_;
    double inflation_radius_;
    int max_search_area_;
    double path_resolution_;
    int min_points_for_spline_;
    int min_points_per_segment_;
    int occupancy_threshold_;
    int map_threshold_;
    int occupied_value_;
    int free_value_;
    int timer_count = 0;

    // Local Path Planning을 위한 추가 멤버 변수
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::TimerBase::SharedPtr local_planner_timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
    nav_msgs::msg::Path::SharedPtr global_path_;

    // tf 관련 멤버 변수 추가
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_goal_ = *msg;
        goal_received_ = true;
        plan_path();  // 새로운 목표를 받으면 경로 계획 실행
    }

    void plan_path() {
        if (!received_pose_ || !goal_received_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for pose or goal data...");
            return;
        }

        // 현재 위치에서 가장 가까운 vertex 찾기
        int start = findNearestVertex(current_odom_.pose.pose.position);
        
        // 목표 위치에서 가장 가까운 vertex 찾기
        int goal = findNearestVertex(current_goal_.pose.position);
        
        if (start == -1 || goal == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not find nearest vertices");
            return;
        }
        
        // RCLCPP_INFO(this->get_logger(), "Planning path from vertex %d to vertex %d", 
        //     start, goal);
        
        // 경로 계산 및 퍼블리시
        if (!graph_.empty()) {
            PathInfo path_info = findPath(start, goal);
            if (!path_info.path.empty()) {
                nav_msgs::msg::Path path_msg;
                path_msg.header.stamp = this->now();
                path_msg.header.frame_id = "map";

                std::vector<geometry_msgs::msg::PoseStamped> complete_path;

                // 1. 현재 위치에서 첫 번째 vertex까지의 경로 계획
                auto first_vertex_pos = vertex_positions_[path_info.path[0]];
                geometry_msgs::msg::Pose first_vertex_pose;
                first_vertex_pose.position.x = first_vertex_pos.first * 
                    map_msg_.info.resolution + map_msg_.info.origin.position.x;
                first_vertex_pose.position.y = first_vertex_pos.second * 
                    map_msg_.info.resolution + map_msg_.info.origin.position.y;
                first_vertex_pose.position.z = 0.0;

                // 첫 번째 vertex의 방향을 다음 vertex를 향하도록 설정
                if (path_info.path.size() > 1) {
                    auto next_pos = vertex_positions_[path_info.path[1]];
                    double dx = next_pos.first - first_vertex_pos.first;
                    double dy = next_pos.second - first_vertex_pos.second;
                    double yaw = std::atan2(dy, dx);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    first_vertex_pose.orientation = tf2::toMsg(q);
                }

                auto initial_path = calculateThetaStarPath(
                    map_msg_, 
                    current_odom_.pose.pose, 
                    first_vertex_pose);
                
                double gap = 0.0;
                if (initial_path.empty()) {
                    gap = 0.0;
                } else{
                    double dx = initial_path.back().pose.position.x - initial_path.begin()->pose.position.x;
                    double dy = initial_path.back().pose.position.y - initial_path.begin()->pose.position.y;
                    gap = std::hypot(dx, dy);
                }

                if (gap > 2.0) {
                    complete_path.insert(complete_path.end(), 
                    initial_path.begin(), initial_path.end());
                }


                // 2. vertex들을 따라가는 경로 추가
                for (size_t i = 1; i < path_info.path.size() - 1; ++i) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path_msg.header;
                    auto vertex_pos = vertex_positions_[path_info.path[i]];
                    
                    double wx, wy;
                    vertexToWorld(vertex_pos, wx, wy);
                    pose.pose.position.x = wx;
                    pose.pose.position.y = wy;
                    pose.pose.position.z = 0.0;

                    // 방향 설정
                    if (i < path_info.path.size() - 1) {
                        auto next_pos = vertex_positions_[path_info.path[i+1]];
                        double dx = next_pos.first - vertex_pos.first;
                        double dy = next_pos.second - vertex_pos.second;
                        double yaw = std::atan2(dy, dx);
                        tf2::Quaternion q;
                        q.setRPY(0, 0, yaw);
                        pose.pose.orientation = tf2::toMsg(q);
                    } else {
                        double dx = current_goal_.pose.position.x - pose.pose.position.x;
                        double dy = current_goal_.pose.position.y - pose.pose.position.y;
                        double yaw = std::atan2(dy, dx);
                        tf2::Quaternion q;
                        q.setRPY(0, 0, yaw);
                        pose.pose.orientation = tf2::toMsg(q);
                    }
                    
                    complete_path.push_back(pose);
                }

                // 3. 마지막 vertex에서 실제 목표 지점까지의 경로 계획
                if (!path_info.path.empty()) {
                    auto last_vertex_pos = vertex_positions_[path_info.path.back()];
                    geometry_msgs::msg::Pose last_vertex_pose;
                    last_vertex_pose.position.x = last_vertex_pos.first * 
                        map_msg_.info.resolution + map_msg_.info.origin.position.x;
                    last_vertex_pose.position.y = last_vertex_pos.second * 
                        map_msg_.info.resolution + map_msg_.info.origin.position.y;
                    last_vertex_pose.position.z = 0.0;
                    
                    // 마지막 vertex의 방향을 목표 지점을 향하도록 설정
                    double dx = current_goal_.pose.position.x - last_vertex_pose.position.x;
                    double dy = current_goal_.pose.position.y - last_vertex_pose.position.y;
                    double yaw = std::atan2(dy, dx);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    last_vertex_pose.orientation = tf2::toMsg(q);

                    auto final_path = calculateThetaStarPath(
                        map_msg_, 
                        last_vertex_pose, 
                        current_goal_.pose);
                    complete_path.insert(complete_path.end(), 
                        final_path.begin(), final_path.end());
                }

                // 4. 전체 경로에 B-spline 적용
                if (complete_path.size() >= 4) {  // B-spline에는 최소 4개의 점이 필요
                    auto smoothed_path = bsplineInterpolation(complete_path);
                    auto resample_path = resamplePath(complete_path, path_resolution_); 
                    
                    geometry_msgs::msg::PoseStamped last_pose;
		    last_pose.pose = current_goal_.pose;
		    resample_path.push_back(last_pose);
                    path_msg.poses = resample_path;
                } else {
                    path_msg.poses = complete_path;  // 점이 충분하지 않으면 원본 경로 사용
                }
                

                // global_path_ 설정 추가
                global_path_ = std::make_shared<nav_msgs::msg::Path>(path_msg);
                
                // 기존의 path publish
                path_pub_->publish(path_msg);
            }
        }
    }

    // 현재 위치 저장 변수를 Odometry 타입으로 변경
    nav_msgs::msg::Odometry current_odom_;
    bool received_pose_ = false;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

    // 콜백 함수를 Odometry 용으로 변경
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        received_pose_ = true;
    }

    void loadMap() {
        std::string map_yaml_path = this->get_parameter("map_yaml_path").as_string();
        if (map_yaml_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Map YAML path is not set!");
            return;
        }

        try {
            YAML::Node config = YAML::LoadFile(map_yaml_path);
            std::string image_path = config["image"].as<std::string>();
            double resolution = config["resolution"].as<double>();
            
            RCLCPP_INFO(this->get_logger(), "Loading map from: %s", image_path.c_str());
            
            cv::Mat map_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
            
            if (map_image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map image from: %s", image_path.c_str());
                return;
            }

            // Flip the map if needed
            if (flip_x_) {
                cv::flip(map_image, map_image, 1);
                RCLCPP_INFO(this->get_logger(), "Applied horizontal flip to map");
            }
            if (flip_y_) {
                cv::flip(map_image, map_image, 0);
                RCLCPP_INFO(this->get_logger(), "Applied vertical flip to map");
            }

            // 1. 이진화
            cv::Mat binary_map;
            cv::threshold(map_image, binary_map, 128, 255, cv::THRESH_BINARY);

            // 2. 거리 변환
            cv::Mat distance_map;
            cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);

            // 3. cost map 생성
            cv::Mat cost_map = cv::Mat::zeros(distance_map.size(), CV_8UC1);
            double inflation_radius_pixels = inflation_radius_ / resolution;

            for (int y = 0; y < distance_map.rows; ++y) {
                for (int x = 0; x < distance_map.cols; ++x) {
                    float distance = distance_map.at<float>(y, x);
                    
                    // 장애물인 경우
                    if (map_image.at<uchar>(y, x) < 128) {
                        cost_map.at<uchar>(y, x) = 100;
                    }
                    // inflation 영역
                    else if (distance < inflation_radius_pixels) {
                        double cost = 100.0 * (1.0 - distance / inflation_radius_pixels);
                        cost_map.at<uchar>(y, x) = static_cast<uchar>(cost);
                    }
                    // 자유 공간
                    else {
                        cost_map.at<uchar>(y, x) = 0;
                    }
                }
            }

            // occupancy grid로 변환
            map_msg_.header.frame_id = "map";
            map_msg_.header.stamp = this->now();
            map_msg_.info.resolution = resolution;
            map_msg_.info.width = cost_map.cols;
            map_msg_.info.height = cost_map.rows;
            map_msg_.info.origin.position.x = config["origin"][0].as<double>();
            map_msg_.info.origin.position.y = config["origin"][1].as<double>();
            map_msg_.info.origin.position.z = 0.0;
            map_msg_.info.origin.orientation.w = 1.0;

            map_msg_.data.resize(map_msg_.info.width * map_msg_.info.height);
            
            // cost map을 occupancy grid data로 복사
            for (int y = 0; y < cost_map.rows; ++y) {
                for (int x = 0; x < cost_map.cols; ++x) {
                    int index = y * map_msg_.info.width + x;
                    map_msg_.data[index] = cost_map.at<uchar>(y, x);
                }
            }

            RCLCPP_INFO(this->get_logger(), "Successfully created cost map");
            map_pub_->publish(map_msg_);

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse YAML file: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading map: %s", e.what());
        }
    }

    void loadGraph() {
        std::string vertex_yaml_path = this->get_parameter("vertex_yaml_path").as_string();
        if (vertex_yaml_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Vertex YAML path is not set!");
            return;
        }

        try {
            YAML::Node config = YAML::LoadFile(vertex_yaml_path);
            auto vertices = config["graph"]["vertices"];  // graph.vertices로 경로 수정
            
            RCLCPP_INFO(this->get_logger(), "Loading graph from: %s", vertex_yaml_path.c_str());
            
            // 정점 위치 로드
            for (const auto& vertex : vertices) {
                int id = vertex.first.as<int>();
                int x = vertex.second["x"].as<int>();
                int y = vertex.second["y"].as<int>();
                
                // flip 적용
                if (flip_x_) {
                    x = map_msg_.info.width - 1 - x;
                }
                if (flip_y_) {
                    y = map_msg_.info.height - 1 - y;
                }
                
                vertex_positions_[id] = std::make_pair(x, y);
                // RCLCPP_INFO(this->get_logger(), "Loaded vertex %d at (%d, %d)", 
                    // id, x, y);

                // 간선 정보도 함께 로드
                for (const auto& conn : vertex.second["connections"]) {
                    int target = conn.as<int>();
                    auto source_pos = std::make_pair(x, y);
                    auto target_pos = vertex_positions_[target];  // 이미 로드된 vertex만 연결
                    
                    // 거리 계산
                    double dx = source_pos.first - target_pos.first;
                    double dy = source_pos.second - target_pos.second;
                    double distance = std::sqrt(dx*dx + dy*dy);
                    
                    graph_[id].push_back({target, distance});
                    // 양방향 그래프이므로 반대 방향도 추가
                    graph_[target].push_back({id, distance});
                }
            }

            RCLCPP_INFO(this->get_logger(), "Successfully loaded graph with %ld vertices", 
                vertex_positions_.size());

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse vertex YAML: %s", e.what());
        }
    }

    vector<int> dijkstra(const unordered_map<int, vector<Edge>>& graph, int start, int end) {
        unordered_map<int, double> distances;
        unordered_map<int, int> previous;
        priority_queue<PathNode, vector<PathNode>, greater<PathNode>> pq;

        // Initialize distances to infinity
        for (const auto& pair : graph) {
            distances[pair.first] = numeric_limits<double>::infinity();
        }
        distances[start] = 0.0;

        pq.push({start, 0.0});

        while (!pq.empty()) {
            PathNode current = pq.top();
            pq.pop();

            if (current.vertex == end) {
                break;
            }

            for (const Edge& edge : graph.at(current.vertex)) {
                double newDist = distances[current.vertex] + edge.weight;
                if (newDist < distances[edge.target]) {
                    distances[edge.target] = newDist;
                    previous[edge.target] = current.vertex;
                    pq.push({edge.target, newDist});
                }
            }
        }

        // Reconstruct the shortest path
        vector<int> path;
        for (int at = end; at != start; at = previous[at]) {
            path.push_back(at);
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }

    PathInfo findPath(int start, int end) {
        PathInfo result;
        result.heading = 0.0;  // 기본값

        if (graph_.find(start) == graph_.end() || graph_.find(end) == graph_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Start or goal vertex not found in graph!");
            return result;
        }

        result.path = dijkstra(graph_, start, end);
        
        // 경로가 2개 이상의 vertex를 포함할 경우 heading 계산
        if (result.path.size() >= 2) {
            // 첫 번째와 두 번째 vertex의 위치
            auto first_vertex = vertex_positions_[result.path[0]];
            auto second_vertex = vertex_positions_[result.path[1]];
            
            // 월드 좌표계로 변환
            double x1 = (map_msg_.info.width - first_vertex.first) * 
                map_msg_.info.resolution + map_msg_.info.origin.position.x;
            double y1 = first_vertex.second * 
                map_msg_.info.resolution + map_msg_.info.origin.position.y;
            
            double x2 = (map_msg_.info.width - second_vertex.first) * 
                map_msg_.info.resolution + map_msg_.info.origin.position.x;
            double y2 = second_vertex.second * 
                map_msg_.info.resolution + map_msg_.info.origin.position.y;
            
            // heading 계산 (atan2는 -π에서 π 사이의 각도를 반환)
            result.heading = std::atan2(y2 - y1, x2 - x1);
        }
        
        return result;
    }

    void publishPath(const vector<int>& path) {
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty path, nothing to publish");
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        
        for (int vertex_id : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            
            auto pos = vertex_positions_[vertex_id];
            pose.pose.position.x = (map_msg_.info.width - pos.first) * map_msg_.info.resolution + map_msg_.info.origin.position.x;
            pose.pose.position.y = pos.second * map_msg_.info.resolution + map_msg_.info.origin.position.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path_msg.poses.push_back(pose);
            RCLCPP_DEBUG(this->get_logger(), "Added path point for vertex %d at (%.2f, %.2f)", 
                vertex_id, pose.pose.position.x, pose.pose.position.y);
        }
        
        path_pub_->publish(path_msg);
    }

    // 가장 가까운 vertex를 찾는 함수 추가
    int findNearestVertex(const geometry_msgs::msg::Point& position) {
        int nearest_vertex = -1;
        double min_distance = std::numeric_limits<double>::max();
        RCLCPP_INFO(this->get_logger(), "findNearestVertex");
        
        // vertex_positions_ 맵의 크기 확인을 위한 로그 추가
        RCLCPP_INFO(this->get_logger(), "Number of vertices in map: %ld", vertex_positions_.size());

        for (const auto& vertex : vertex_positions_) {
            // RCLCPP_INFO(this->get_logger(), "Checking vertex %d at position (%d, %d)",
            //     vertex.first, vertex.second.first, vertex.second.second);
                
            double wx, wy;
            vertexToWorld(vertex.second, wx, wy);

            double dx = wx - position.x;
            double dy = wy - position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            // RCLCPP_INFO(this->get_logger(), "distance is %.2f", distance);

            if (distance < min_distance) {
                min_distance = distance;
                nearest_vertex = vertex.first;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Nearest vertex to position (%.2f, %.2f) is vertex %d",
            position.x, position.y, nearest_vertex);
        return nearest_vertex;
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    nav_msgs::msg::OccupancyGrid map_msg_;
    unordered_map<int, vector<Edge>> graph_;
    std::map<int, std::pair<int, int>> vertex_positions_;

    /**
     * @brief Plan a path using Theta* algorithm
     * @param map The occupancy grid map
     * @param start Start pose
     * @param goal Goal pose
     * @return Path as a vector of geometry_msgs::msg::PoseStamped
     */
    nav_msgs::msg::Path planThetaStarPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& goal)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        
        std::vector<geometry_msgs::msg::PoseStamped> path = calculateThetaStarPath(map, start, goal);
        path_msg.poses = path;  // vector<PoseStamped>를 Path 메시지에 할당
        
        return path_msg;
    }

    /**
     * @brief Plan a path using Theta* algorithm
     * @param map The occupancy grid map
     * @param start Start pose
     * @param goal Goal pose
     * @return Path as a vector of geometry_msgs::msg::PoseStamped
     */
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

        // RCLCPP_INFO(this->get_logger(), "Planning path from (%d,%d) to (%d,%d)", 
        //     start_x, start_y, goal_x, goal_y);

        // 탐색 영역 제한
        int search_min_x = std::max(0, (int)start_x - max_search_area_);
        int search_max_x = std::min((int)map.info.width - 1, (int)start_x + max_search_area_);
        int search_min_y = std::max(0, (int)start_y - max_search_area_);
        int search_max_y = std::min((int)map.info.height - 1, (int)start_y + max_search_area_);

        // Initialize nodes grid with limited size
        std::vector<std::vector<ThetaStarNode>> nodes(
            search_max_x - search_min_x + 1,
            std::vector<ThetaStarNode>(search_max_y - search_min_y + 1));

        // RCLCPP_INFO(this->get_logger(), "Initialized nodes grid of size %ldx%ld",
        //     static_cast<long>(nodes.size()), static_cast<long>(nodes[0].size()));

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
        // RCLCPP_INFO(this->get_logger(), "Added start node to open list with f_cost: %.2f", 
        //     nodes[local_start_x][local_start_y].f_cost);

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
                // RCLCPP_INFO(this->get_logger(), "Goal found after %d iterations!", iterations);
                path = reconstructPath(map, nodes, current_x, current_y);
                // RCLCPP_INFO(this->get_logger(), "Path reconstructed with %ld points", path.size());
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

        // RCLCPP_INFO(this->get_logger(), "Path planning completed with %ld points", path.size());
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
            pose.header.frame_id = "map";
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

    // Helper functions for coordinate transformation
    bool worldToMap(
        const nav_msgs::msg::OccupancyGrid& map,
        double wx, double wy,
        unsigned int& mx, unsigned int& my)
    {
        double map_x = (wx - map.info.origin.position.x) / map.info.resolution;
        double map_y = (wy - map.info.origin.position.y) / map.info.resolution;
        
        if (map_x < 0 || map_x >= map.info.width || 
            map_y < 0 || map_y >= map.info.height) {
            return false;
        }
        
        mx = static_cast<unsigned int>(map_x);
        my = static_cast<unsigned int>(map_y);
        return true;
    }

    void mapToWorld(
        const nav_msgs::msg::OccupancyGrid& map,
        int mx, int my,
        double& wx, double& wy)
    {
        wx = mx * map.info.resolution + map.info.origin.position.x;
        wy = my * map.info.resolution + map.info.origin.position.y;
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
        return map.data[y * map.info.width + x] < occupancy_threshold_;  // 파라미터화된 임계값 사용
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

    // B-spline 보간 (3차)
    std::vector<geometry_msgs::msg::PoseStamped> bsplineInterpolation(
        const std::vector<geometry_msgs::msg::PoseStamped>& points,
        double interval = 0.05)  // 5cm 간격
    {
        if (points.size() < min_points_for_spline_) return points;
        
        std::vector<geometry_msgs::msg::PoseStamped> result;
        
        // B-spline 기저 함수
        auto b0 = [](double t) { return (1-t)*(1-t)*(1-t)/6.0; };
        auto b1 = [](double t) { return (3*t*t*t - 6*t*t + 4)/6.0; };
        auto b2 = [](double t) { return (-3*t*t*t + 3*t*t + 3*t + 1)/6.0; };
        auto b3 = [](double t) { return t*t*t/6.0; };

        // 전체 경로 길이 계산
        double path_length = 0.0;
        for (size_t i = 1; i < points.size(); i++) {
            const auto& p1 = points[i-1].pose.position;
            const auto& p2 = points[i].pose.position;
            path_length += std::hypot(p2.x - p1.x, p2.y - p1.y);
        }
        
        int segments = points.size() - 3;
        int points_per_segment = std::max(min_points_per_segment_, 
            static_cast<int>((path_length / path_resolution_) / segments));

        // 각 세그먼트에 대해
        for (size_t i = 0; i < points.size() - 3; i++) {
            const auto& p0 = points[i].pose.position;
            const auto& p1 = points[i+1].pose.position;
            const auto& p2 = points[i+2].pose.position;
            const auto& p3 = points[i+3].pose.position;

            // 세그먼트 내 보간점 생성
            for (int j = 0; j < points_per_segment; j++) {
                double t = static_cast<double>(j) / points_per_segment;
                
                // B-spline 보간
                double x = b0(t)*p0.x + b1(t)*p1.x + b2(t)*p2.x + b3(t)*p3.x;
                double y = b0(t)*p0.y + b1(t)*p1.y + b2(t)*p2.y + b3(t)*p3.y;

                geometry_msgs::msg::PoseStamped pose;
                pose.header = points[0].header;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                // 방향 계산
                if (!result.empty()) {
                    const auto& prev = result.back().pose.position;
                    double dx = x - prev.x;
                    double dy = y - prev.y;
                    
                    if (std::hypot(dx, dy) > 1e-6) {  // 너무 가까운 점은 건너뜀
                        double yaw = std::atan2(dy, dx);
                        tf2::Quaternion q;
                        q.setRPY(0, 0, yaw);
                        pose.pose.orientation = tf2::toMsg(q);
                        result.push_back(pose);
                    }
                } else {
                    // 첫 번째 점의 방향
                    double dx = p1.x - p0.x;
                    double dy = p1.y - p0.y;
                    double yaw = std::atan2(dy, dx);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    pose.pose.orientation = tf2::toMsg(q);
                    result.push_back(pose);
                }
            }
        }

        // 마지막 점 추가
        if (!points.empty()) {
            result.push_back(points.back());
        }

        return result;
    }

    // vertex 위치를 월드 좌표로 변환
    void vertexToWorld(const std::pair<int, int>& vertex_pos, 
                      double& wx, double& wy)
    {
        wx = vertex_pos.first * map_msg_.info.resolution + 
             map_msg_.info.origin.position.x;
        wy = vertex_pos.second * map_msg_.info.resolution + 
             map_msg_.info.origin.position.y;
    }

    // Local Costmap Callback
    void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        local_costmap_ = msg;
    }
    
        std::vector<geometry_msgs::msg::PoseStamped> resamplePath(
        const std::vector<geometry_msgs::msg::PoseStamped>& path,
        double target_resolution)  // target_resolution = 0.05 (5cm)
    {
        std::vector<geometry_msgs::msg::PoseStamped> new_path;
        if (path.empty()) return new_path;

        // 1. 경로 상의 누적 거리를 계산
        std::vector<double> cum_dist;
        cum_dist.push_back(0.0);
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].pose.position.x - path[i - 1].pose.position.x;
            double dy = path[i].pose.position.y - path[i - 1].pose.position.y;
            double d = std::hypot(dx, dy);
            cum_dist.push_back(cum_dist.back() + d);
        }

        double total_length = cum_dist.back();
        int num_samples = static_cast<int>(total_length / target_resolution);

        // 2. 0부터 total_length까지 target_resolution 간격으로 샘플링
        for (int i = 0; i <= num_samples; ++i) {
            double target_d = i * target_resolution;

            // target_d가 전체 길이를 넘어가는 경우 마지막 포인트를 넣음
            if (target_d > total_length) {
                new_path.push_back(path.back());
                break;
            }

            // target_d가 속하는 구간 찾기
            size_t j = 0;
            while (j < cum_dist.size() - 1 && cum_dist[j + 1] < target_d) {
                j++;
            }

            // 구간 [j, j+1] 사이에서 선형 보간
            double segment_length = cum_dist[j + 1] - cum_dist[j];
            double t = (target_d - cum_dist[j]) / segment_length;

            geometry_msgs::msg::PoseStamped interp_pose;
            // 위치 선형 보간
            interp_pose.pose.position.x = path[j].pose.position.x + 
                t * (path[j + 1].pose.position.x - path[j].pose.position.x);
            interp_pose.pose.position.y = path[j].pose.position.y + 
                t * (path[j + 1].pose.position.y - path[j].pose.position.y);
            interp_pose.pose.position.z = path[j].pose.position.z + 
                t * (path[j + 1].pose.position.z - path[j].pose.position.z);

            // 간단하게 해당 구간의 기울기로부터 yaw 계산 (더 정교한 보간이 필요하면 slerp 등을 고려)
            double dx = path[j + 1].pose.position.x - path[j].pose.position.x;
            double dy = path[j + 1].pose.position.y - path[j].pose.position.y;
            double yaw = std::atan2(dy, dx);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            interp_pose.pose.orientation = tf2::toMsg(q);

            // 헤더는 첫 포인트의 헤더를 그대로 사용하거나, 필요에 맞게 갱신
            interp_pose.header = path[j].header;

            new_path.push_back(interp_pose);
        }

        return new_path;
    }

    // Local Path Planning Timer Callback
    void local_planner_callback() {
        timer_count++;
        if (timer_count % 100 == 0) {
            map_pub_->publish(map_msg_);
            timer_count = 0;
        }
                
        if (!local_costmap_) {
            // RCLCPP_WARN(this->get_logger(), "No local costmap received");
            return;
        }
        if (!global_path_) {
            // RCLCPP_WARN(this->get_logger(), "No global path available");
            return;
        }
        if (!received_pose_) {
            // RCLCPP_WARN(this->get_logger(), "No pose received");
            return;
        }

        // local costmap 정보 출력
        // RCLCPP_INFO(this->get_logger(), "Local costmap info - width: %d, height: %d, resolution: %f",
        //             local_costmap_->info.width,
        //             local_costmap_->info.height,
        //             local_costmap_->info.resolution);

        // global path 정보 출력
        // RCLCPP_INFO(this->get_logger(), "Global path info - frame_id: %s, points: %zu",
        //             global_path_->header.frame_id.c_str(),
        //             global_path_->poses.size());

        nav_msgs::msg::Path transformed_path;
        transformed_path.header.frame_id = "os_sensor";
        transformed_path.header.stamp = this->now();

        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_.lookupTransform("os_sensor", global_path_->header.frame_id, tf2::TimePointZero);

            // RCLCPP_INFO(this->get_logger(), "Transform found from %s to base_footprint",
            //             global_path_->header.frame_id.c_str());

            for (const auto& pose : global_path_->poses) {
                geometry_msgs::msg::PoseStamped transformed_pose;
                tf2::doTransform(pose, transformed_pose, transform_stamped);
                transformed_path.poses.push_back(transformed_pose);
            }

            // RCLCPP_INFO(this->get_logger(), "Transformed path has %zu points",
            //             transformed_path.poses.size());

        } catch (tf2::TransformException &ex) {
            // RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            return;
        }

        // local costmap 범위 계산
        double costmap_x_max = local_costmap_->info.width * local_costmap_->info.resolution / 2.0;
        double costmap_y_max = local_costmap_->info.height * local_costmap_->info.resolution / 2.0;
        double costmap_x_min = -costmap_x_max;
        double costmap_y_min = -costmap_y_max;

        // RCLCPP_INFO(this->get_logger(), "Local costmap bounds: x[%f, %f], y[%f, %f]",
        //             costmap_x_min, costmap_x_max, costmap_y_min, costmap_y_max);

        nav_msgs::msg::Path local_path_msg;
        local_path_msg.header.frame_id = "os_sensor";
        local_path_msg.header.stamp = this->now();

        const int cost_threshold = 50;  // 비용 임계값 설정
        std::vector<geometry_msgs::msg::PoseStamped> final_path;
        int start_idx = -1;
        int current_obstacle_start = -1;

        // 첫 번째 for문: 연속성 검사 및 기본 경로 생성
        std::vector<geometry_msgs::msg::PoseStamped> continuous_path;
        for (size_t i = 0; i < transformed_path.poses.size(); ++i) {
            const auto& pose = transformed_path.poses[i];
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;

            // local costmap 범위 내에 있는지 확인
            if (x >= costmap_x_min && x <= costmap_x_max &&
                y >= costmap_y_min && y <= costmap_y_max)
            {
                // 시작점 설정 (최초 로컬 경로 포인트)
                if (start_idx == -1) {
                    start_idx = i;
                    continuous_path.push_back(pose);
                    continue;
                }

                // 연속성 체크: 마지막으로 추가한 로컬 경로 점과 현재 후보 점 사이의 거리를 계산
                double dx = pose.pose.position.x - continuous_path.back().pose.position.x;
                double dy = pose.pose.position.y - continuous_path.back().pose.position.y;
                double gap = std::hypot(dx, dy);
                // gap_threshold는 전역 경로 상에서 허용할 최대 간격 (예: 0.10m, 필요에 따라 조정)
                double gap_threshold = 0.10;  // 10cm 예시
                if (gap > gap_threshold) {
                    RCLCPP_WARN(this->get_logger(), "Gap of %.2f m detected; stopping local path extraction", gap);
                    break;  // 연속성이 깨지면 더 이상 추가하지 않음
                }

                continuous_path.push_back(pose);
            }
        }

        // 두 번째 for문: cost 기반 경로 생성 (장애물 회피)
        current_obstacle_start = -1;
        for (size_t i = 0; i < continuous_path.size(); ++i) {
            const auto& pose = continuous_path[i];
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;

            // local costmap 내의 cost 확인
            unsigned int mx, my;
            if (worldToMap(*local_costmap_, x, y, mx, my)) {
                int cost = local_costmap_->data[my * local_costmap_->info.width + mx];
                
                // 장애물 구간 처리: cost 임계값을 넘어가면 장애물 구간의 시작으로 표시
                if (cost > cost_threshold && current_obstacle_start == -1) {
                    current_obstacle_start = i;
                }
                // 장애물 구간 종료 시, 우회 경로 계산
                else if (cost <= cost_threshold && current_obstacle_start != -1) {
                    geometry_msgs::msg::Pose start_pose = continuous_path[current_obstacle_start - 1].pose;
                    geometry_msgs::msg::Pose goal_pose = pose.pose;
                    std::vector<geometry_msgs::msg::PoseStamped> bypass_path =
                        calculateThetaStarPath(*local_costmap_, start_pose, goal_pose);
                    final_path.insert(final_path.end(), bypass_path.begin(), bypass_path.end());
                    current_obstacle_start = -1;
                }
                // 일반 구간: 장애물 구간이 아니라면 해당 포인트 추가
                else if (cost <= cost_threshold && current_obstacle_start == -1) {
                    final_path.push_back(pose);
                }
            }
        }
        if (current_obstacle_start != -1 && !continuous_path.empty()) {
            geometry_msgs::msg::Pose start_pose = continuous_path[current_obstacle_start - 1].pose;
            geometry_msgs::msg::Pose goal_pose = continuous_path.back().pose;

            std::vector<geometry_msgs::msg::PoseStamped> bypass_path = 
                calculateThetaStarPath(*local_costmap_, start_pose, goal_pose);

            final_path.insert(final_path.end(), bypass_path.begin(), bypass_path.end());
        }

        final_path = resamplePath(final_path, 0.05);

        // 최종 경로 발행 전에 frame 변환
        nav_msgs::msg::Path map_frame_path;
        map_frame_path.header.frame_id = "map";
        map_frame_path.header.stamp = this->now();

        try {
            // base_footprint -> map 변환 행렬 얻기
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_.lookupTransform("map", "os_sensor", tf2::TimePointZero);

            // 각 pose를 변환
            for (const auto& local_pose : final_path) {
                geometry_msgs::msg::PoseStamped transformed_pose;
                
                // 현재 시간으로 업데이트
                transformed_pose.header.stamp = this->now();
                transformed_pose.header.frame_id = "map";

                // tf2를 사용하여 pose 변환
                tf2::doTransform(local_pose, transformed_pose, transform);
                
                map_frame_path.poses.push_back(transformed_pose);
            }

            // 변환된 경로 발행
            local_path_pub_->publish(map_frame_path);
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopologyPathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
