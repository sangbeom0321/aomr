# AOMR (ROS 2 Humble Workspace)

This repository is a ROS 2 Humble workspace for an autonomous mobile robot stack used in orchard environments (AOMR). It contains four-wheel steering control, LiDAR odometry and localization (LIO-SAM), waypoint/arm following utilities, simulation tools, and assorted helpers. A ready-to-use Docker environment (Ubuntu 22.04 + ROS 2 Humble) is provided for development with GUI support.

## Repository Layout (high level)
- `control/control_4ws/control_4ws`: Four-wheel steering control (C++), depends on `rclcpp`, `std_msgs`, `sensor_msgs`.
- `localization/LIO-SAM-ROS2-REFACTORING`: LIO-SAM port/refactor to ROS 2 (`lio_sam`), LiDAR Odometry using PCL, GTSAM, Eigen.
- `localization/LIO-SAM-LOC-ROS2-REFACTORING`: LIO-SAM localization-only variant (`lio_sam_loc`).
- `localization/robot_localization`: Third-party package for nonlinear state estimation via sensor fusion (Apache 2.0).
- `orbit_planner`: Autonomous exploration planner for orchard environments using Voxblox mapping and frontier-based exploration (C++), integrates with LIO-SAM and control_4ws.
- `waypoint_arm_following/interface_rb10_apple`: ROS 2 interface package (actions/messages) for RB10 manipulator integration.
- `waypoint_arm_following/goal_pose_manager`: Goal management utilities using `rclcpp_action`, `nav2_msgs`, `nav_msgs`, `geometry_msgs`.
- `waypoint_arm_following/pkg_rb10_apple`: Python nodes using the above interfaces/utilities.
- `utils/kimm_orchard`: Gazebo-based orchard world, keyboard teleop, and related tools.
- `utils/Isaac_ml`: Isaac Sim-based integration (WIP).
- `utils/pcd_cal`: Example for integrating PCL in ROS 2.
- `utils/pointcloud_crop`: Point cloud cropping utilities.

> Note: Many subpackages have placeholder descriptions; see each `package.xml` and source for details.

## Quick Start (Docker)
The repository includes a development container with Ubuntu 22.04 + ROS 2 Humble, preinstalled tools (git, colcon, rosdep, vcstool, terminator, gedit, nano), and X11 GUI support.

1) Allow local X11 access (once per host session):
```bash
xhost +local:
```

2) Build the image (use your host UID/GID to match file ownership):
```bash
export UID=$(id -u)
export GID=$(id -g)
export USER=${USER}
docker compose build
```

3) Run and attach a shell:
```bash
docker compose up -d
docker compose exec ros2 bash
```

Inside the container, ROS 2 is sourced automatically. The repository root is mounted at `/workspace`.

## Build (colcon)
From inside the container shell (or a native Humble setup):
```bash
rosdep update
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Examples
- Verify ROS 2:
```bash
ros2 topic list
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener &
```

- LIO-SAM (mapping) and localization (launch files exist under each package's `launch/`):
```bash
ros2 launch lio_sam run.launch.py      # mapping (package: lio_sam)
ros2 launch lio_sam_loc run.launch.py  # localization (package: lio_sam_loc)
```

- Orbit Planner autonomous exploration (from `orbit_planner`):
```bash
ros2 launch orbit_planner orbit_exploration.launch.py  # full exploration system
ros2 run orbit_planner orbit_planner_node             # planner node only
ros2 run orbit_planner orbit_voxblox_interface        # mapping interface only
```

- Gazebo orchard simulation and teleop (from `utils/kimm_orchard`):
```bash
ros2 launch kimm_orchard gazebo.launch.py
ros2 run kimm_orchard keyboard_teleop.py
```

- Robot localization: see `localization/robot_localization/launch/` for ready-made launch files.

## Orbit Planner 상세 사용법

### 설치 및 빌드
```bash
# 필수 패키지 설치
sudo apt install ros-humble-rviz2 \
                 ros-humble-pcl-ros \
                 ros-humble-pcl-conversions \
                 ros-humble-eigen3-cmake-module \
                 ros-humble-tf2-geometry-msgs \
                 ros-humble-visualization-msgs

# Voxblox 설치 (소스에서 빌드)
cd ~/ros2_ws/src
git clone https://github.com/ethz-asl/voxblox.git
cd voxblox
git checkout ros2

# 패키지 빌드
colcon build --packages-select orbit_planner
source install/setup.bash
```

### 기본 실행
```bash
# 전체 시스템 실행
ros2 launch orbit_planner orbit_exploration.launch.py

# RViz 없이 실행
ros2 launch orbit_planner orbit_exploration.launch.py use_rviz:=false

# 시뮬레이션 시간 사용
ros2 launch orbit_planner orbit_exploration.launch.py use_sim_time:=true
```

### RViz2 패널 사용법
1. **패널 로드**: RViz2 → Panels → Add New Panel → Orbit Planner Panel
2. **탐사 영역 설정**:
   - "Select Start Point" 버튼으로 시작점 선택
   - "Start Polygon Mode" 버튼으로 탐사 영역 다각형 그리기
   - "Start Exploration" 버튼으로 탐사 시작
3. **매개변수 조정**: Robot Radius, Goal Tolerance, Exploration Rate 등 설정 가능

### 고급 설정
```bash
# 설정 파일 편집
nano ~/ros2_ws/src/orbit_planner/config/orbit_planner_params.yaml

# 런타임 매개변수 변경
ros2 param set /orbit_planner robot_radius 0.5
ros2 param set /orbit_planner goal_tolerance 1.5
ros2 param set /orbit_planner exploration_rate 0.5

# 토픽 리매핑
ros2 launch orbit_planner orbit_exploration.launch.py \
  --ros-args -r /lio_sam/mapping/cloudRegistered:=/your_pointcloud_topic
```

### API 사용법
```bash
# 서비스 호출
ros2 service call /orbit_planner/start_exploration std_srvs/srv/Empty
ros2 service call /orbit_planner/stop_exploration std_srvs/srv/Empty

# 토픽 구독
ros2 topic echo /orbit_planner/trajectory
ros2 topic echo /orbit_planner/goal
ros2 topic echo /orbit_planner/markers
ros2 topic echo /orbit_planner/occupancy_grid
```

### 주요 기능
- **Voxblox 통합**: TSDF/ESDF 매핑을 통한 실시간 3D 환경 표현
- **프론티어 탐사**: 미지 영역의 경계를 찾아 효율적인 탐사 목표 생성
- **고급 경로 계획**: A*, RRT*, Voronoi 기반의 다중 알고리즘 경로 계획
- **과수원 특화**: 나무 감지, 열 구조 인식, 체계적 탐사 패턴
- **학습 기반 탐사**: 강화학습을 통한 경험 기반 탐사 전략
- **실시간 모니터링**: 탐사 진행률, 성능 메트릭, 학습 상태 추적

### 상세 설정 옵션

#### Voxblox 매개변수
```yaml
voxblox_interface:
  voxel_size: 0.1                    # 복셀 크기 (미터) - 작을수록 정밀하지만 메모리 사용량 증가
  truncation_distance: 0.2           # TSDF 절단 거리 - 센서 노이즈 처리
  esdf_max_distance: 2.0             # 최대 ESDF 거리 - 장애물로부터의 최대 거리
  robot_radius: 0.4                  # 로봇 반지름 - 안전 마진 계산용
  update_rate: 2.0                   # 맵 업데이트 주기 (Hz) - 높을수록 실시간성 향상
  grid_resolution: 0.1               # 2D 격자 해상도
```

#### 플래너 매개변수
```yaml
planner_node:
  robot_radius: 0.4                  # 로봇 반지름 (미터)
  goal_tolerance: 1.0                # 목표 도달 허용 오차 (미터)
  frontier_cluster_min_size: 5       # 최소 프론티어 클러스터 크기
  frontier_cluster_max_distance: 2.0 # 프론티어 클러스터링 최대 거리
  yaw_change_weight: 0.5             # 방향 변경 페널티 가중치
  frontier_gain_weight: 1.0          # 프론티어 정보 획득 가중치
  distance_weight: 0.3               # 거리 비용 가중치
  exploration_rate: 1.0              # 탐사 주기 (Hz)
  max_planning_distance: 50.0        # 최대 계획 거리 (미터)
  visited_radius: 3.0                # 방문 표시 반지름 (미터)
```

#### 고급 경로 계획 매개변수
```yaml
advanced_path_planning:
  algorithm_type: "A_STAR"           # A_STAR, RRT_STAR, VORONOI, HYBRID
  enable_smoothing: true             # 경로 스무딩 활성화
  enable_optimization: true          # 경로 최적화 활성화
  max_planning_time: 5.0             # 최대 계획 시간 (초)
  path_resolution: 0.5               # 경로 해상도 (미터)
  max_curvature: 0.5                 # 최대 경로 곡률
  safety_margin: 0.1                 # 안전 마진 (미터)
  max_iterations: 1000               # 최대 계획 반복 횟수
```

#### 과수원 특화 매개변수
```yaml
orchard_specialized:
  enable_tree_detection: true        # 나무 감지 활성화
  enable_row_detection: true         # 열 감지 활성화
  expected_row_spacing: 3.0          # 예상 열 간격 (미터)
  expected_tree_spacing: 2.0         # 예상 나무 간격 (미터)
  tree_diameter: 0.5                 # 예상 나무 직경 (미터)
  row_width: 1.0                     # 열 폭 (미터)
  exploration_margin: 0.5            # 탐사 마진 (미터)
  exploration_pattern: "ADAPTIVE"    # ROW_BY_ROW, SPIRAL, ZIGZAG, ADAPTIVE, CUSTOM
  min_tree_confidence: 0.7           # 최소 나무 감지 신뢰도
  max_trees_per_row: 50              # 열당 최대 나무 수
```

#### 학습 기반 탐사 매개변수
```yaml
learning_based:
  enable_learning: false             # 학습 기반 탐사 활성화
  enable_experience_replay: true     # 경험 재생 활성화
  learning_rate: 0.001               # 학습률
  discount_factor: 0.99              # 할인 인수
  epsilon_start: 1.0                 # 초기 탐험률
  epsilon_end: 0.01                  # 최종 탐험률
  epsilon_decay: 0.995               # 탐험률 감소율
  replay_buffer_size: 10000          # 경험 재생 버퍼 크기
  batch_size: 32                     # 학습 배치 크기
  update_frequency: 100              # 모델 업데이트 주기
  use_prioritized_replay: true       # 우선순위 기반 경험 재생
  model_save_path: "/tmp/orbit_planner/models" # 모델 저장 경로
```

### 성능 최적화 설정
```yaml
performance:
  enable_multi_threading: true       # 멀티스레딩 활성화
  planning_thread_count: 2           # 계획 스레드 수
  enable_caching: true               # 프론티어 캐싱 활성화
  cache_duration: 5.0                # 캐시 지속 시간 (초)
  adaptive_update_rate: true         # 적응형 업데이트율 활성화
  max_planning_time: 2.0             # 최대 계획 시간 (초)
  enable_parallel_processing: true   # 병렬 처리 활성화
```

### 모니터링 및 로깅 설정
```yaml
monitoring:
  enable_performance_monitoring: true # 성능 모니터링 활성화
  enable_exploration_metrics: true   # 탐사 메트릭 활성화
  metrics_publish_rate: 1.0          # 메트릭 발행 주기 (Hz)
  enable_visualization: true         # 시각화 마커 활성화
  visualization_rate: 2.0            # 시각화 업데이트 주기 (Hz)
  save_exploration_log: true         # 탐사 로그 저장
  log_file_path: "/tmp/orbit_planner/logs" # 로그 파일 경로
```

### 테스트 및 검증
```bash
# 단위 테스트 실행
colcon build --packages-select orbit_planner --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select orbit_planner
colcon test-result --all --verbose

# 성능 테스트 실행
ros2 run orbit_planner test_performance

# 통합 테스트 실행
ros2 run orbit_planner test_integration

# 디버그 모드 실행
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG
ros2 run orbit_planner orbit_planner_node --ros-args --log-level debug

# 프로파일링 활성화
ros2 param set /orbit_planner enable_profiling true
ros2 run orbit_planner orbit_planner_node
```

### 시뮬레이션 환경에서 테스트
```bash
# Gazebo 시뮬레이션 실행
ros2 launch kimm_orchard gazebo.launch.py

# Orbit Planner와 함께 실행
ros2 launch orbit_planner orbit_exploration.launch.py use_sim_time:=true

# Bag 파일로 테스트
ros2 bag play your_orchard_data.bag
ros2 launch orbit_planner orbit_exploration.launch.py
```

### 실시간 모니터링
```bash
# 노드 상태 확인
ros2 node list
ros2 node info /orbit_planner

# 토픽 상태 확인
ros2 topic list
ros2 topic hz /orbit_planner/trajectory
ros2 topic hz /orbit_planner/goal

# 매개변수 확인
ros2 param list /orbit_planner
ros2 param get /orbit_planner robot_radius

# 서비스 확인
ros2 service list | grep orbit_planner
ros2 service type /orbit_planner/start_exploration

# 시스템 리소스 모니터링
htop
nvidia-smi  # GPU 사용량 (Voxblox 사용 시)
```

### RViz2 시각화 설정
```bash
# RViz2에서 확인할 수 있는 항목들:
# - PointCloud2: /pointcloud_map (LIO-SAM 출력)
# - Path: /trajectory (계획된 경로)
# - MarkerArray: /orbit_planner/markers (프론티어, 목표, 나무)
# - Polygon: /orbit_planner/exploration_polygon (탐사 영역)
# - PoseStamped: /exploration_goal (현재 목표)
# - OccupancyGrid: /orbit_planner/occupancy_grid (점유 격자)
# - MarkerArray: /orbit_planner/map_markers (맵 시각화)
```

### 문제 해결 가이드

#### 일반적인 문제
1. **Voxblox 컴파일 오류**:
   ```bash
   # Eigen3 의존성 확인
   sudo apt install libeigen3-dev
   # CMake 버전 확인 (3.8 이상 필요)
   cmake --version
   ```

2. **RViz 플러그인 로드 실패**:
   ```bash
   # 패키지 재빌드 후 환경 설정
   colcon build --packages-select orbit_planner
   source install/setup.bash
   # RViz2 재시작
   ```

3. **메모리 부족**:
   ```bash
   # 복셀 크기 증가
   ros2 param set /orbit_planner voxel_size 0.2
   # ESDF 최대 거리 감소
   ros2 param set /orbit_planner esdf_max_distance 1.0
   ```

4. **계획 실패**:
   ```bash
   # 로봇 반지름 조정
   ros2 param set /orbit_planner robot_radius 0.3
   # 목표 허용 오차 증가
   ros2 param set /orbit_planner goal_tolerance 2.0
   ```

#### 성능 최적화
```bash
# 멀티스레딩 활성화
ros2 param set /orbit_planner enable_multi_threading true

# 캐싱 활성화
ros2 param set /orbit_planner enable_caching true

# 적응형 업데이트율 활성화
ros2 param set /orbit_planner adaptive_update_rate true

# 병렬 처리 활성화
ros2 param set /orbit_planner enable_parallel_processing true
```

#### 로그 분석
```bash
# 로그 레벨 설정
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# 특정 노드 디버깅
ros2 run orbit_planner orbit_planner_node --ros-args --log-level debug

# 로그 파일 확인
tail -f /tmp/orbit_planner/logs/orbit_planner.log
```

## Orbit Planner 코드 구조 및 아키텍처

### 전체 시스템 아키텍처
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   LIO-SAM       │───▶│  Voxblox         │───▶│  Orbit Planner  │
│   (SLAM)        │    │  Interface       │    │  Node           │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │                       ▼                       ▼
         │              ┌──────────────────┐    ┌─────────────────┐
         │              │  TSDF/ESDF       │    │  Path Planning  │
         │              │  Mapping         │    │  & Exploration  │
         │              └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Point Cloud    │    │  Distance Field  │    │  Trajectory     │
│  Output         │    │  & Occupancy     │    │  Output         │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### 패키지 구조
```
orbit_planner/
├── package.xml                    # ROS2 패키지 매니페스트
├── CMakeLists.txt                 # 빌드 설정
├── README.md                      # 패키지 문서
├── include/orbit_planner/         # 헤더 파일
│   ├── orbit_planner_node.hpp     # 메인 플래너 노드
│   ├── orbit_voxblox_interface.hpp # Voxblox 통합 인터페이스
│   ├── orbit_panel_plugin.hpp     # RViz2 패널 플러그인
│   ├── advanced_path_planner.hpp  # 고급 경로 계획 알고리즘
│   ├── orchard_specialized_planner.hpp # 과수원 특화 기능
│   └── learning_based_explorer.hpp # 학습 기반 탐사
├── src/                          # 소스 코드
│   ├── orbit_planner_node.cpp     # 메인 플래너 구현
│   ├── orbit_voxblox_interface.cpp # Voxblox 인터페이스 구현
│   └── orbit_panel_plugin.cpp     # RViz2 패널 구현
├── launch/                       # 런치 파일
│   └── orbit_exploration.launch.py
├── config/                       # 설정 파일
│   └── orbit_planner_params.yaml
├── rviz/                         # RViz 설정
│   ├── orbit_planner.rviz
│   └── orbit_panel_plugin.xml
├── test/                         # 테스트 코드
│   └── test_orbit_planner.cpp
└── README.md                     # 패키지 문서
```

### 핵심 클래스 및 모듈

#### 1. OrbitPlannerNode (메인 플래너 노드)
```cpp
class OrbitPlannerNode : public rclcpp::Node
{
public:
    // 주요 메서드
    void startExploration(const geometry_msgs::msg::PolygonStamped & polygon,
                         const geometry_msgs::msg::PoseStamped & start_pose);
    void stopExploration();
    bool isExplorationActive() const;
    
private:
    // 탐사 로직
    void explorationLoop();                    // 메인 탐사 루프
    std::vector<FrontierCandidate> generateFrontierCandidates(); // 프론티어 생성
    FrontierCandidate selectBestGoal(const std::vector<FrontierCandidate> & candidates); // 목표 선택
    PathResult planPath(const geometry_msgs::msg::PoseStamped & goal); // 경로 계획
    
    // ROS2 컴포넌트
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_exploration_srv_;
    
    // Voxblox 인터페이스
    std::shared_ptr<OrbitVoxbloxInterface> voxblox_interface_;
};
```

**역할**: 
- 전체 탐사 시스템의 중앙 제어
- 프론티어 감지 및 목표 선택
- 경로 계획 및 실행
- ROS2 토픽/서비스 관리

#### 2. OrbitVoxbloxInterface (Voxblox 통합)
```cpp
class OrbitVoxbloxInterface : public rclcpp::Node
{
public:
    // 주요 메서드
    bool isFree(const geometry_msgs::msg::Point & point) const;
    double getDistance(const geometry_msgs::msg::Point & point) const;
    nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;
    
private:
    // 맵 관리
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void updateMap();
    void publishOccupancyGrid();
    
    // 내부 데이터 구조
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map_;
    std::vector<std::vector<std::vector<double>>> distance_field_;
    std::vector<std::vector<std::vector<int>>> occupancy_grid_3d_;
};
```

**역할**:
- LIO-SAM 포인트 클라우드를 TSDF/ESDF로 변환
- 실시간 거리 필드 계산
- 충돌 검사 및 안전성 확인
- 2D 점유 격자 생성

#### 3. OrbitPanelPlugin (RViz2 GUI)
```cpp
class OrbitPanelPlugin : public rviz_common::Panel
{
    Q_OBJECT
    
public:
    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config & config) override;
    
private slots:
    void onSelectStartPoint();        // 시작점 선택
    void onTogglePolygonMode();       // 다각형 모드 토글
    void onStartExploration();        // 탐사 시작
    void onStopExploration();         // 탐사 중지
    
private:
    // UI 컴포넌트
    QPushButton * select_start_btn_;
    QPushButton * start_exploration_btn_;
    QListWidget * polygon_points_list_;
    
    // ROS2 통신
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr start_exploration_client_;
};
```

**역할**:
- 사용자 인터페이스 제공
- 탐사 영역 정의 및 시작점 설정
- 실시간 상태 모니터링
- 매개변수 조정

### 데이터 구조

#### 1. FrontierCandidate (프론티어 후보)
```cpp
struct FrontierCandidate
{
    geometry_msgs::msg::Point position;    // 위치
    double information_gain;               // 정보 획득량
    double travel_cost;                    // 이동 비용
    double total_utility;                  // 총 효용
    bool visited;                          // 방문 여부
    int cluster_id;                        // 클러스터 ID
};
```

#### 2. PathResult (경로 계획 결과)
```cpp
struct PathResult
{
    nav_msgs::msg::Path path;              // 계획된 경로
    bool success;                          // 성공 여부
    double path_length;                    // 경로 길이
    double clearance;                      // 안전 여유
};
```

### 주요 알고리즘

#### 1. 프론티어 감지 알고리즘
```cpp
std::vector<FrontierCandidate> OrbitPlannerNode::generateFrontierCandidates()
{
    std::vector<FrontierCandidate> candidates;
    
    // 1. 점유 격자에서 프론티어 셀 찾기
    for (int x = 1; x < grid_width - 1; ++x) {
        for (int y = 1; y < grid_height - 1; ++y) {
            // 현재 셀이 자유 공간이고
            if (occupancy_grid.data[index] == 0) {
                // 인접 셀 중 미지 영역이 있는지 확인
                bool has_unknown_neighbor = false;
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (occupancy_grid.data[neighbor_index] == -1) {
                            has_unknown_neighbor = true;
                            break;
                        }
                    }
                }
                
                if (has_unknown_neighbor) {
                    // 프론티어 후보 생성
                    FrontierCandidate candidate;
                    candidate.position = world_point;
                    candidate.information_gain = calculateInformationGain(candidate);
                    candidate.travel_cost = calculateTravelCost(candidate);
                    candidates.push_back(candidate);
                }
            }
        }
    }
    
    return candidates;
}
```

#### 2. 목표 선택 알고리즘
```cpp
FrontierCandidate OrbitPlannerNode::selectBestGoal(const std::vector<FrontierCandidate> & candidates)
{
    // 효용 함수: U(g) = w_f * information_gain - w_d * travel_cost
    for (auto & candidate : candidates) {
        candidate.total_utility = frontier_gain_weight_ * candidate.information_gain - 
                                 distance_weight_ * candidate.travel_cost;
    }
    
    // 최고 효용을 가진 후보 선택
    auto best_candidate = std::max_element(candidates.begin(), candidates.end(),
        [](const FrontierCandidate & a, const FrontierCandidate & b) {
            return a.total_utility < b.total_utility;
        });
    
    return *best_candidate;
}
```

#### 3. 경로 계획 알고리즘 (A*)
```cpp
PathResult OrbitPlannerNode::planPath(const geometry_msgs::msg::PoseStamped & goal)
{
    PathResult result;
    
    // 1. 시작점과 목표점 사이의 직접 경로 검사
    geometry_msgs::msg::Point start = current_pose_.pose.position;
    geometry_msgs::msg::Point end = goal.pose.position;
    
    // 2. 경로를 여러 점으로 샘플링하여 충돌 검사
    int num_samples = static_cast<int>(std::sqrt(
        std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2)) / 0.5) + 1;
    
    bool collision_free = true;
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        geometry_msgs::msg::Point point;
        point.x = start.x + t * (end.x - start.x);
        point.y = start.y + t * (end.y - start.y);
        
        // ESDF를 사용한 충돌 검사
        if (!voxblox_interface_->isFree(point)) {
            collision_free = false;
            break;
        }
    }
    
    // 3. 충돌이 없으면 경로 생성
    if (collision_free) {
        result.success = true;
        // 경로 포인트 생성...
    }
    
    return result;
}
```

### ROS2 통신 구조

#### 토픽 (Topics)
```yaml
입력 토픽:
  /lio_sam/mapping/cloudRegistered: sensor_msgs/PointCloud2  # LIO-SAM 포인트 클라우드
  /clicked_point: geometry_msgs/PointStamped                 # RViz에서 클릭한 점

출력 토픽:
  /orbit_planner/trajectory: nav_msgs/Path                   # 계획된 경로
  /orbit_planner/goal: geometry_msgs/PoseStamped             # 현재 목표
  /orbit_planner/markers: visualization_msgs/MarkerArray     # 시각화 마커
  /orbit_planner/occupancy_grid: nav_msgs/OccupancyGrid      # 점유 격자
  /orbit_planner/exploration_polygon: geometry_msgs/PolygonStamped # 탐사 영역
```

#### 서비스 (Services)
```yaml
서비스:
  /orbit_planner/start_exploration: std_srvs/srv/Empty       # 탐사 시작
  /orbit_planner/stop_exploration: std_srvs/srv/Empty        # 탐사 중지
```

### 설정 파일 구조

#### orbit_planner_params.yaml
```yaml
# Voxblox 인터페이스 설정
voxblox_interface:
  voxel_size: 0.1                    # 복셀 크기 (미터)
  truncation_distance: 0.2           # TSDF 절단 거리
  esdf_max_distance: 2.0             # 최대 ESDF 거리
  robot_radius: 0.4                  # 로봇 반지름
  update_rate: 2.0                   # 맵 업데이트 주기 (Hz)

# 플래너 노드 설정
planner_node:
  robot_radius: 0.4                  # 로봇 반지름 (미터)
  goal_tolerance: 1.0                # 목표 허용 오차 (미터)
  frontier_cluster_min_size: 5       # 최소 프론티어 클러스터 크기
  exploration_rate: 1.0              # 탐사 주기 (Hz)
  max_planning_distance: 50.0        # 최대 계획 거리 (미터)

# 고급 경로 계획 설정
advanced_path_planning:
  algorithm_type: "A_STAR"           # A_STAR, RRT_STAR, VORONOI, HYBRID
  enable_smoothing: true             # 경로 스무딩 활성화
  max_planning_time: 5.0             # 최대 계획 시간 (초)

# 과수원 특화 설정
orchard_specialized:
  enable_tree_detection: true        # 나무 감지 활성화
  enable_row_detection: true         # 열 감지 활성화
  expected_row_spacing: 3.0          # 예상 열 간격 (미터)
  exploration_pattern: "ADAPTIVE"    # 탐사 패턴
```

### 실행 흐름

#### 1. 초기화 단계
```cpp
// 1. 노드 생성 및 매개변수 로드
OrbitPlannerNode::OrbitPlannerNode()
{
    // 매개변수 선언 및 로드
    this->declare_parameter("robot_radius", 0.4);
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    
    // ROS2 컴포넌트 초기화
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orbit_planner/trajectory", 10);
    start_exploration_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/orbit_planner/start_exploration",
        std::bind(&OrbitPlannerNode::startExplorationCallback, this, ...));
    
    // Voxblox 인터페이스 초기화
    voxblox_interface_ = std::make_shared<OrbitVoxbloxInterface>();
    
    // 탐사 타이머 시작
    exploration_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / exploration_rate_),
        std::bind(&OrbitPlannerNode::explorationLoop, this));
}
```

#### 2. 탐사 시작
```cpp
void OrbitPlannerNode::startExplorationCallback(...)
{
    // 1. 탐사 영역 및 시작점 설정
    exploration_polygon_ = polygon;
    start_pose_ = start_pose;
    exploration_active_ = true;
    
    // 2. 방문 기록 초기화
    visited_positions_.clear();
    visited_cluster_ids_.clear();
    
    RCLCPP_INFO(this->get_logger(), "Exploration started");
}
```

#### 3. 메인 탐사 루프
```cpp
void OrbitPlannerNode::explorationLoop()
{
    if (!exploration_active_) return;
    
    // 1. 로봇 위치 업데이트
    if (!updateRobotPose()) return;
    
    // 2. 현재 목표 도달 확인
    if (isGoalReached(current_goal_)) {
        markAreaAsVisited(current_goal_.pose.position, visited_radius_);
    }
    
    // 3. 프론티어 후보 생성
    std::vector<FrontierCandidate> candidates = generateFrontierCandidates();
    if (candidates.empty()) {
        stopExploration();
        return;
    }
    
    // 4. 프론티어 클러스터링
    candidates = clusterFrontiers(candidates);
    
    // 5. 최적 목표 선택
    FrontierCandidate best_goal = selectBestGoal(candidates);
    
    // 6. 경로 계획
    PathResult path_result = planPath(goal_pose);
    
    // 7. 결과 발행
    publishTrajectory(path_result.path);
    publishGoal(goal_pose);
    publishVisualizationMarkers(candidates, best_goal);
}
```

### 확장 가능한 아키텍처

#### 1. 새로운 경로 계획 알고리즘 추가
```cpp
// advanced_path_planner.hpp에 새로운 알고리즘 추가
enum class AlgorithmType
{
    A_STAR,
    RRT_STAR,
    VORONOI,
    HYBRID,
    YOUR_NEW_ALGORITHM  // 새로운 알고리즘 추가
};

// 해당 알고리즘 구현
PlanningResult planYourNewAlgorithm(...);
```

#### 2. 새로운 탐사 전략 추가
```cpp
// orchard_specialized_planner.hpp에 새로운 패턴 추가
enum class ExplorationPattern
{
    ROW_BY_ROW,
    SPIRAL,
    ZIGZAG,
    ADAPTIVE,
    CUSTOM,
    YOUR_NEW_PATTERN  // 새로운 패턴 추가
};
```

#### 3. 새로운 센서 통합
```cpp
// orbit_voxblox_interface.hpp에 새로운 센서 추가
void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
```

### 성능 최적화 기법

#### 1. 멀티스레딩
```cpp
// 별도 스레드에서 경로 계획 실행
std::thread planning_thread_([this]() {
    while (planning_active_) {
        // 경로 계획 로직
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
});
```

#### 2. 캐싱 시스템
```cpp
// 프론티어 결과 캐싱
std::unordered_map<std::string, std::vector<FrontierCandidate>> frontier_cache_;
std::chrono::steady_clock::time_point last_frontier_update_;
```

#### 3. 적응형 업데이트율
```cpp
// 시스템 부하에 따른 업데이트율 조정
if (planning_time > planning_time_threshold_) {
    adaptive_exploration_rate_ *= 0.9;  // 업데이트율 감소
}
```

이 문서는 orbit_planner 패키지의 코드 구조를 처음 접하는 개발자가 전체 시스템을 이해하고 확장할 수 있도록 상세히 설명합니다.

## Troubleshooting
- GUI/X11: Ensure `$DISPLAY` is set on the host and run `xhost +local:`. Wayland sessions may need Xorg fallback.
- Permissions: Rebuild with your host UID/GID exported before `docker compose build`.
- Networking: The container uses host networking for DDS discovery. Check local firewall rules if discovery fails.

## Development Container Files
- `Dockerfile`: Ubuntu 22.04 + ROS 2 Humble desktop, dev tools, non-root user, ROS env.
- `entrypoint.sh`: Sources ROS, initializes/updates rosdep, sets GUI env, overlays `/workspace` if built.
- `docker-compose.yml`: Host networking, X11 passthrough, UID/GID mapping, workspace mount.
- `.dockerignore`: Minimizes build context.

## License
This repository aggregates multiple packages with different licenses. Refer to each subpackage's `package.xml` and `LICENSE` (where present). `robot_localization` is Apache 2.0; LIO-SAM is originally by Tixiao Shan (see upstream license and attribution).

## Acknowledgements
- LIO-SAM by Tixiao Shan and contributors.
- robot_localization by Tom Moore, Steve Macenski, and contributors.
- Voxblox by Helen Oleynikova and contributors (ETH Zürich).
- Orbit Planner inspired by frontier-based exploration and orchard-specific planning research.
