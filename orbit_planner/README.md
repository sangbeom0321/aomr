# Orbit Planner Package

ROS2 패키지로 과수원 환경에서의 자율 탐사를 위한 궤도 계획 시스템입니다.

## 개요

Orbit Planner는 Voxblox 매핑과 프론티어 기반 탐사를 사용하여 과수원 환경에서 자율적으로 탐사 경로를 생성하는 시스템입니다. 이 패키지는 LIO-SAM SLAM 시스템과 4륜 조향 제어 시스템과 통합되어 작동합니다.

## 주요 기능

### 핵심 기능
- **Voxblox 통합**: TSDF/ESDF 매핑을 통한 실시간 3D 환경 표현
- **프론티어 탐사**: 미지 영역의 경계를 찾아 효율적인 탐사 목표 생성
- **고급 경로 계획**: A*, RRT*, Voronoi 기반의 다중 알고리즘 경로 계획
- **RViz2 통합**: 사용자 친화적인 GUI를 통한 탐사 영역 정의 및 모니터링
- **실시간 성능**: 온보드 컴퓨터에서 실시간으로 작동

### 과수원 특화 기능
- **나무 감지**: 포인트 클라우드에서 개별 나무 식별 및 위치 추정
- **열 감지**: 과수원의 열 구조 자동 인식 및 분석
- **체계적 탐사**: 열별, 나선형, 지그재그 등 다양한 탐사 패턴 지원
- **적응형 계획**: 환경 구조에 따른 자동 탐사 전략 조정

### 고급 기능
- **학습 기반 탐사**: 강화학습을 통한 경험 기반 탐사 전략
- **성능 최적화**: 멀티스레딩, 캐싱, 적응형 업데이트율
- **실시간 모니터링**: 탐사 진행률, 성능 메트릭, 학습 상태 추적
- **확장 가능한 아키텍처**: 모듈화된 설계로 쉬운 기능 확장

## 시스템 아키텍처

### 주요 구성 요소

1. **orbit_planner_node**: 핵심 탐사 로직을 담당하는 메인 노드
2. **orbit_voxblox_interface**: Voxblox 라이브러리와의 인터페이스
3. **orbit_panel_plugin**: RViz2 패널 플러그인

### 데이터 흐름

```
LIO-SAM → Voxblox Interface → Planner Node → Control System
    ↓              ↓              ↓
Point Cloud → TSDF/ESDF → Frontier Detection → Path Planning
```

## 설치 및 빌드

### 의존성

- ROS2 Humble
- Voxblox (TSDF/ESDF 매핑)
- PCL (포인트 클라우드 처리)
- RViz2
- Eigen3

### 빌드

```bash
# 워크스페이스로 이동
cd your_ros2_workspace

# 패키지 빌드
colcon build --packages-select orbit_planner

# 환경 설정
source install/setup.bash
```

## 사용법

### 기본 실행

```bash
# 전체 시스템 실행
ros2 launch orbit_planner orbit_exploration.launch.py

# RViz 없이 실행
ros2 launch orbit_planner orbit_exploration.launch.py use_rviz:=false
```

### 개별 노드 실행

```bash
# Voxblox 인터페이스만 실행
ros2 run orbit_planner orbit_voxblox_interface

# 플래너 노드만 실행
ros2 run orbit_planner orbit_planner_node
```

### RViz2에서 패널 사용

1. RViz2 실행
2. 패널 추가: `Panels` → `Add New Panel` → `Orbit Planner Panel`
3. 탐사 영역 정의:
   - "Select Start Point" 버튼으로 시작점 선택
   - "Start Polygon Mode" 버튼으로 다각형 모드 활성화
   - 지도에서 클릭하여 탐사 영역 정의
4. "Start Exploration" 버튼으로 탐사 시작

## 설정

### 매개변수 설정

주요 매개변수는 `config/orbit_planner_params.yaml`에서 설정할 수 있습니다:

```yaml
# 로봇 반지름 (미터)
robot_radius: 0.4

# 목표 허용 오차 (미터)
goal_tolerance: 1.0

# 탐사 속도 (Hz)
exploration_rate: 1.0

# 최대 계획 거리 (미터)
max_planning_distance: 50.0
```

### 토픽 리매핑

다른 SLAM 시스템과 통합하려면 토픽을 리매핑하세요:

```bash
ros2 launch orbit_planner orbit_exploration.launch.py \
  --ros-args -r /lio_sam/mapping/cloudRegistered:=/your_pointcloud_topic
```

## API

### 서비스

- `/orbit_planner/start_exploration`: 탐사 시작
- `/orbit_planner/stop_exploration`: 탐사 중지

### 토픽

- `/orbit_planner/trajectory`: 계획된 경로 (nav_msgs/Path)
- `/orbit_planner/goal`: 현재 탐사 목표 (geometry_msgs/PoseStamped)
- `/orbit_planner/markers`: 시각화 마커 (visualization_msgs/MarkerArray)
- `/orbit_planner/occupancy_grid`: 점유 격자 (nav_msgs/OccupancyGrid)

## 개발자 가이드

### 코드 구조

```
orbit_planner/
├── src/                    # 소스 코드
│   ├── orbit_planner_node.cpp
│   ├── orbit_voxblox_interface.cpp
│   └── orbit_panel_plugin.cpp
├── include/orbit_planner/  # 헤더 파일
├── launch/                 # 런치 파일
├── config/                 # 설정 파일
└── rviz/                   # RViz 설정
```

### 새로운 기능 추가

1. 헤더 파일에 인터페이스 정의
2. 소스 파일에 구현
3. CMakeLists.txt에 빌드 설정 추가
4. 설정 파일에 매개변수 추가

## 문제 해결

### 일반적인 문제

1. **Voxblox 컴파일 오류**: Eigen3 의존성 확인
2. **RViz 플러그인 로드 실패**: 패키지 빌드 후 환경 설정 확인
3. **메모리 부족**: voxel_size 매개변수 조정

### 디버깅

```bash
# 로그 레벨 설정
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# 특정 노드 디버깅
ros2 run orbit_planner orbit_planner_node --ros-args --log-level debug
```

## 라이선스

MIT License

## 기여

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## 참고 문헌

- [Voxblox: Incremental 3D Euclidean Signed Distance Fields for on-board MAV planning](https://github.com/ethz-asl/voxblox)
- [LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping](https://github.com/TixiaoShan/LIO-SAM)
- [ROS2 Navigation Stack](https://github.com/ros-planning/navigation2)

## 연락처

개발팀: developer@example.com
