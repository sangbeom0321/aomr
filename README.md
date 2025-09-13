# ROS 2 Humble 개발 컨테이너 (Ubuntu 22.04)

Ubuntu 22.04 베이스에 ROS 2 Humble + 개발 도구(git, terminator, gedit, nano 등)를 포함한 Docker 환경입니다. `docker compose` 한 번으로 실행하며 X11 포워딩으로 GUI 앱(RViz2, gedit, terminator 등)도 사용할 수 있습니다.

## 포함 사항
- Ubuntu 22.04 + ROS 2 Humble (desktop)
- 기본 개발 도구: git, build-essential, cmake, nano, gedit, terminator, bash-completion 등
- colcon 확장: `python3-colcon-common-extensions`
- rosdep / vcstool 사전 설치
- 비루트 사용자 생성(기본: dev) + 호스트 UID/GID 매핑
- X11 GUI 지원(RViz2, gedit 등)

## 사전 준비
- Docker(=20.10+) 및 Docker Compose v2 설치
- X11 접근 허용 (한 번만 실행)

```bash
xhost +local:  # 보안 이슈가 있을 수 있으니 신뢰되는 로컬 사용자만 허용된 환경에서 사용하세요.
```

Wayland 세션에서도 X11 소켓(`/tmp/.X11-unix`)을 통해 동작합니다. 문제가 있으면 로그인 화면에서 Xorg 세션으로 변경해 테스트해 보세요.

## 빌드
환경 변수를 호스트의 UID/GID로 설정하면 컨테이너 안에서 생성되는 파일의 소유권이 올바르게 매핑됩니다.

```bash
export UID=$(id -u)
export GID=$(id -g)
# 옵션: 사용자명 지정 (기본: 현재 사용자명)
export USER=${USER}

# 이미지 빌드
docker compose build
# (또는 구버전 CLI)
# docker-compose build
```

## 실행
```bash
# 백그라운드 실행
docker compose up -d

# 셸 접속
docker compose exec ros2 bash

# (구버전)
# docker-compose up -d
# docker-compose exec ros2 bash
```

컨테이너는 host 네트워크/IPC를 사용하므로 ROS 2 DDS 디스커버리가 원활합니다. GUI 앱 실행을 위해 X11 소켓과 DISPLAY가 전달됩니다.

## 사용 예시
컨테이너 내부 셸에서 다음 명령을 실행해 보세요.

```bash
# ROS 2 토픽 확인
ros2 topic list

# 데모 노드 실행
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener &

# RViz2 실행 (GUI)
rviz2
```

작업공간은 컨테이너의 `/workspace`에 호스트의 현재 디렉토리가 마운트됩니다. 기존 ROS 2 워크스페이스가 있다면 빌드 후 `install/setup.bash`가 자동 소싱됩니다.

## 재빌드/정리
```bash
# Dockerfile 변경 반영
docker compose build --no-cache

# 컨테이너/네트워크 중지 및 삭제
docker compose down

# 이미지 삭제(원할 경우)
docker rmi aomr/ros2-humble:latest || true
```

## 문제 해결
- DISPLAY/X11 이슈: `echo $DISPLAY` 가 비어 있지 않은지 확인하고 `xhost +local:` 실행 여부를 점검하세요.
- 권한 이슈: `export UID=$(id -u); export GID=$(id -g)` 후 다시 `docker compose build` 하세요.
- 네트워킹 이슈: 컨테이너가 `network_mode: host` 를 사용하므로 방화벽 규칙을 확인하세요.

## 구성 파일
- `Dockerfile`: Ubuntu 22.04 + ROS 2 Humble + 개발 도구 설치, 비루트 사용자 생성
- `entrypoint.sh`: ROS/rosdep 초기화 및 워크스페이스/GUI 환경 설정
- `docker-compose.yml`: 호스트 UID/GID/X11/네트워킹 매핑 설정
- `.dockerignore`: 빌드 컨텍스트 최소화
