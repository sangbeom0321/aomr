# AOMR (ROS 2 Humble Workspace)

This repository is a ROS 2 Humble workspace for an autonomous mobile robot stack used in orchard environments (AOMR). It contains four-wheel steering control, LiDAR odometry and localization (LIO-SAM), waypoint/arm following utilities, simulation tools, and assorted helpers. A ready-to-use Docker environment (Ubuntu 22.04 + ROS 2 Humble) is provided for development with GUI support.

## Repository Layout (high level)
- `control/control_4ws/control_4ws`: Four-wheel steering control (C++), depends on `rclcpp`, `std_msgs`, `sensor_msgs`.
- `localization/LIO-SAM-ROS2-REFACTORING`: LIO-SAM port/refactor to ROS 2 (`lio_sam`), LiDAR Odometry using PCL, GTSAM, Eigen.
- `localization/LIO-SAM-LOC-ROS2-REFACTORING`: LIO-SAM localization-only variant (`lio_sam_loc`).
- `localization/robot_localization`: Third-party package for nonlinear state estimation via sensor fusion (Apache 2.0).
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

- Gazebo orchard simulation and teleop (from `utils/kimm_orchard`):
```bash
ros2 launch kimm_orchard gazebo.launch.py
ros2 run kimm_orchard keyboard_teleop.py
```

- Robot localization: see `localization/robot_localization/launch/` for ready-made launch files.

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
