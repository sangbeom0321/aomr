# kimm_orchard
kimm_orchard (2023)

# Dependencies
```
sudo apt-get install ros-humble-gazebo-ros-pkgs

sudo apt install python3-rosdep2

rosdep update

cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install ros-humble-ros2-control

sudo apt install ros-humble-ros2-controllers

sudo apt-get install ros-humble-joint-state-publisher-gui

sudo apt-get install ros-humble-rqt-robot-steering

sudo apt install ros-humble-tf2-tools ros-humble-tf-transformations

sudo apt install ros-humble-gazebo-*

sudo apt install ros-humble-velodyne-gazebo-plugins

sudo apt install python3-pip

pip install transforms3d

sudo apt install ros-humble-velodyne-gazebo-plugins
```

# First setup

```
cd ~/ros2_ws/src
git clone <package>
cd ~/ros2_ws/src/kimm_orchard/map/urdf
xacro orchard_geometry.urdf.xacro  > orchard_geometry.urdf
cd ~/ros2_ws && colcon build --symlink-install
```
# Docker setup

```
cd ~/ros2_ws/src/docker
./create_docker.sh
# in docker
cd /tmp
./install.sh
```

# gazebo simulation 실행
```
ros2 launch kimm_orchard gazebo.launch.py
```

# joint_state_publisher_gui 실행
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

# tf pdf 생성
```
ros2 run tf2_tools view_frames
```

# Ranger Control
```
ros2 run kimm_orchard keyboard_teleop.py
```
### Keyboard

1. **In-Phase mode** 

**w/W**(straight), **s/S**(back), **QAZCDE**(Other Phase) of 8 direction

2. **Opposite phase**

**q**(turn left && straight), **e**(turn right && straight), **z, c**

3. **Pivot turn**

**a or d** (Rotate in place)
