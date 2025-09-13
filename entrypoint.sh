#!/usr/bin/env bash
set -e

# Source ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Initialize rosdep if needed, then update (best-effort)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo -E rosdep init || true
fi
rosdep update || true

# Source workspace overlay if present
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

# X11 defaults for GUI apps (e.g., rviz2, gedit, terminator)
export QT_X11_NO_MITSHM=1
if [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:0
fi

exec "$@"


