#!/usr/bin/env bash
set -e

# Source ROS
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace if built
if [ -f "/root/ws/install/setup.bash" ]; then
  source "/root/ws/install/setup.bash"
fi

exec "$@"
