#!/usr/bin/env bash
set -euo pipefail

# Source ROS 2 for every shell
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# If a workspace is bind-mounted at /root/ws and already built, source it
if [ -f "/root/ws/install/setup.bash" ]; then
  source "/root/ws/install/setup.bash"
fi

# Optional convenience: auto-build if requested and not yet built
# Usage: docker run -e AUTO_BUILD=1 ...
if [[ "${AUTO_BUILD:-0}" == "1" ]]; then
  if [ -d "/root/ws/src" ] && [ ! -f "/root/ws/install/setup.bash" ]; then
    echo "[entrypoint] AUTO_BUILD=1 -> running colcon build..."
    pushd /root/ws >/dev/null
    colcon build --symlink-install
    source install/setup.bash
    popd >/dev/null
  fi
fi

# Pass control to the container's CMD
exec "$@"
