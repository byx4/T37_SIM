#!/usr/bin/env bash
set -e
pushd "$(dirname "$0")/../ws" >/dev/null
source install/setup.bash
ros2 launch sim_core sim_launch.py
popd >/dev/null
