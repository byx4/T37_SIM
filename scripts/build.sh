#!/usr/bin/env bash
set -e
pushd "$(dirname "$0")/../ws" >/dev/null
colcon build --symlink-install
popd >/dev/null
