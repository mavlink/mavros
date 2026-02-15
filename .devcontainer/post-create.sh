#!/usr/bin/env bash
set -euo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"

WS_ROOT="/ws"
ROSINSTALL_FILE="${WS_ROOT}/src/mavros/dependencies.rosinstall"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi

rosdep update || true

if [ -f "${ROSINSTALL_FILE}" ]; then
  vcs import --input "${ROSINSTALL_FILE}" --skip-existing "${WS_ROOT}/src"
fi

pushd "${WS_ROOT}" >/dev/null
rosdep install -y -i --from-paths src --rosdistro "${ROS_DISTRO}"
popd >/dev/null

echo "Devcontainer ready."
echo "Workspace dependencies are installed."
