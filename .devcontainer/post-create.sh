#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
export PATH="/root/.local/bin:${PATH}"

WS_ROOT="/ws"
ROSINSTALL_FILE="${WS_ROOT}/src/mavros/dependencies.rosinstall"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi

rosdep update || true

if [ -f "${ROSINSTALL_FILE}" ]; then
  vcs import --input "${ROSINSTALL_FILE}" --skip-existing "${WS_ROOT}/src"
fi

if ! command -v mr-cog >/dev/null 2>&1; then
  uv tool install "${WS_ROOT}/src/mavros/tools"
fi

pushd "${WS_ROOT}" >/dev/null
apt-get update
rosdep install -y -i --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}"
popd >/dev/null

echo "Devcontainer ready."
echo "Workspace dependencies and mavros tools are installed."
