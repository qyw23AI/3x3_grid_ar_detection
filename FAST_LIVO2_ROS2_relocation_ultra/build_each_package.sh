#!/bin/bash

set -euo pipefail

readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

# usage: ./build_each_package.sh <ROS2|humble> [parallel_workers]
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <ROS2|humble> [parallel_workers]"
  exit 1
fi

ARG_ROS="$1"
PARALLEL_WORKERS="${2:-}"

ROS_HUMBLE=""
ROS_VERSION=""
if [ "$ARG_ROS" = "ROS2" ]; then
  ROS_VERSION=${VERSION_ROS2}
elif [ "$ARG_ROS" = "humble" ]; then
  ROS_VERSION=${VERSION_ROS2}
  ROS_HUMBLE=${VERSION_HUMBLE}
else
  echo "Invalid ROS arg: $ARG_ROS. Use 'ROS2' or 'humble'"
  exit 1
fi

echo "ROS version: ${ROS_VERSION}  HUMBLE flag: ${ROS_HUMBLE}"

# ensure sourced environment (user may want to change this path)
if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/humble/setup.bash || true
fi

ROOT_DIR=$(pwd)
if [ -d "src" ]; then
  cd "${ROOT_DIR}"
else
  echo "This script should be run from the workspace root (containing src/)"
  exit 1
fi

echo "Listing packages under src/..."
pkg_list=$(colcon list --base-paths src | awk '{print $1}')
if [ -z "$pkg_list" ]; then
  echo "No packages found in src/. Check your workspace." >&2
  exit 1
fi

for pkg in $pkg_list; do
  echo
  echo "=== Building package: $pkg ==="
  if [ -n "$PARALLEL_WORKERS" ]; then
    colcon build --packages-select "$pkg" --parallel-workers "$PARALLEL_WORKERS" --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
  else
    colcon build --packages-select "$pkg" --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
  fi
done

echo
echo "All done. To use the built workspace: source install/setup.bash"
