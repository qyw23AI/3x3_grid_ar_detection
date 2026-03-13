#!/usr/bin/env bash
set -euo pipefail

# 项目路径配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAIN_WORKSPACE="/home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra"
AR_WORKSPACE="${SCRIPT_DIR}"
LOG_DIR="${SCRIPT_DIR}/log/one_click"
mkdir -p "${LOG_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"

echo "[INFO] Script directory: ${SCRIPT_DIR}"
echo "[INFO] Main workspace: ${MAIN_WORKSPACE}"
echo "[INFO] AR workspace: ${AR_WORKSPACE}"

if command -v conda >/dev/null 2>&1; then
  conda deactivate >/dev/null 2>&1 || true
fi

# Source ROS2 基础环境
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  echo "[OK] ROS2 Humble sourced"
else
  echo "[ERROR] /opt/ros/humble/setup.bash not found"
  exit 1
fi

# Source 主工作空间 (FAST-LIVO2)
if [[ -f "${MAIN_WORKSPACE}/install/setup.bash" ]]; then
  set +u
  source "${MAIN_WORKSPACE}/install/setup.bash"
  set -u
  echo "[OK] Main workspace sourced: ${MAIN_WORKSPACE}"
else
  echo "[ERROR] ${MAIN_WORKSPACE}/install/setup.bash not found"
  echo "[ERROR] Please build the main workspace first:"
  echo "  cd ${MAIN_WORKSPACE} && colcon build"
  exit 1
fi

# Source AR 工作空间 (可选，如果存在)
if [[ -f "${AR_WORKSPACE}/install/setup.bash" ]]; then
  set +u
  source "${AR_WORKSPACE}/install/setup.bash"
  set -u
  echo "[OK] AR workspace sourced: ${AR_WORKSPACE}"
else
  echo "[WARN] AR workspace not built, AR overlay will not be available"
  echo "       To build: cd ${AR_WORKSPACE} && colcon build"
fi


PIDS=()
ENABLE_CAMERA_DRIVER="${ENABLE_CAMERA_DRIVER:-1}"
ENABLE_AR_OVERLAY="${ENABLE_AR_OVERLAY:-1}"
ENABLE_RELOCALIZATION="${ENABLE_RELOCALIZATION:-1}"
CAMERA_LAUNCH_CMD="${CAMERA_LAUNCH_CMD:-ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false rgb_camera.color_profile:=640x480x30}"

start_bg() {
  local name="$1"
  local cmd="$2"
  local log_file="${LOG_DIR}/${TS}_${name}.log"
  echo "[START] ${name}: ${cmd}"
  # 在子 shell 中重新 source 所有环境，确保路径正确
  bash -lc "set -e; \
    if command -v conda >/dev/null 2>&1; then conda deactivate >/dev/null 2>&1 || true; fi; \
    set +u; \
    source /opt/ros/humble/setup.bash; \
    source '${MAIN_WORKSPACE}/install/setup.bash'; \
    [[ -f '${AR_WORKSPACE}/install/setup.bash' ]] && source '${AR_WORKSPACE}/install/setup.bash'; \
    set -u; \
    ${cmd}" >"${log_file}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  echo "[PID] ${name}: ${pid}"
  echo "[LOG] ${name}: ${log_file}"
}

cleanup() {
  echo ""
  echo "[INFO] Stopping launched processes..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  echo "[INFO] All processes stopped"
}
trap cleanup EXIT INT TERM

echo ""
echo "=========================================="
echo "  AR 定位九宫格系统一键启动"
echo "=========================================="
echo "[INFO] Log directory: ${LOG_DIR}"
echo "[INFO] Timestamp: ${TS}"
echo ""

# 1. 启动相机驱动 (可选)
if [[ "${ENABLE_CAMERA_DRIVER}" == "1" ]]; then
  echo "[STEP 1/6] Starting RealSense camera..."
  start_bg "realsense_camera" "${CAMERA_LAUNCH_CMD}"
  sleep 3
  echo "  ✓ Camera driver started"
else
  echo "[STEP 1/6] Camera driver disabled (set ENABLE_CAMERA_DRIVER=1 to enable)"
fi

# 2. 启动 Livox 雷达驱动
echo "[STEP 2/6] Starting Livox MID360 driver..."
start_bg "livox_driver" "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
sleep 3
echo "  ✓ Livox driver started"

# 3. 启动 FAST-LIVO2 里程计 (带或不带重定位配置)
echo "[STEP 3/6] Starting FAST-LIVO2 odometry..."
if [[ "${ENABLE_RELOCALIZATION}" == "1" ]]; then
  # 使用重定位配置启动 FAST-LIVO2
  # 注意：example_teaser_gicp.launch.py 会启动 FAST-LIVO2
  echo "  Note: FAST-LIVO2 will be started by relocalization launch file"
else
  # 纯建图模式
  start_bg "fast_livo" "ros2 launch fast_livo mapping_avia.launch.py use_rviz:=True"
  sleep 5
  echo "  ✓ FAST-LIVO2 started (mapping mode)"
fi

# 4. 启动 TEASER++GICP 重定位 (可选)
if [[ "${ENABLE_RELOCALIZATION}" == "1" ]]; then
  echo "[STEP 4/6] Starting TEASER++GICP relocalization..."
  start_bg "teaser_gicp" "cd '${MAIN_WORKSPACE}' && ros2 launch example_teaser_gicp.launch.py"
  sleep 5
  echo "  ✓ Relocalization module started (includes FAST-LIVO2)"
else
  echo "[STEP 4/6] Relocalization disabled (set ENABLE_RELOCALIZATION=1 to enable)"
fi

# 5. 启动 AR 九宫格叠加 (可选)
if [[ "${ENABLE_AR_OVERLAY}" == "1" ]]; then
  echo "[STEP 5/6] Starting AR overlay node..."
  if [[ -f "${AR_WORKSPACE}/ar_overlay.launch.py" ]]; then
    start_bg "ar_overlay" "ros2 launch '${AR_WORKSPACE}/ar_overlay.launch.py' params_file:='${AR_WORKSPACE}/ar_overlay.params.yaml'"
    sleep 2
    echo "  ✓ AR overlay started"
  else
    echo "  ✗ AR overlay launch file not found at ${AR_WORKSPACE}/ar_overlay.launch.py"
  fi
else
  echo "[STEP 5/6] AR overlay disabled (set ENABLE_AR_OVERLAY=1 to enable)"
fi

# 6. 启动位置偏移发布器 (如果需要)
# 这个节点用于发布位置偏移，可根据需要启用
# echo "[STEP 6/6] Starting position offset publisher..."
# start_bg "offset_publisher" "ros2 run fast_livo position_offset_publisher"
# sleep 2
# echo "  ✓ Offset publisher started"
echo "[STEP 6/6] Position offset publisher (optional, currently disabled)"
echo ""

echo "=========================================="
echo "[INFO] All components launched successfully!"
echo "=========================================="
echo ""
echo "System Status:"
echo "  • Camera driver:      $( [[ "${ENABLE_CAMERA_DRIVER}" == "1" ]] && echo "ENABLED" || echo "DISABLED" )"
echo "  • Livox driver:       ENABLED"
echo "  • FAST-LIVO2:         ENABLED"
echo "  • Relocalization:     $( [[ "${ENABLE_RELOCALIZATION}" == "1" ]] && echo "ENABLED" || echo "DISABLED" )"
echo "  • AR overlay:         $( [[ "${ENABLE_AR_OVERLAY}" == "1" ]] && echo "ENABLED" || echo "DISABLED" )"
echo ""
echo "Checking ROS2 topics..."
sleep 3

# 显示关键话题状态
echo ""
echo "Key topics status:"
for topic in "/livox/lidar" "/livox/imu" "/aft_mapped_to_init" "/camera/camera/color/image_raw"; do
  if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
    echo "  ✓ ${topic}"
  else
    echo "  ✗ ${topic} (not available)"
  fi
done

echo ""
echo "=========================================="
echo "  System is running!"
echo "=========================================="
echo ""
echo "Tips:"
echo "  • View logs in: ${LOG_DIR}/"
echo "  • View topics: ros2 topic list"
echo "  • View transforms: ros2 run tf2_tools view_frames"
echo "  • Monitor odometry: ros2 topic echo /aft_mapped_to_init"
if [[ "${ENABLE_AR_OVERLAY}" == "1" ]]; then
  echo "  • View AR output: ros2 run rqt_image_view rqt_image_view /ar_overlay/image"
fi
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# 保持脚本运行，监控子进程
while true; do
  sleep 5
  # 检查是否有进程意外退出
  for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
      echo "[WARN] Process ${PIDS[$i]} has died unexpectedly"
    fi
  done
done
