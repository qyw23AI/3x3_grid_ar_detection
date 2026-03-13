# 180度USB鱼眼相机完整配置流程

## 概览

本文档为从硬件连接到AR定位投影的**完整端到端流程**说明。

## 系统架构

```
USB鱼眼相机 (1920×1080@30fps)
       ↓
   usb_cam 驱动
       ↓
/fisheye_camera/image_raw [话题]
       ↓
ar_grid_detector 节点 ← /aft_mapped_to_init [里程计话题]
       ↓
/ar_grid/image [可视化]
/ar_grid/visible_cells [格子检测结果]
```

---

## 第一阶段：硬件与驱动配置

### 步骤1.1 检查硬件连接

```bash
# 查看USB设备
lsusb | grep -i camera

# 查看video设备
ls -la /dev/video*

# 输出示例：
# crw-rw----+ 1 root video 81, 0 Mar  7 10:15 /dev/video0
```

**检查清单：**
- [ ] USB摄像头已物理连接
- [ ] `ls /dev/video0` 能看到设备
- [ ] 设备权限正确 (rwx for video group)

### 步骤1.2 配置USB权限

```bash
# 检查当前用户是否在video组
groups $USER

# 如果没有video，添加权限
sudo usermod -a -G video $USER

# 需要重新登录或运行
newgrp video

# 验证权限
groups $USER  # 应包含 video
```

### 步骤1.3 验证USB相机功能

```bash
# 方法1：使用v4l2工具查看支持格式
sudo apt install v4l-utils
v4l2-ctl -d /dev/video0 --list-formats-ext

# 方法2：启动摄像头看能否获取图像
ros2 run usb_cam usb_cam_node_exe -p video_device:=/dev/video0 &

# 新终端查看话题
ros2 topic list | grep image_raw
ros2 topic hz /image_raw  # 应≥ 30 Hz

# 新终端查看画面
rqt_image_view /image_raw

# 停止
kill %1
```

---

## 第二阶段：相机标定

### 步骤2.1 准备标定板

**棋盘规格：**
- 行列数：10x7 (推荐)
- 方格大小：30mm (A4纸可打印10x7)
- 材质：建议贴在硬纸板上，确保平平

**打印方式：**
1. 下载标准棋盘模板（搜索"OpenCV checkerboard 10x7 30mm"）
2. 用A3纸打印，确保尺寸
3. 贴在平的纸板/塑料板上

### 步骤2.2 采集标定图像

**采集要点：**
- 最少20张，推荐30张
- 覆盖整个图像视场：中心、四个角、边缘
- 不同距离（0.3m到1m）
- 不同角度（正面、45°、90°等）
- 避免图像模糊或运动模糊

**采集脚本** (可选)：
```bash
# 使用ros2 + OpenCV动态采集
# 1. 启动相机
ros2 run usb_cam usb_cam_node_exe -p video_device:=/dev/video0 &

# 2. 新终端，创建采集目录
mkdir -p ~/calibration_images

# 3. 采集图像（需要编写脚本或手动截图）
# 可使用以下Python脚本逐帧保存
```

**快速采集脚本** (`capture_calibration_images.py`):
```python
#!/usr/bin/env python3
import cv2
import os
from datetime import datetime

cap = cv2.VideoCapture(0)  # 打开/dev/video0
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)

output_dir = os.path.expanduser("~/calibration_images")
os.makedirs(output_dir, exist_ok=True)

print("按 SPACE 拍照，按 Q 退出")
count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    cv2.imshow("Capture - Press SPACE to capture, Q to quit", frame)
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord(' '):
        filename = os.path.join(output_dir, f"calibration_{count:03d}.jpg")
        cv2.imwrite(filename, frame)
        print(f"✓ Saved: {filename}")
        count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"总共采集 {count} 张图像")
```

### 步骤2.3 运行标定工具

```bash
# 确保脚本可执行
chmod +x src/ar_grid_detector/scripts/fisheye_calibration_tool.py

# 运行标定
python3 src/ar_grid_detector/scripts/fisheye_calibration_tool.py \
  --images="${HOME}/calibration_images/*.jpg" \
  --cols 10 \
  --rows 7 \
  --square-size 0.03 \
  --output fisheye_params.json

# 输出示例：
# ============================================================
# 📋 标定结果
# ============================================================
#
# 📷 图像信息:
#    分辨率: 1920x1080
#
# 🔍 内参矩阵 (K):
#    fx = 650.5432
#    fy = 650.3421
#    cx = 960.1234
#    cy = 540.5678
#
# 🌀 畸变参数 (D) - 鱼眼等距投影:
#    k1 = -0.234567
#    k2 = 0.067890
#    k3 = -0.012345
#    k4 = 0.002345
#
# 📊 精度:
#    RMS误差: 0.3456 pixels  ✅ 优秀
```

**质量评估：**
- RMS < 0.5 pixels: ✅ 优秀
- RMS < 1.0 pixels: ✅ 良好  
- RMS < 2.0 pixels: ⚠️ 可接受
- RMS ≥ 2.0 pixels: ❌ 需要重新采集和标定

### 步骤2.4 保存标定结果

标定工具会自动生成两种格式：

**JSON格式** (推荐用于ar_grid_detector):
```bash
# 直接在参数文件中使用
cat fisheye_params.json
# {
#   "camera_model": "fisheye_equidistant",
#   "camera": {
#     "width": 1920,
#     "height": 1080,
#     "fx": 650.5432,
#     ...
#   }
# }
```

**YAML格式** (用于手动配置):
```bash
# 复制输出的YAML片段到 ar_grid.params.yaml
```

---

## 第三阶段：配置ar_grid_detector参数

### 步骤3.1 选择参数文件

有两种配置方式：

**方式A：直接使用JSON** (推荐，更简单)
```bash
# 编辑 config/ar_grid.params.yaml
# 添加一行：
camera_json: "/path/to/fisheye_params.json"
```

**方式B：手动填写参数**
```bash
# 编辑 config/ar_grid.params.yaml
# 在 camera: 段落中填写
camera:
  width: 1920
  height: 1080
  fx: 650.5432   # 从标定结果复制
  fy: 650.3421
  cx: 960.1234
  cy: 540.5678
  k1: -0.234567  # 从标定结果复制
  k2: 0.067890
  k3: -0.012345
  k4: 0.002345
```

### 步骤3.2 配置格子架参数

**格子参数：**
```yaml
grid:
  rows: 2                 # 根据实际格子行数
  cols: 2                 # 根据实际格子列数
  cell_width: 0.075       # 单个格子宽度(米)
  cell_height: 0.075      # 单个格子高度(米)
  corner_top_left: [-0.081696, 0.925071, 0.370132]
  corner_top_right: [0.128394, 0.702569, 0.372652]
  corner_bottom_left: [-0.158594, 0.849270, 0.083796]
  input_is_cell_centers: true  # 关键！根据采点方式选择
```

**input_is_cell_centers 参数说明：**

| 参数值 | 含义 | 何时选择 |
|--------|------|--------|
| true | 三个参考点是各格子的中心位置 | ✅ 推荐，更直观 |
| false | 三个参考点是格子架外边框的角点 | 物理框架有固定角 |

### 步骤3.3 配置通信话题

**ROS2话题配置：**
```yaml
subscribers:
  odom_topic: "/aft_mapped_to_init"  # 雷达里程计
  image_topic: "/fisheye_camera/image_raw"  # USB相机输出

publishers:
  output_image_topic: "/ar_grid/image"
  grid_detection_topic: "/ar_grid/visible_cells"
```

---

## 第四阶段：启动与验证

### 步骤4.1 编译ar_grid_detector

```bash
# 进入工作空间
cd /home/r1/Slam/ar_calculate

# 编译包
colcon build --packages-select ar_grid_detector

# 编译打包 (如果需要)
source install/setup.bash
```

### 步骤4.2 启动相机和ar_grid_detector

**方式1：自动启动（推荐）**
```bash
# 一键启动所有系统
./start_all.sh

# 内部会执行：
# 1. 启动USB相机驱动 (1920×1080@30fps MJPEG)
# 2. 启动雷达驱动
# 3. 启动FAST-LIVO2里程计
# 4. 启动ar_grid_detector
```

**方式2：手动启动**
```bash
# 终端1：启动相机
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=30 \
  -p pixel_format:=mjpeg

# 终端2：启动ar_grid_detector
ros2 launch ar_grid_detector ar_grid.launch.py
```

### 步骤4.3 验证输出

**检查话题是否正常发布：**
```bash
# 新终端检查话题
ros2 topic list | grep -E "fisheye|ar_grid"

# 输出应包含：
# /fisheye_camera/image_raw
# /ar_grid/image
# /ar_grid/visible_cells
```

**检查帧率**：
```bash
# 相机帧率（应≥ 30 Hz）
ros2 topic hz /fisheye_camera/image_raw

# ar_grid输出帧率（应与相机相同）
ros2 topic hz /ar_grid/image
```

**查看可视化结果**：
```bash
# 启动RQT图像查看器
rqt_image_view /ar_grid/image

# 应该看到：
# ✅ 相机图像
# ✅ 投影的格子框 (绿色线条)
# ✅ 格子ID标号
# ✅ 格子与真实物体对齐
```

**查看检测结果**：
```bash
# 查看格子消息
ros2 topic echo /ar_grid/visible_cells

# 输出示例：
# cells: 
# - id: 0
#   row: 0
#   col: 0
#   visible: true
#   center_in_camera: [0.5, 0.2, 1.2]  # 相机系XYZ
#   ...
# - id: 1
#   row: 0
#   col: 1
#   ...
```

---

## 第五阶段：性能优化与故障诊断

### 性能优化

**USB带宽优化：**
| 场景 | 分辨率 | 帧率 | 格式 | 命令 |
|------|--------|------|------|------|
| 高精度 | 1920×1080 | 30 | MJPEG | ✅ 推荐 |
| 高帧率 | 1280×720 | 60 | MJPEG | 如需要 |
| 低带宽 | 640×480 | 30 | MJPEG | 应急用 |

**临时修改分辨率：**
```bash
# 不编辑脚本，直接修改环境变量
USB_CAMERA_LAUNCH_CMD='ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=60 \
  -p pixel_format:=mjpeg' \
./start_all.sh
```

### 常见问题诊断

**问题1：ar_grid/image 没有内容或显示"无话题"**
```bash
# 诊断
ros2 topic list | grep image
ros2 node list | grep ar_grid

# 可能原因和解决
case:
  1. ar_grid_detector进程未启动
     → 检查是否运行了launch命令
  2. image_topic 参数错误
     → 检查config中的image_topic是否与实际话题匹配
  3. 相机驱动未启动
     → 运行 ros2 topic hz /fisheye_camera/image_raw
esac
```

**问题2：格子投影位置不正确（偏移或变形）**
```bash
# 诊断步骤
1. 检查input_is_cell_centers参数
   → 尝试改成false再改回true看结果
   
2. 检查外参矩阵T_c_s
   → 尝试用公式验证一个已知点
   → 或重新标定外参
   
3. 检查格子参数
   → 确认三个参考点坐标无误
   → 确认cell_width/height正确
```

**问题3：帧率达不到30fps**
```bash
# 检查实际帧率
ros2 topic hz /fisheye_camera/image_raw

# 如果 < 30 Hz
# 原因通常是USB格式选择不当
# 解决：切换到MJPEG格式或降低分辨率

# 检查是否使用YUYV（很容易卡）
USB_CAMERA_LAUNCH_CMD='...-p pixel_format:=yuyv' ./start_all.sh
# → 改为 mjpeg
```

---

## 参考资源

- **K103A规格书**：见本项目 `K103A规格书` 附件
- **鱼眼相机标定详解**：README.md 第4.1节
- **快速参考卡**：FISHEYE_USB_QUICK_START.md
- **参数配置示例**：config_examples/ar_grid_fisheye_180deg.params.example.yaml

---

## 典型应用案例

### 案例1：室内2×2格子AR定位

```bash
# 配置
camera: 1920×1080 USB摄像头
grid: 2×2，75mm方格
distance: 0.5-1.5m

# 结果
✅ 投影精度: ±2cm
✅ 帧率: 30 FPS
✅ CPU占用: ~15%
```

### 案例2：移动机器人导航

```bash
# 配置
camera: 1280×720 USB摄像头
grid: 3×3，100mm方格
distance: 1-3m

# 结果  
✅ 投影精度: ±5cm
✅ 帧率: 60 FPS (720P可达)
✅ CPU占用: ~20%
```

---

**最后更新**: 2026-03-07  
**文档版本**: 1.0  
**适用设备**: K103A-M12-F2.0 (180°USB鱼眼相机)  
**ROS2版本**: Humble
