# AR 定位九宫格系统一键启动脚本使用指南

## 脚本功能

`start_all_with_offset.sh` 是一个一键启动脚本，可以自动启动 AR 定位九宫格系统的所有组件。

## 快速开始

### 1. 基础启动（纯建图模式）

```bash
cd /home/r1/Slam/ar_calculate
./start_all_with_offset.sh
```

这将启动：
- ✅ RealSense 相机驱动
- ✅ Livox MID360 雷达驱动
- ✅ FAST-LIVO2 里程计（纯建图模式）
- ✅ TEASER++GICP 重定位模块

### 2. 禁用重定位（纯 SLAM 建图）

```bash
ENABLE_RELOCALIZATION=0 ./start_all_with_offset.sh
```

### 3. 启用 AR 九宫格叠加

```bash
ENABLE_AR_OVERLAY=1 ./start_all_with_offset.sh
```

### 4. 不启动相机

```bash
ENABLE_CAMERA_DRIVER=0 ./start_all_with_offset.sh
```

### 5. 完整功能（所有模块）

```bash
ENABLE_CAMERA_DRIVER=1 ENABLE_AR_OVERLAY=1 ENABLE_RELOCALIZATION=1 ./start_all_with_offset.sh
```

## 环境变量配置

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| `ENABLE_CAMERA_DRIVER` | `1` | 是否启动 RealSense 相机驱动 |
| `ENABLE_RELOCALIZATION` | `1` | 是否启用重定位功能 |
| `ENABLE_AR_OVERLAY` | `0` | 是否启动 AR 九宫格叠加 |
| `CAMERA_LAUNCH_CMD` | 见脚本 | 相机启动命令（可自定义） |

## 系统组件启动顺序

1. **相机驱动** (可选)
   - `ros2 launch realsense2_camera rs_launch.py`
   - 采集 RGB 图像

2. **Livox 雷达驱动**
   - `ros2 launch livox_ros_driver2 msg_MID360_launch.py`
   - 采集点云和 IMU 数据

3. **FAST-LIVO2 里程计**
   - 纯建图模式：`ros2 launch fast_livo mapping_avia.launch.py`
   - 重定位模式：由 `example_teaser_gicp.launch.py` 启动

4. **TEASER++GICP 重定位** (可选)
   - `ros2 launch example_teaser_gicp.launch.py`
   - 加载先验地图，进行全局重定位

5. **AR 九宫格叠加** (可选)
   - `ros2 launch ar_overlay.launch.py`
   - 将九宫格叠加到相机画面

## 日志查看

所有组件的输出日志保存在：

```bash
/home/r1/Slam/ar_calculate/log/one_click/<timestamp>_<component_name>.log
```

例如：
- `20260306_153045_livox_driver.log`
- `20260306_153045_fast_livo.log`
- `20260306_153045_teaser_gicp.log`

## 停止系统

按 `Ctrl+C` 即可停止所有启动的进程。脚本会自动清理所有子进程。

## 常见使用场景

### 场景 1：首次建图

```bash
# 不需要重定位，只做建图
ENABLE_RELOCALIZATION=0 ./start_all_with_offset.sh
```

运行后，在 RViz 中查看建图效果，完成后保存点云地图：

```bash
# 在另一个终端
ros2 service call /save_map std_srvs/srv/Trigger
```

### 场景 2：使用先验地图重定位

确保 `example_teaser_gicp.launch.py` 中的 `map_path` 参数指向正确的地图文件：

```python
{'map_path': '/home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra/src/FAST-LIVO2/Log/PCD/all_raw_points.pcd'}
```

然后启动：

```bash
# 默认启用重定位
./start_all_with_offset.sh
```

### 场景 3：完整 AR 显示

```bash
ENABLE_AR_OVERLAY=1 ./start_all_with_offset.sh
```

然后在另一个终端查看 AR 输出：

```bash
ros2 run rqt_image_view rqt_image_view /ar_overlay/image
```

## 监控系统状态

### 查看 ROS2 话题

```bash
ros2 topic list
```

关键话题：
- `/livox/lidar` - 点云数据
- `/livox/imu` - IMU 数据
- `/aft_mapped_to_init` - 里程计位姿
- `/camera/color/image_raw` - 相机图像
- `/ar_overlay/image` - AR 叠加输出

### 监控里程计

```bash
ros2 topic echo /aft_mapped_to_init
```

### 查看 TF 树

```bash
ros2 run tf2_tools view_frames
```

## 故障排除

### 1. 相机启动失败

```bash
# 禁用相机独立运行
ENABLE_CAMERA_DRIVER=0 ./start_all_with_offset.sh
```

### 2. 雷达连接失败

检查雷达网络连接：

```bash
ping 192.168.1.146
```

### 3. 重定位失败

检查地图文件路径：

```bash
ls -lh /home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra/src/FAST-LIVO2/Log/PCD/all_raw_points.pcd
```

### 4. AR 叠加无输出

确保九宫格配置文件存在：

```bash
ls -lh /home/r1/Slam/ar_calculate/nine_grid_points.yaml
cat /home/r1/Slam/ar_calculate/ar_overlay.params.yaml
```

### 5. 查看详细日志

```bash
# 查看最新日志
cd /home/r1/Slam/ar_calculate/log/one_click
ls -lt | head
tail -f <最新的日志文件>
```

## 前置条件

### 1. 编译主工作空间

```bash
cd /home/r1/Slam/FAST_LIVO2_ROS2_relocation_ultra
colcon build
```

### 2. 编译 AR 工作空间（如需 AR 功能）

```bash
cd /home/r1/Slam/ar_calculate
colcon build
```

### 3. 配置九宫格坐标

```bash
cd /home/r1/Slam/ar_calculate
python3 nine_grid.py --input-json nine_grid_input_template_unified.json --output-yaml nine_grid_points.yaml
```

## 自定义配置

### 修改相机分辨率

```bash
CAMERA_LAUNCH_CMD="ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false rgb_camera.color_profile:=1280x720x30" ./start_all_with_offset.sh
```

### 修改工作空间路径

编辑脚本中的路径变量：

```bash
MAIN_WORKSPACE="/your/path/to/FAST_LIVO2_ROS2_relocation_ultra"
AR_WORKSPACE="/your/path/to/ar_calculate"
```

## 技术支持

如遇问题，请查看：
1. 日志文件：`/home/r1/Slam/ar_calculate/log/one_click/`
2. 使用指南：`/home/r1/Slam/AR定位九宫格系统使用指南.md`
3. ROS2 日志：`~/.ros/log/`
