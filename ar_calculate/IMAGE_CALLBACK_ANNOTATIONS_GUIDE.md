# image_callback 函数详细注释指南

## 概述
`image_callback` 是 AR 九宫格系统的核心投影函数，负责将世界坐标系中的格子投影到鱼眼相机图像上。该函数实现了完整的 **AR 变换链路**，并包含详细的可见性判定、消息发布和诊断功能。

---

## 函数执行流程分解

### **第一步：时间戳转换与 Odometry Synchronization**
```python
image_stamp_sec = self._stamp_to_sec(msg.header.stamp)
t_w_s, pose_age_sec = self._select_pose_for_image(image_stamp_sec)
```
- **时间戳提取**：从 ROS Image 消息的 Header 中获取图像采集时刻（秒级浮点数）
- **Pose 选择**：从 `_odom_history` 中查询与图像最接近的里程计位姿（`T_w_s`）
- **返回值**：
  - `t_w_s`: 4×4 齐次变换矩阵（Sensor 在 World 中的位姿）
  - `pose_age_sec`: 图像与所选 Pose 的时间间隔（毫秒级精度）

### **第二步：时间同步验证**
```python
max_pose_age_sec = float(self.get_parameter("odom_sync.max_pose_age_sec").value)  # 默认 0.25 秒
if pose_age_sec > max_pose_age_sec:
    # 警告日志
```
- **参数含义**：`odom_sync.max_pose_age_sec` 控制能容忍的最大 pose-image 时间差
- **影响**：超过阈值时系统会警告投影可能不准确（原因：里程计波动、图像处理延迟）
- **调优建议**：
  - **紧 (0.05-0.1s)**：实时系统、高频更新
  - **宽 (0.25-0.5s)**：网络延迟、低频传感器

---

## AR 变换链路详解

### **第三步：核心变换计算**

```python
# 已知：T_w_s（从 Odometry 得到，Sensor 在 World 中的位姿）
# 求解：P_c = T_c_w * P_w（世界点投影到相机图像）

# 1) 求逆得坐标系反向变换
t_s_w = inverse_transform(t_w_s)     # T_s_w = inv(T_w_s)，World -> Sensor

# 2) 与外参复合得最终变换
t_c_w = self.t_c_s @ t_s_w           # T_c_w = T_c_s * T_s_w，World -> Camera
                                     # 其中 T_c_s 是外参（Sensor -> Camera）
```

### **坐标系约定**
| 坐标系 | 含义 | 来源 |
|-------|------|------|
| `{world}` | 九宫格世界参考系 | SLAM 里程计 / 用户设定 |
| `{sensor}` | Lidar 原点（通常是 Odom 原点） | `/aft_mapped_in_map` 话题 |
| `{camera}` | 鱼眼相机光学中心 | 标定结果 |

### **外参错误的影响**
若 `T_c_s` 标定错误，会导致：
- **平移错误** → 整体位置偏移
- **旋转错误** → 格子倾斜、镜像反转
- **轴约定不一致** → 完全无法对齐

---

## 详细投影过程

### **第四步：参数加载**
```python
# 绘制参数（控制视觉效果）
border_color = tuple(int(v) for v in self.get_parameter("draw.cell_border_color_bgr").value)
border_thickness = int(self.get_parameter("draw.cell_border_thickness").value)
center_radius = int(self.get_parameter("draw.cell_center_radius").value)
show_labels = bool(self.get_parameter("draw.show_labels").value)
show_status_text = bool(self.get_parameter("draw.show_status_text").value)
curve_samples_per_edge = int(self.get_parameter("draw.curve_samples_per_edge").value)
use_curved_grid = self._is_fisheye_model_selected()
```

### **第五步：逐格子投影与可见性判定**

#### **5.1) 中心点投影 (World → Camera)**
```python
center_w_h = np.array([x, y, z, 1.0], dtype=np.float64)  # 齐次坐标
center_c_h = t_c_w @ center_w_h                          # 4×4 矩阵乘法
center_c = center_c_h[:3]                                # 提取 xyz，丢弃齐次分量
```
- `center_c[2]` 是**深度**（单位：米），必须 > 0 才能投影到图像前方

#### **5.2) 四角点投影与深度检查**
```python
for cw in cell.corners_world:
    p_w_h = np.array([cw[0], cw[1], cw[2], 1.0], dtype=np.float64)
    p_c_h = t_c_w @ p_w_h
    p_c = p_c_h[:3]
    
    # 深度判定：z <= 0 说明点在相机后方或极近处
    if p_c[2] <= 1e-6:
        corners_front = False
        break
    
    # 使用相机模型投影：camera 3D → pixel 2D
    uv = self.camera_model.project_point(p_c)
    if uv is None:  # 投影失败（可能：鱼眼畸变异常、点过靠近边缘）
        corners_front = False
        break
```

#### **5.3) 可见性判定逻辑**
```python
has_valid_corners = corners_front and len(corners_px) == 4
any_corner_in_image = has_valid_corners and any(
    0 <= u < w and 0 <= v < h for (u, v) in corners_px
)

visible = bool(has_valid_corners and (center_in_image or any_corner_in_image))
```
**可见性条件**：
1. **所有 4 个角点必须能投影成功**（没有在相机后方、畸变模型合法）
2. **至少有一个点在图像范围内**（中心或某个角点）

这样设计避免了投影失败产生的垃圾数据。

---

## 消息构建与发布

### **第六步：GridCell 消息构建**
每个格子生成一条 `GridCell` 消息，包含：
- **cell_id, row, col**: 格子标识
- **position_world_frame**: 世界坐标系中的固定位置
- **position_camera_frame**: 相机系中的位置（含深度）
- **corners_pixel**: 四个角点的像素坐标
- **center_pixel**: 中心点的像素坐标
- **is_visible**: 是否在图像中可见

### **第七步：消息发布**
```python
cells_msg = GridCellArray()
cells_msg.header = msg.header           # 继承图像时间戳
cells_msg.visible_cells = int(visible_count)
cells_msg.cells = [...]                # 逐个格子消息

self.visible_cells_pub.publish(cells_msg)
```

---

## 状态显示与可视化

### **第八步：状态文本生成** 
```
Status Format: odom=ok front=7/9 visible=6/9 dt_ms=12.5
├─ odom: 是否接收到里程计（ok 绿 / missing 红）
├─ front: 多少格子中心在相机前方 (z > 0)
├─ visible: 多少格子实际投影到图像上
└─ dt_ms: 图像与 Pose 的时间差（毫秒）
```

### **第九步：绘制边界**
- **Pinhole 模式**：直线四边形（`cv2.polylines`）
- **Fisheye 模式**：采样曲线四边形（`_draw_sampled_world_edge`）
  - 参数 `curve_samples_per_edge` 控制采样密度（越多越光滑但越慢）

---

## 参数调优指南

| 参数 | 类型 | 默认值 | 影响 | 调整建议 |
|------|------|--------|------|---------|
| `odom_sync.max_history` | int | 400 | 历史队列大小（容纳多久的旧 pose） | 增加：容忍图像处理延迟；减少：节省内存 |
| `odom_sync.use_closest_by_stamp` | bool | True | 选择"最接近"vs"最新"的 Pose | False：简单稳定；True：精确时序 |
| `odom_sync.max_pose_age_sec` | float | 0.25 | 时间同步警告阈值 | 紧：0.05-0.1s；宽：0.25-0.5s |
| `draw.show_window` | bool | True | 是否显示 OpenCV 窗口 | Headless 必须设 False |
| `draw.curve_samples_per_edge` | int | 16 | 鱼眼曲线采样点数 | 越多越光滑但越慢 |

---

## 诊断与故障排查

### **问题 1：没有可见格子**
检查清单：
1. ✓ 里程计话题工作？（查看状态栏 odom 是否为 ok）
2. ✓ 外参 `T_c_s` 是否正确标定？
3. ✓ 相机内参与畸变系数是否准确？
4. ✓ 格子位置（`grid.corner_*`）是否在相机前方？

### **问题 2：投影抖动/不稳定**
- 增加 `odom_sync.max_history`（长历史缓冲）
- 减小 `odom_sync.max_pose_age_sec`（严格时间对齐）
- 检查里程计频率是否够高（建议 ≥ 50 Hz）

### **问题 3：鱼眼模式下边界不光滑**
- 增加 `draw.curve_samples_per_edge`（例如 32 或 64）

---

## 性能注意事项

- **消息大小优化**：不可见格子用 0 填充，减少 ~50% 网络带宽
- **锁竞争**：Pose 选择时仅在必要时持锁，避免长时间等待
- **浮点精度**：所有矩阵计算使用 `float64` 以确保数值稳定性

---

## 相关代码模块

| 模块 | 功能 |
|------|------|
| `_select_pose_for_image` | 按时间戳查询最接近的 Pose |
| `odom_callback` | 维护 `_odom_history` 队列 |
| `camera_model.project_point` | 相机模型投影（Pinhole/Fisheye） |
| `_draw_sampled_world_edge` | 鱼眼曲线采样绘制 |

---

## 关键数值

| 常量 | 值 | 说明 |
|------|-----|------|
| 深度阈值 | 1e-6 m | 防止点在相机光心附近产生数值问题 |
| 默认历史大小 | 400 条 | @ 100 Hz Odom ≈ 4 秒缓冲 |
| 默认时间阈值 | 0.25 s | 250 ms 时间同步警告线 |

---

**最后更新**：2026-03-21  
**相关文件**：`ar_grid_node.py` 行 498-912
