# 180度鱼眼相机配置快速参考卡

## 🚀 快速开始（5分钟配置）

### 1️⃣ 选择鱼眼模型

```bash
# 决策树：
OpenCV标定？ → fisheye_kannala_brandt
规格书说equidistant？ → fisheye_equidistant
不确定？ → fisheye_equidistant（最常见，先试这个）
```

### 2️⃣ 获取内参

**已有OpenCV标定结果？**
```yaml
camera_model: "fisheye_kannala_brandt"
camera_json: "/path/to/your_calibration.json"
```

**需要标定？**
```bash
cd /home/r1/Slam/ar_calculate/src/ar_grid_detector/scripts
python3 fisheye_calibration_tool.py \
  --images "calib/*.jpg" \
  --cols 10 --rows 7 \
  --square-size 0.03 \
  --output fisheye_params.json
```

**没有标定条件？**
```yaml
# 使用典型参数（1920x1080分辨率）
camera:
  width: 1920
  height: 1080
  fx: 650.0   # 400-800范围
  fy: 650.0
  cx: 960.0   # width/2
  cy: 540.0   # height/2
  k1: 0.0     # 如果选择equidistant可设为0
  k2: 0.0
  k3: 0.0
  k4: 0.0
```

### 3️⃣ 启动节点

```bash
# 编辑配置文件
nano /home/r1/Slam/ar_calculate/src/ar_grid_detector/config/ar_grid_fisheye_180deg.params.yaml

# 编译并启动
cd /home/r1/Slam/ar_calculate
colcon build --packages-select ar_grid_detector
source install/setup.bash
ros2 launch ar_grid_detector ar_grid_fisheye.launch.py
```

### 4️⃣ 验证效果

```bash
# 可视化检查
rqt_image_view /ar_grid/image

# 查看日志确认相机模型
ros2 run ar_grid_detector ar_grid_node
# 应看到：[INFO] Camera model: fisheye_equidistant
```

---

## 🎯 投影不准？问题诊断速查表

| 症状 | 原因 | 修改参数 |
|-----|------|---------|
| 格子整体偏移10cm | 坐标语义错误 | `input_is_cell_centers: true` |
| 边缘格子偏移20+cm | 鱼眼模型错误 | 尝试其他 `camera_model` |
| 边缘格子变形拉伸 | 畸变参数错误 | 重新标定k1-k4 |
| 左右不对称 | 主点cx偏移 | 微调 `cx` |
| 上下不对称 | 主点cy偏移 | 微调 `cy` |
| 整体平移（非10cm） | 外参平移错误 | 检查 `extrinsic_matrix_4x4` |
| 格子翻转 | 外参旋转错误 | 重新标定外参 |
| 完全没有投影 | 话题/外参/时间戳 | 检查ROS话题和外参数量级 |

---

## 📝 配置文件模板（复制粘贴）

### equidistant模型（无畸变参数）
```yaml
camera_model: "fisheye_equidistant"
camera:
  width: 1920
  height: 1080
  fx: 650.0
  fy: 650.0
  cx: 960.0
  cy: 540.0
  k1: 0.0
  k2: 0.0
  k3: 0.0
  k4: 0.0
```

### kannala_brandt模型（OpenCV标定）
```yaml
camera_model: "fisheye_kannala_brandt"
camera:
  width: 1920
  height: 1080
  fx: 650.5
  fy: 650.3
  cx: 960.2
  cy: 540.1
  k1: -0.234   # 从标定结果获取
  k2: 0.067    # 从标定结果获取
  k3: -0.012   # 从标定结果获取
  k4: 0.002    # 从标定结果获取
```

---

## 🔧 常用命令速查

```bash
# 查看相机话题
ros2 topic list | grep image

# 查看里程计话题
ros2 topic echo /aft_mapped_to_init --once

# 查看格子检测结果
ros2 topic echo /ar_grid/visible_cells

# 查看节点日志（检查加载的参数）
ros2 run ar_grid_detector ar_grid_node

# 检查时间戳同步
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /aft_mapped_to_init

# 重新编译
cd /home/r1/Slam/ar_calculate
colcon build --packages-select ar_grid_detector --symlink-install

# 运行标定工具
python3 scripts/fisheye_calibration_tool.py \
  --images "calib/*.jpg" \
  --cols 10 --rows 7 \
  --square-size 0.03 \
  --output fisheye_params.json
```

---

## 🎓 5种鱼眼模型速记

| 模型 | 公式 | 适用 |
|-----|------|------|
| equidistant | r=θ | 80%工业鱼眼 ⭐推荐 |
| equisolid | r=2sin(θ/2) | 全景相机 |
| orthographic | r=sin(θ) | 特殊光学 |
| stereographic | r=2tan(θ/2) | 保角需求 |
| kannala_brandt | θ_d=θ(1+k₁θ²+k₂θ⁴+...) | OpenCV标定 ⭐必用 |

---

## 📊 标定质量评估标准

| RMS误差 | 质量评级 | 是否可用 |
|--------|---------|---------|
| < 0.5 | 优秀 ⭐⭐⭐ | ✅ 直接使用 |
| 0.5-1.0 | 良好 ⭐⭐ | ✅ 直接使用 |
| 1.0-2.0 | 可接受 ⭐ | ⚠️ 建议重新标定 |
| > 2.0 | 不合格 | ❌ 必须重新标定 |

---

## 📁 文件路径速查

```
配置文件：  config/ar_grid_fisheye_180deg.params.yaml
启动文件：  launch/ar_grid_fisheye.launch.py
标定工具：  scripts/fisheye_calibration_tool.py
完整文档：  README.md (Section 4.1)
```

---

## ✅ 检查清单

配置前：
- [ ] 确认相机视场角≥180度
- [ ] 准备标定图像（或规格书内参）
- [ ] 确认ROS2话题正常发布

配置中：
- [ ] 选择正确的鱼眼模型
- [ ] 填写准确的内参（fx,fy,cx,cy）
- [ ] 配置畸变参数（kannala_brandt必需）
- [ ] 验证外参矩阵

配置后：
- [ ] 启动节点无报错
- [ ] 可视化图像中有格子投影
- [ ] 格子位置与真实世界对齐
- [ ] 边缘格子可见且位置正确

---

**提示：** 完整配置指南见 [README.md Section 4.1](README.md#41-180度鱼眼相机配置详细指南)
