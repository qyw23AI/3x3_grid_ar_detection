# ar_grid_detector

一个用于 AR 格子定位与可见格发布的 ROS2 功能包。

核心目标：
- 支持针孔与鱼眼相机投影模型。
- 通过格子架三个角点（左上、右上、左下）+ 行列参数，自动生成任意尺寸格子架。
- 将 3D 格子架投影到图像并叠加绘制。
- 发布当前视野中格子的结构化结果（编号、相机系位置、像素角点等），供后续 CV 节点做箱体识别与类型判断。

---

## 1. 功能包作用

本包解决的问题是：
1. 已知格子架在世界坐标系中的几何定义（角点 + 行列），以及系统里程计位姿。
2. 在每帧图像中实时计算哪些格子“可见”。
3. 把可见格子的编号与相机系位置发布出去，便于后续节点直接按格子索引处理图像 ROI。

相比“只画点”的方案，本包同时输出：
- 可视化图像（便于人工检查）；
- 机器可读消息（便于自动化识别与任务调度）。

---

## 2. 系统架构与处理逻辑

### 2.1 架构模块

- `ar_grid_detector_py/ar_grid_node.py`
  - 主节点：订阅里程计与图像，完成投影、可见性判断、可视化与消息发布。
- `ar_grid_detector/grid_generator.py`
  - 格子架生成器：根据三个角点生成全体格子（行优先编号）。
- `ar_grid_detector/camera_models.py`
  - 相机模型：针孔 + 多种鱼眼投影模型。
- `msg/GridCell.msg`
  - 单格结果描述。
- `msg/GridCellArray.msg`
  - 整帧格子结果数组。

### 2.2 数据流

1. 订阅里程计 `odom_topic`，得到传感器位姿 `T_w_s`。
2. 结合外参 `T_c_s`，计算 `T_c_w = T_c_s * inv(T_w_s)`。
3. 将格子架中每个格子的中心点/四角点从世界系变换到相机系。
4. 用相机模型把 3D 点投影到像素平面。
5. 判断可见性并绘制图像。
6. 发布：
   - 叠加图像；
   - 可见格消息数组（包含编号、相机系位置、像素信息）。

### 2.3 格子架几何定义（重点）

当前逻辑支持两种输入语义（通过 `grid.input_is_cell_centers` 控制）：

1) **角点模式**（`input_is_cell_centers=false`，默认语义）
- `corner_top_left`、`corner_top_right`、`corner_bottom_left` 表示**整块格子架外边框**的三个角点。
- 由三角点确定整体宽高，再按 `rows/cols` 等分生成每个格子。
- 计算关系：
  - `total_width = ||TR - TL||`
  - `total_height = ||BL - TL||`
  - `cell_width = total_width / cols`
  - `cell_height = total_height / rows`

2) **中心点模式**（`input_is_cell_centers=true`）
- 三个输入点解释为：左上格中心、右上格中心、左下格中心。
- 先按行列数把“中心点”反推为“外角点”，再走角点模式。
- 对 `rows x cols`：
  - `cell_width_vec = (CTR - CTL) / (cols - 1)`
  - `cell_height_vec = (CBL - CTL) / (rows - 1)`
  - `TL = CTL - 0.5*cell_width_vec - 0.5*cell_height_vec`
  - `TR = CTR + 0.5*cell_width_vec - 0.5*cell_height_vec`
  - `BL = CBL - 0.5*cell_width_vec + 0.5*cell_height_vec`

说明：
- `cell_width`、`cell_height` 仅用于可选一致性校验（`strict_size_check`），不主导几何构建。
- 若你是“站在格子中心采点”，必须启用 `input_is_cell_centers=true`，否则会出现整体偏移。

---

## 3. 发布结果说明

### 3.1 图像结果

- 话题：`/ar_grid/image`（可配置）
- 类型：`sensor_msgs/Image`
- 内容：
  - 当前相机画面
  - 可见格子的边框、中心点、编号
  - 状态文本（odom状态、可见格统计）

### 3.2 结构化格子结果

- 话题：`/ar_grid/visible_cells`（可配置）
- 类型：`ar_grid_detector/msg/GridCellArray`

关键字段：
- `rows` / `cols` / `total_cells` / `visible_cells`
- `visible_cell_ids`：当前可见格子的 ID 列表
- `cells`：全体格子信息（包含可见与不可见）

`GridCell` 关键字段：
- `cell_id`：格子编号（行优先）
- `row` / `col`
- `is_visible`
- `position_camera_frame`：格子中心相机系位置（米）
- `position_world_frame`：格子中心世界系位置（米）
- `corners_pixel[4]`：像素角点（左上、右上、右下、左下）
- `center_pixel`
- `depth`

不可见格子的约定：
- `is_visible = false`
- 像素坐标与相机系坐标置零（便于下游统一判空）

---

## 4. 相机模型与鱼眼支持

支持的 `camera_model`：
- `pinhole`
- `fisheye_equidistant`
- `fisheye_equisolid`
- `fisheye_orthographic`
- `fisheye_stereographic`
- `fisheye_kannala_brandt`

建议：
- 若使用 OpenCV 鱼眼标定结果，优先 `fisheye_kannala_brandt`。
- 可通过 `camera_json` 直接加载内参与畸变参数；也可用 `camera.*` 手动填写。

### 4.1 180度鱼眼相机配置详细指南

本节详细说明如何配置和使用180度视场角的鱼眼相机进行AR格子投影。

#### 4.1.1 鱼眼相机基础知识

**什么是鱼眼相机？**
- 鱼眼相机具有超广视场角（通常≥180度），能在单帧图像中捕获更大范围的场景
- 与针孔相机的线性投影不同，鱼眼相机使用非线性投影模型
- 图像边缘会产生明显的桶形畸变，这是正常现象，由投影模型决定

**180度鱼眼的特点：**
- 视场角（FOV）达到180度或接近180度
- 可以看到相机光轴垂直方向的物体（入射角θ=90°）
- 需要合适的投影模型来正确计算3D点到像素的映射关系
- 相比针孔相机，能覆盖更多格子（特别是侧边格子）

#### 4.1.2 选择合适的鱼眼投影模型

本系统支持5种鱼眼投影模型，选择依据如下：

| 模型类型 | camera_model参数 | 数学公式 | 适用场景 |
|---------|------------------|---------|---------|
| **等距投影** | `fisheye_equidistant` | r = f·θ | **最常见的鱼眼模型**，大多数工业鱼眼相机采用此投影 |
| **等立体角投影** | `fisheye_equisolid` | r = 2f·sin(θ/2) | 保持立体角均匀性，适合全景相机 |
| **正交投影** | `fisheye_orthographic` | r = f·sin(θ) | 较少使用，物理透镜较难实现 |
| **立体投影** | `fisheye_stereographic` | r = 2f·tan(θ/2) | 保持角度关系，边缘放大明显 |
| **Kannala-Brandt** | `fisheye_kannala_brandt` | θ_d = θ(1+k₁θ²+k₂θ⁴+k₃θ⁶+k₄θ⁸) | **OpenCV标定专用**，支持高阶畸变校正 |

**推荐选择流程：**

```
是否使用OpenCV cv2.fisheye标定结果？
├─ 是 → 使用 fisheye_kannala_brandt
└─ 否 → 根据相机规格书选择：
    ├─ 规格书注明"equidistant" → fisheye_equidistant（最常见）
    ├─ 规格书注明"equisolid angle" → fisheye_equisolid  
    ├─ 规格书未明确说明 → 先尝试 fisheye_equidistant
    └─ 投影有明显误差 → 依次测试其他模型
```

**重要提示：**
- **模型选择错误会导致系统性投影偏差**（特别是广角区域），表现为格子位置整体偏移或变形
- 如果没有规格书，建议使用 `fisheye_equidistant`（覆盖80%以上的鱼眼相机）
- 对于OpenCV标定的相机，**必须使用 `fisheye_kannala_brandt`**，其他模型无法利用k1-k4参数


#### 4.1.3 获取鱼眼相机内参

**方法1：使用OpenCV进行鱼眼标定（推荐）**

```python
import cv2
import numpy as np
import glob

# 1. 准备棋盘格标定板（建议10x7或更大）
chessboard_size = (10, 7)  # 内角点数量
square_size = 0.03  # 单位：米

# 2. 采集20-30张不同角度的标定图像
images = glob.glob('calibration_images/*.jpg')

# 3. 提取角点
objpoints = []  # 3D点
imgpoints = []  # 2D像素点

objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# 4. 鱼眼标定
img_shape = gray.shape[::-1]
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
K = np.zeros((3, 3))
D = np.zeros((4, 1))

ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints, imgpoints, img_shape, K, D, 
    flags=calibration_flags,
    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
)

# 5. 输出结果
print("Camera Matrix (K):")
print(K)
print("\nDistortion Coefficients (D):")
print(D)

# K[0,0]=fx, K[1,1]=fy, K[0,2]=cx, K[1,2]=cy
# D[0]=k1, D[1]=k2, D[2]=k3, D[3]=k4
```

**方法2：从相机ROS驱动获取**

如果你的鱼眼相机有ROS2驱动节点，可以从 `/camera_info` 话题读取：

```bash
ros2 topic echo /camera/camera_info
```

查看输出的 `k` (内参矩阵) 和 `d` (畸变系数)。

**方法3：从相机规格书/出厂标定文件**

部分工业相机会提供标定文件（JSON/YAML格式），直接使用 `camera_json` 参数加载。


#### 4.1.4 配置180度鱼眼相机参数

**配置文件示例：** `config/ar_grid.params.yaml`

```yaml
/**:
  ros__parameters:
    # ========== 相机模型选择 ==========
    # 根据4.1.2节的流程选择模型
    camera_model: "fisheye_equidistant"  # 或 fisheye_kannala_brandt
    
    # 可选：直接从JSON加载（与下方手动配置二选一）
    camera_json: ""  # 例如 "/path/to/fisheye_calibration.json"

    # ========== 手动配置内参 ==========
    camera:
      # 图像分辨率
      width: 1920
      height: 1080
      
      # 内参矩阵参数
      fx: 650.0   # x方向焦距（像素）
      fy: 650.0   # y方向焦距（像素）
      cx: 960.0   # 主点x坐标（通常接近width/2）
      cy: 540.0   # 主点y坐标（通常接近height/2）
      
      # 畸变参数（针孔模型用k1-k3、p1-p2；鱼眼模型用k1-k4）
      k1: 0.0     # 一阶径向畸变
      k2: 0.0     # 二阶径向畸变
      k3: 0.0     # 三阶径向畸变
      k4: 0.0     # 四阶径向畸变（鱼眼专用）
      p1: 0.0     # 切向畸变（针孔模型使用）
      p2: 0.0     # 切向畸变（针孔模型使用）

    # ========== 外参配置 ==========
    # 如果你的鱼眼相机与激光雷达之间的外参与之前的D435不同，
    # 需要重新标定 T_c_s（相机←雷达）变换矩阵
    extrinsic_matrix_4x4: [
      0.0, -1.0, 0.0, 0.0,
      0.0,  0.0,-1.0, 0.0,
      1.0,  0.0, 0.0, 0.0,
      0.0,  0.0, 0.0, 1.0
    ]
    
    # 或使用RPY表示（与matrix二选一）
    # extrinsic_rpy_xyz: [roll, pitch, yaw, tx, ty, tz]
```

**180度鱼眼典型参数范围：**
- `fx/fy`：通常在 400-800 之间（取决于分辨率和焦距）
- `cx`：应接近 `width/2`（误差<50像素）
- `cy`：应接近 `height/2`（误差<50像素）
- OpenCV标定的 `k1` 通常为负值（-0.1 ~ -0.5），`k2`为正值

**camera_json格式示例：**

```json
{
  "camera_model": "fisheye_kannala_brandt",
  "image_width": 1920,
  "image_height": 1080,
  "fx": 650.5,
  "fy": 650.3,
  "cx": 960.2,
  "cy": 540.1,
  "k1": -0.234,
  "k2": 0.067,
  "k3": -0.012,
  "k4": 0.002
}
```


#### 4.1.4.1 根据180度鱼眼镜头规格书配置完整流程

本节以**K103A-M12-F2.0 (焦距3.6mm, 180°视场角, -99.57%畸变)** 为例，说明如何根据镜头规格书完成完整配置。

---

##### **步骤1：解读规格书，确定相机模型类型**

**关键参数分析：**

| 规格书参数 | 数值 | 技术含义 |
|-----------|------|---------|
| 焦距 (Focal Length) | 3.6mm | 超短焦镜头（典型180°鱼眼特征） |
| 视场角 (HFOV) | 180°/180°/180° | 对角/水平/垂直均为180度 → 半球视野 |
| 畸变 (TV Distortion) | -99.57% | **极强桶形畸变**，无法用针孔模型处理 |
| 分辨率 | 中心200lp/mm, 边缘125lp/mm | 中心锐利、边缘下降（鱼眼固有特性） |

**模型选择判断：**

```
规格书特征                    → 推荐模型
─────────────────────────────────────────────────────
✓ 180度FOV                   → 必须使用鱼眼模型（fisheye_*）
✓ 焦距3.6mm（超短焦）          → 排除针孔模型
✓ 畸变-99.57%（近似-100%）    → 典型等距投影特征
✓ 规格书未说明具体投影类型     → 默认等距投影（最常见）

最终选择：fisheye_equidistant（等距投影模型）
```

**为什么是等距投影？**
- 80%以上的工业180度鱼眼镜头使用等距投影（r = f·θ）
- 畸变接近-100%是等距投影的典型特征
- 如果规格书明确标注其他投影类型（如equisolid），则按规格书选择

---

##### **步骤2：计算像素焦距 fx/fy**

**物理焦距 → 像素焦距转换公式：**

```
fx = f_mm / pixel_size_mm * 缩放因子
```

**对于K103A-M12-F2.0镜头：**

| 传感器尺寸 | 对角线 | 建议分辨率 | 像素尺寸估算 | fx计算 |
|-----------|--------|-----------|-------------|--------|
| 1/3" sensor | 6.0mm | 1920x1080 (1080P) | ~3.0μm | fx ≈ 3.6/0.003 = 1200 |
| 1/3" sensor | 6.0mm | 1280x720 (720P) | ~4.5μm | fx ≈ 3.6/0.0045 = 800 |
| 1/4" sensor | 4.5mm | 1280x720 (720P) | ~3.4μm | fx ≈ 3.6/0.0034 = 1058 |

**重要提示：**
- ⚠️ **规格书的物理焦距（3.6mm）不能直接使用**，必须转换为像素焦距
- 实际 `fx` 受传感器尺寸、分辨率、镜头设计影响，**强烈建议OpenCV标定获取准确值**
- 上表计算仅供初始化，实际误差可达±20%

**推荐做法：**
1. 使用上表估算值初始化配置
2. 用OpenCV `cv2.fisheye.calibrate()` 标定获取精确 `fx/fy/cx/cy/k1-k4`
3. 如果无法标定，可尝试在估算值基础上微调 `fx` (±100像素范围)

---

##### **步骤3：完整配置180度鱼眼相机（以1080P为例）**

**配置文件：** `config/ar_grid_fisheye.params.yaml`

```yaml
/**:
  ros__parameters:
    # ==================== 相机模型选择 ====================
    # 根据规格书判断为等距投影模型
    camera_model: "fisheye_equidistant"
    
    # ==================== 手动配置内参 ====================
    camera:
      # 图像分辨率（根据你的实际相机输出设置）
      width: 1920
      height: 1080
      
      # 内参矩阵 - 初始估算值（需后续标定修正）
      fx: 1200.0   # 按1/3"传感器+3.6mm焦距估算
      fy: 1200.0   # 通常与fx相同
      cx: 960.0    # width/2
      cy: 540.0    # height/2
      
      # 畸变参数 - 初始化为0（需标定后填入）
      k1: 0.0      # 第一次使用时先设为0
      k2: 0.0      # 标定后会得到负值（典型范围 -0.2 ~ -0.4）
      k3: 0.0      # 标定后会得到小正值
      k4: 0.0      # 标定后会得到更小的正值
      
      # 针孔模型参数（鱼眼不使用，保持0即可）
      p1: 0.0
      p2: 0.0

    # ==================== 外参配置 ====================
    # 相机-雷达外参（需根据你的硬件安装情况标定）
    extrinsic_matrix_4x4: [
      0.0, -1.0, 0.0, 0.0,   # 第一行：相机X轴在雷达系的方向
      0.0,  0.0,-1.0, 0.0,   # 第二行：相机Y轴在雷达系的方向
      1.0,  0.0, 0.0, 0.0,   # 第三行：相机Z轴（光轴）在雷达系的方向
      0.0,  0.0, 0.0, 1.0    # 第四行：齐次坐标
    ]
    
    # ==================== 格子架配置 ====================
    grid:
      rows: 3
      cols: 3
      cell_width: 0.075
      cell_height: 0.075
      corner_top_left: [-0.532, 0.902, 0.347]
      corner_top_right: [0.226, 0.776, 0.347]
      corner_bottom_left: [-0.597, 0.277, 0.347]
      input_is_cell_centers: false  # 如果采点时站在格子中心，改为true
    
    # ==================== 话题与可视化 ====================
    image_topic: "/camera/image_raw"
    odom_topic: "/aft_mapped_to_init"
    publish_image: true
    publish_grid_cells: true
```

---

##### **步骤4：使用OpenCV进行鱼眼相机标定（推荐）**

**4.1 准备棋盘格标定板**

- 尺寸：10x7 内角点（或更大，如 12x9）
- 方格大小：30mm x 30mm（打印在A3纸上）
- 要求：平整、对比度高、黑白分明

**4.2 采集标定图像**

```bash
# 启动相机节点（假设你的鱼眼相机驱动已运行）
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# 使用rqt_image_view查看图像并保存
rqt_image_view /camera/image_raw

# 采集20-30张不同角度、距离的标定图像：
# - 棋盘格占据画面30-80%
# - 覆盖图像中心、四角、边缘区域
# - 不同倾斜角度（±30度）
# - 保存为 calibration_images/img_001.jpg, img_002.jpg, ...
```

**4.3 运行标定脚本**

```python
#!/usr/bin/env python3
import cv2
import numpy as np
import glob

# 棋盘格参数
CHECKERBOARD = (10, 7)  # 内角点数量
SQUARE_SIZE = 0.03      # 方格边长（米）

# 准备3D Object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D points in world space
imgpoints = []  # 2D points in image plane

# 加载标定图像
images = sorted(glob.glob('calibration_images/*.jpg'))
print(f"Found {len(images)} calibration images")

for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    
    if ret:
        # 亚像素精度优化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        
        # 可视化（可选）
        img_with_corners = cv2.drawChessboardCorners(img, CHECKERBOARD, corners_refined, ret)
        cv2.imwrite(f'calibration_images/detected_{idx:03d}.jpg', img_with_corners)
        print(f"✓ {fname}: corners detected")
    else:
        print(f"✗ {fname}: corners NOT detected")

# 执行鱼眼标定
print(f"\nCalibrating with {len(objpoints)} valid images...")
img_shape = gray.shape[::-1]

K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = []
tvecs = []

calibration_flags = (
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
    cv2.fisheye.CALIB_FIX_SKEW +
    cv2.fisheye.CALIB_CHECK_COND
)

ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints, imgpoints, img_shape,
    K, D, rvecs, tvecs,
    flags=calibration_flags,
    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
)

# 输出标定结果
print("\n" + "="*60)
print("鱼眼相机标定结果")
print("="*60)
print("\nCamera Matrix (K):")
print(K)
print(f"\nIntrinsic Parameters:")
print(f"  fx = {K[0, 0]:.2f}")
print(f"  fy = {K[1, 1]:.2f}")
print(f"  cx = {K[0, 2]:.2f}")
print(f"  cy = {K[1, 2]:.2f}")
print(f"\nDistortion Coefficients (D):")
print(D.ravel())
print(f"  k1 = {D[0, 0]:.6f}")
print(f"  k2 = {D[1, 0]:.6f}")
print(f"  k3 = {D[2, 0]:.6f}")
print(f"  k4 = {D[3, 0]:.6f}")
print(f"\nRMS re-projection error: {ret:.4f} pixels")
print("="*60)

# 保存为JSON配置文件
config = {
    "camera_model": "fisheye_kannala_brandt",
    "image_width": img_shape[0],
    "image_height": img_shape[1],
    "fx": float(K[0, 0]),
    "fy": float(K[1, 1]),
    "cx": float(K[0, 2]),
    "cy": float(K[1, 2]),
    "k1": float(D[0, 0]),
    "k2": float(D[1, 0]),
    "k3": float(D[2, 0]),
    "k4": float(D[3, 0])
}

import json
with open('fisheye_calibration.json', 'w') as f:
    json.dump(config, f, indent=2)

print("\n配置已保存到: fisheye_calibration.json")
print("在 ar_grid.params.yaml 中设置:")
print(f"  camera_json: \"{os.path.abspath('fisheye_calibration.json')}\"")
```

**4.4 将标定结果写入配置文件**

标定完成后，更新 `ar_grid.params.yaml`：

```yaml
/**:
  ros__parameters:
    # 注意：标定后应使用 kannala_brandt 模型（OpenCV的fisheye模型）
    camera_model: "fisheye_kannala_brandt"
    
    # 方法1：直接加载标定JSON（推荐）
    camera_json: "/home/r1/Slam/ar_calculate/fisheye_calibration.json"
    
    # 方法2：手动填写（与camera_json二选一）
    # camera:
    #   width: 1920
    #   height: 1080
    #   fx: 1205.34    # 从标定结果复制
    #   fy: 1203.21    # 从标定结果复制
    #   cx: 962.15     # 从标定结果复制
    #   cy: 541.03     # 从标定结果复制
    #   k1: -0.2341    # 从标定结果复制
    #   k2: 0.0672     # 从标定结果复制
    #   k3: -0.0124    # 从标定结果复制
    #   k4: 0.0021     # 从标定结果复制
```

---

##### **步骤5：验证180度鱼眼投影准确性**

**5.1 启动系统并观察投影**

```bash
cd /home/r1/Slam/ar_calculate
colcon build --packages-select ar_grid_detector
source install/setup.bash
ros2 launch ar_grid_detector ar_grid.launch.py
```

**5.2 检查关键区域**

| 检查项 | 预期结果 | 异常诊断 |
|-------|---------|---------|
| **中心格子** | 与真实位置精确对齐（误差<2cm） | fx/fy不准确 → 重新标定 |
| **边缘格子** | 可见且形状合理（有透视变形） | 外参T_c_s错误 → 检查相机-雷达外参 |
| **侧面格子** | 180度FOV应能看到侧面90度区域 | k1-k4畸变参数错误 → 重新标定 |
| **格子形状** | 直线（在3D中）投影为曲线（鱼眼特性） | 相机模型选择错误 → 检查camera_model |

**5.3 通过日志验证配置加载**

```bash
ros2 run ar_grid_detector ar_grid_node --ros-args --log-level debug
```

应看到：
```
[INFO] Camera model: fisheye_kannala_brandt
[INFO] Camera: 1920x1080, fx=1205.3, fy=1203.2, cx=962.2, cy=541.0
[INFO] Fisheye distortion: k1=-0.2341, k2=0.0672, k3=-0.0124, k4=0.0021
[INFO] Loaded grid: 3x3, cell_size=0.075m
```

---

##### **步骤6：常见问题排查**

| 问题现象 | 根本原因 | 解决方案 |
|---------|---------|---------|
| **边缘格子严重变形/拉伸** | 未标定或k1-k4为0 | 必须OpenCV标定获取畸变系数 |
| **格子整体偏移5-10cm** | `input_is_cell_centers`设置错误 | 检查坐标采集方式（角点/中心点） |
| **侧面格子完全不可见** | 外参T_c_s与实际硬件不符 | 重新标定相机-雷达外参 |
| **主点cx/cy偏离中心>100px** | 镜头光轴与传感器不同心 | 必须标定，不能用width/2估算 |
| **中心准确但边缘偏差递增** | 相机模型选择错误 | 尝试其他鱼眼投影模型 |
| **投影准确但FPS很低** | 图像分辨率过高 | 降低分辨率到720P |

---

##### **快速配置总结（TL;DR）**

```bash
# 1. 根据规格书选择模型
camera_model: "fisheye_equidistant"  # 180度FOV → 等距投影

# 2. 初始化内参（仅用于第一次测试）
fx/fy: 1200  # 按3.6mm焦距+1/3"传感器估算
cx/cy: width/2, height/2
k1-k4: 0     # 必须标定后填入

# 3. 使用OpenCV标定（必须步骤）
python3 fisheye_calibration.py  # 获取准确内参

# 4. 更新配置文件
camera_model: "fisheye_kannala_brandt"  # 标定后改用此模型
camera_json: "/path/to/fisheye_calibration.json"

# 5. 验证
rqt_image_view /ar_grid/image  # 检查格子投影是否准确
```

---

#### 4.1.5 验证配置正确性

**步骤1：启动节点**

```bash
cd /home/r1/Slam/ar_calculate
colcon build --packages-select ar_grid_detector
source install/setup.bash
ros2 launch ar_grid_detector ar_grid.launch.py
```

**步骤2：检查投影准确性**

在 `rqt_image_view /ar_grid/image` 中观察：

1. **格子中心点是否对齐真实世界位置**
   - 在已知格子位置放置特征物体（如彩色标记）
   - 检查投影的格子中心是否与标记重合

2. **格子形状是否正确**
   - 等大格子在图像中应呈现合理的透视变形
   - 不应出现拉伸、压缩异常

3. **边缘格子投影是否完整**
   - 180度鱼眼应能看到侧边格子
   - 如果靠近图像边缘的格子丢失，检查：
     - 相机外参是否正确
     - `cx/cy` 是否准确

**步骤3：通过日志排查问题**

```bash
# 查看节点输出，确认加载的相机模型
ros2 run ar_grid_detector ar_grid_node

# 应输出：
# [INFO] Camera model: fisheye_equidistant
# [INFO] Camera: 1920x1080, fx=650.0, fy=650.0, cx=960.0, cy=540.0
```

**常见问题诊断：**

| 现象 | 可能原因 | 解决方法 |
|-----|---------|---------|
| 格子整体偏移10-20cm | 输入坐标语义错误 | 检查 `input_is_cell_centers` 参数（见2.3节） |
| 格子在边缘严重变形 | 鱼眼模型选择错误 | 尝试其他投影模型 |
| 格子只在中心区域准确 | 畸变参数k1-k4未配置 | 使用OpenCV标定获取畸变系数 |
| 侧边格子完全不可见 | 外参 `T_c_s` 错误 | 重新标定相机-雷达外参 |
| 主点cx/cy偏差大 | 内参标定不准确 | 重新标定或调整 `cx/cy` 值 |


#### 4.1.6 针孔相机与鱼眼相机对比

| 特性 | 针孔相机 (Pinhole) | 180度鱼眼相机 (Fisheye) |
|-----|-------------------|----------------------|
| 视场角 | 60-90度 | ≥180度 |
| 畸变特性 | 径向+切向畸变（轻微） | 强烈桶形畸变（固有特性） |
| 投影模型 | 线性透视投影 | 非线性鱼眼投影 |
| 边缘图像质量 | 较好 | 分辨率下降 |
| 可见格子数 | 较少（仅前方） | 多（含侧面） |
| 标定难度 | 简单 | 中等（需要cv2.fisheye） |
| 配置参数 | fx,fy,cx,cy,k1-k3,p1-p2 | fx,fy,cx,cy,k1-k4 |
| camera_model | `pinhole` | `fisheye_*` 系列 |

**何时使用鱼眼相机？**
- ✅ 需要监控大范围格子架（如3x3以上）
- ✅ 格子分布在相机两侧
- ✅ 移动机器人需要广域感知
- ❌ 只关注正前方2x2格子 → 针孔相机即可
- ❌ 对边缘图像质量要求极高 → 针孔相机更合适

---

## 5. 使用方式

## 5.1 编译

在工作区根目录执行：

```bash
cd /home/r1/Slam/ar_calculate
colcon build --packages-select ar_grid_detector
source install/setup.bash
```

## 5.2 启动

**针孔相机（默认配置）：**
```bash
ros2 launch ar_grid_detector ar_grid.launch.py
```

**180度鱼眼相机（专用配置）：**
```bash
ros2 launch ar_grid_detector ar_grid_fisheye.launch.py
```

默认参数文件：
- 针孔相机：`config/ar_grid.params.yaml`
- 鱼眼相机：`config/ar_grid_fisheye_180deg.params.yaml`

## 5.3 180度鱼眼相机快速开始

### 硬件前置：USB鱼眼相机驱动配置

#### 硬件参数确认

根据**K103A-M12-F2.0规格书**，180度鱼眼相机规格：

| 参数 | 值 | 说明 |
|-----|-----|------|
| 焦距 | 3.6mm | 超广角 |
| 视场角 | 180° | 真正的全向鱼眼 |
| 分辨率支持 | 1080P、720P | 根据规格书支持 |
| 畸变 | -99.57% (TV) | 极强桶形畸变 |
| 工作距离 | 0.2m~∞ | 近距离工作能力强 |
| 工作波长 | 436~700nm | 可见光 |
| 工作温度 | -20~+60°C | 适应温度范围 |

#### USB设备识别与选择

**查看系统中的USB视频设备：**

```bash
# 列出所有视频设备
ls -la /dev/video*

# 应输出类似：
# /dev/video0  （通常是第一个摄像头）
# /dev/video1  （如果有多个摄像头）
```

**确定USB鱼眼相机对应的设备号：**

```bash
# 方法1：查看usb设备信息
lsusb | grep -i camera
# 或
lsusb | grep -i video

# 方法2：使用v4l2工具查看支持的格式（需要安装）
sudo apt install v4l-utils
v4l2-ctl --list-devices

# 输出示例：
# fisheye_camera (usb-0000:00:14.0-1):
#         /dev/video0

# 方法3：查看设备详细信息
sudo v4l2-ctl -d /dev/video0 --all
```

**如果有多个USB摄像头，确定正确的设备：**

```bash
# 单独测试video0
ros2 run usb_cam usb_cam_node_exe -p video_device:=/dev/video0

# 新终端查看画面
rqt_image_view /image_raw

# 确认是鱼眼相机后按Ctrl+C停止，记住设备号
```

#### USB启动命令配置

**完整的USB鱼眼相机启动命令（已在start_all.sh中配置）：**

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=30 \
  -p pixel_format:=mjpeg \
  -p camera_name:=fisheye_camera
```

**参数说明：**

| 参数 | 值 | 说明 |
|-----|-----|------|
| `image_raw` | `/fisheye_camera/image_raw` | 输出话题名称（重映射） |
| `video_device` | `/dev/video0` | USB设备路径（确认后修改） |
| `image_width` | `1920` | 宽度 - 1920x1080为1080P标准 |
| `image_height` | `1080` | 高度 - 根据规格书选择 |
| `framerate` | `30` | **关键参数** - 确保至少30fps |
| `pixel_format` | `mjpeg` | MJPEG压缩格式（可选：mjpeg/yuyv/rgb24） |
| `camera_name` | `fisheye_camera` | 驱动内部名称 |

**分辨率与帧率选择权衡表：**

| 分辨率 | 推荐帧率 | 带宽需求 | 说明 |
|--------|--------|--------|------|
| 1920×1080 | 30fps | 中等 | **推荐** - 1080P效果好，帧率足够 |
| 1280×720 | 60fps | 高 | 清晰度一般，帧率高但带宽大 |
| 640×480 | 120fps | 低 | 分辨率太低，AR定位精度差 |

**格式选择说明：**

- `mjpeg`：**推荐** - 压缩率高，1920×1080@30fps可以流畅传输
- `yuyv`：无压缩，带宽需求高（1920×1080@30fps容易卡顿）
- `rgb24`：无压缩，3倍数据量，通常不推荐

#### 通过环境变量灵活配置

在 `start_all.sh` 中已支持环境变量覆盖。如需临时测试其他配置：

```bash
# 尝试720P@60fps
USB_CAMERA_LAUNCH_CMD='ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=60 \
  -p pixel_format:=mjpeg' \
./start_all.sh

# 尝试指定不同USB设备（如有多个摄像头）
USB_CAMERA_LAUNCH_CMD='ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video1 \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=30 \
  -p pixel_format:=mjpeg' \
./start_all.sh
```

#### USB启动故障排除

**错误1：`Could not open video device`**

```bash
# 原因：设备不存在或权限不足
# 解决：
1. 验证设备存在：ls -la /dev/video0
2. 检查权限：sudo usermod -a -G video $USER
3. 需要重新登录或运行：newgrp video
```

**错误2：`Cannot set resolution 1920x1080`**

```bash
# 原因：USB设备不支持该分辨率
# 解决：查看支持的分辨率并选择
v4l2-ctl -d /dev/video0 --list-formats-ext

# 根据输出选择设备实际支持的分辨率，例如只支持1280×720：
USB_CAMERA_LAUNCH_CMD='ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=30 \
  -p pixel_format:=mjpeg' ./start_all.sh
```

**错误3：`Framerate setting failed` 或帧率达不到30fps**

```bash
# 原因1：USB带宽不足（通常因为格式或分辨率过高）
# 解决：切换到mjpeg格式或降低分辨率

# 原因2：USB总线被占用
# 解决：不要同时运行其他USB设备驱动

# 当前配置的带宽估算：
# 1920×1080 MJPEG @30fps ≈ 15~25 Mbps（可用）
# 1920×1080 YUYV @30fps ≈ 200 Mbps（超过USB 2.0限制！）

# 验证实际帧率：
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=30 \
  -p pixel_format:=mjpeg &
sleep 5
ros2 topic hz /fisheye_camera/image_raw
kill %1
```

**错误4：图像质量差，色彩失真**

```bash
# 原因：MJPEG压缩质量设置过低
# 解决：检查usb_cam节点的质量参数
# （注：当前版本无法通过launch文件设置，需编辑源码）

# 临时方案：尝试yuyv格式（帧率降低但质量好）
USB_CAMERA_LAUNCH_CMD='ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r image_raw:=/fisheye_camera/image_raw \
  -p video_device:=/dev/video0 \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=15 \
  -p pixel_format:=yuyv' ./start_all.sh
```

---

### 第一步：相机标定（首次使用）

如果你还没有鱼眼相机内参，需要先进行标定：

```bash
# 1. 准备棋盘格标定板（10x7，30mm方格）
#    可从网上下载打印，建议A3纸

# 2. 采集20-30张标定图像到文件夹
#    覆盖图像的中心、边缘、四个角落
#    不同角度、不同距离

# 3. 运行标定工具
cd /home/r1/Slam/ar_calculate/src/ar_grid_detector/scripts
chmod +x fisheye_calibration_tool.py

python3 fisheye_calibration_tool.py \
  --images "/path/to/calibration_images/*.jpg" \
  --cols 10 --rows 7 \
  --square-size 0.03 \
  --output fisheye_params.json

# 4. 标定完成后会输出：
#    - 内参：fx, fy, cx, cy
#    - 畸变参数：k1, k2, k3, k4
#    - YAML配置片段（可直接复制）
#    - JSON文件（可用camera_json加载）
```

**标定质量评估：**
- RMS < 0.5：优秀 ✅
- RMS < 1.0：良好 ✅
- RMS < 2.0：可接受 ⚠️
- RMS ≥ 2.0：质量不佳，需重新标定 ❌

### 第二步：配置参数文件

编辑 `config/ar_grid_fisheye_180deg.params.yaml`：

```yaml
camera_model: "fisheye_kannala_brandt"  # 如果用OpenCV标定
# 或
camera_model: "fisheye_equidistant"     # 如果相机规格书注明equidistant

# 方式1：直接加载JSON（推荐）
camera_json: "/path/to/fisheye_params.json"

# 方式2：手动填写（从标定工具输出复制）
camera:
  width: 1920
  height: 1080
  fx: 650.5
  fy: 650.3
  cx: 960.2
  cy: 540.1
  k1: -0.234   # 从标定结果获取
  k2: 0.067
  k3: -0.012
  k4: 0.002

# 确认格子架配置正确
grid:
  rows: 3
  cols: 3
  corner_top_left: [-0.081696, 0.925071, 0.370132]
  corner_top_right: [0.128394, 0.702569, 0.372652]
  corner_bottom_left: [-0.158594, 0.849270, 0.083796]
  input_is_cell_centers: true  # 如果坐标是格子中心位置

# 如果鱼眼相机安装位置与之前不同，需要更新外参
extrinsic_matrix_4x4: [...]  # 重新标定相机-雷达外参
```

### 第三步：编译与启动

```bash
# 编译
cd /home/r1/Slam/ar_calculate
colcon build --packages-select ar_grid_detector
source install/setup.bash

# 启动鱼眼配置
ros2 launch ar_grid_detector ar_grid_fisheye.launch.py
```

### 第四步：验证投影效果

```bash
# 查看可视化结果
rqt_image_view /ar_grid/image

# 查看格子消息
ros2 topic echo /ar_grid/visible_cells

# 检查日志中的相机配置
# 应看到：
# [INFO] Camera model: fisheye_kannala_brandt
# [INFO] Camera: 1920x1080, fx=650.5, fy=650.3, ...
```

**验证要点：**
- ✅ 格子中心对齐真实世界位置（放置标记物验证）
- ✅ 格子边框形状正确（透视变形合理）
- ✅ 边缘格子可见（180度鱼眼应该能看到侧边格子）
- ✅ 日志无错误或警告

**常见问题快速诊断：**
```bash
# 问题1：格子整体偏移10-20cm
# 原因：input_is_cell_centers参数错误
# 解决：确认采点方式，修改参数

# 问题2：边缘格子严重变形
# 原因：鱼眼模型选择错误
# 解决：尝试其他模型（equidistant/equisolid/stereographic）

# 问题3：侧边格子不可见
# 原因：外参T_c_s不正确
# 解决：重新标定相机-雷达外参

# 问题4：中心准确但边缘偏差大
# 原因：畸变参数k1-k4未配置或不准确
# 解决：重新标定获取正确的畸变参数
```

## 5.4 查看输出

```bash
ros2 topic echo /ar_grid/visible_cells
ros2 topic hz /ar_grid/visible_cells
rqt_image_view /ar_grid/image
```

---

## 6. 参数说明与调参建议

参数文件：`config/ar_grid.params.yaml`

### 6.1 订阅/发布话题

- `odom_topic`：里程计输入（建议与 FAST-LIVO2 输出一致）
- `image_topic`：图像输入
- `output_image_topic`：叠加图像输出
- `visible_cells_topic`：格子结果输出

调参建议：
- 若无结果先检查话题是否有数据、时间戳是否连续。

### 6.2 相机参数

- `camera_model`
- `camera_json`
- `camera.fx/fy/cx/cy`
- `camera.width/height`
- `camera.k1~k4`，`camera.p1/p2`

调参建议：
- 优先使用标定文件；
- 鱼眼模型错误会表现为边缘格子投影明显漂移或扭曲。

### 6.3 格子架参数（核心）

- `grid.rows` / `grid.cols`
- `grid.corner_top_left`
- `grid.corner_top_right`
- `grid.corner_bottom_left`
- `grid.input_is_cell_centers`
- `grid.cell_width` / `grid.cell_height`（校验项）
- `grid.strict_size_check`
- `grid.size_tolerance`

调参建议：
1. 先确认你的采点语义：采“外角点”还是采“格子中心”。
2. 若采的是格子中心，设置 `grid.input_is_cell_centers=true`。
3. 再校验 `rows/cols` 是否与真实格子架一致。
4. `cell_width/cell_height` 与角点推导值不一致时，先关闭 `strict_size_check` 进行排查。

### 6.6 关键注意事项（避免“飘”和“大偏差”）

1. **语义一致性优先于数值精度**
- 最常见问题不是内参误差，而是“中心点被当成角点”或相反。
- 这会导致所有格子整体平移和尺度错配。

2. **行列数会改变中心点反推结果**
- 中心点模式下，`rows/cols` 直接参与反推外角点。
- 同一组点，`2x2` 与 `3x3` 的外角点不是同一个几何对象。

3. **外参必须统一约定**
- 本节点采用 `T_c_w = T_c_s * inv(T_w_s)`。
- 若与其它节点使用不同 `T_c_s` 或不同坐标轴约定，会表现为整体旋转/镜像/移位。

4. **时间同步会影响“抖动感”**
- 图像与位姿时间戳差过大时，画面会出现漂移或跳变。
- 可调 `odom_sync.use_closest_by_stamp`、`odom_sync.max_pose_age_sec`。

### 6.4 外参参数

- `extrinsic_matrix_4x4`（优先）
- `extrinsic_rpy_xyz`（备用）

调参建议：
- 若投影整体偏移/旋转，优先检查外参；
- 若中心正确边缘漂，优先检查相机模型和畸变参数。

### 6.5 绘图参数

- `draw.cell_border_*`
- `draw.cell_center_*`
- `draw.label_color_bgr`
- `draw.show_labels`
- `draw.show_status_text`
- `draw.show_window`

---

## 7. 快速排障

1. 没有任何可见格：
- 检查里程计是否在发布；
- 检查 `T_c_s` 与坐标轴方向；
- 检查角点是否在当前视野前方。

2. 位置整体偏移很大（但形状看起来“像是对的”）：
- 优先检查 `grid.input_is_cell_centers` 是否与采点方式匹配；
- 检查是否把“格子中心坐标”直接填进了 `corner_*`；
- 检查 `rows/cols` 是否和采点时的真实格子一致。

3. 可见格编号错位：
- 检查角点顺序是否严格为“左上、右上、左下”；
- 检查 `rows/cols` 是否与真实格子架一致。

4. 鱼眼边缘投影失真：
- 尝试切换 `camera_model`；
- 核对 `camera_json` 中畸变参数格式与顺序。

---

## 8. 与下游识别节点对接建议

下游节点推荐处理流程：
1. 订阅 `/ar_grid/visible_cells`。
2. 对 `cells` 里 `is_visible=true` 的格子遍历。
3. 使用 `corners_pixel` 生成 ROI 或透视变换区域。
4. 在 ROI 内做箱体检测/分类。
5. 将检测结果与 `cell_id` 绑定输出，形成“格子编号 -> 物体类型”映射。

---

## 9. 目录

```text
ar_grid_detector/
├── ar_grid_detector/
│   ├── ar_grid_node.py
│   ├── camera_models.py
│   ├── grid_generator.py
│   └── utils.py
├── msg/
│   ├── GridCell.msg
│   └── GridCellArray.msg
├── launch/
│   ├── ar_grid.launch.py
│   └── ar_grid_fisheye.launch.py      # 鱼眼相机专用启动文件
├── config/
│   ├── ar_grid.params.yaml
│   ├── ar_grid_fisheye_180deg.params.yaml  # 鱼眼相机配置模板
│   └── grid_example.yaml
├── scripts/
│   └── fisheye_calibration_tool.py    # 鱼眼标定工具
├── CMakeLists.txt
├── package.xml
└── setup.py
```

---

## 10. 附录：常见180度鱼眼相机配置参考

### 10.1 常见鱼眼相机型号与推荐模型

| 相机型号 | 分辨率 | 视场角 | 推荐模型 | 备注 |
|---------|--------|-------|---------|------|
| **Arducam OV5647 Fisheye** | 2592×1944 | 175° | `fisheye_equidistant` | 适合树莓派，需自行标定 |
| **Intel RealSense T265** | 848×800 | 163° | `fisheye_kannala_brandt` | 官方提供标定参数 |
| **Insta360 ONE X2** | 5760×2880 | 360° (单镜头180°) | `fisheye_equisolid` | 需要去拼接 |
| **FLIR Blackfly S USB3 (鱼眼镜头)** | 1920×1200 | 185° | `fisheye_kannala_brandt` | 工业级，需OpenCV标定 |
| **Lensagon BF2M15420** | 1920×1080 | 185° | `fisheye_equidistant` | M12接口，常用工业镜头 |

### 10.2 鱼眼标定工具对比

| 工具 | 优点 | 缺点 | 推荐度 |
|-----|------|------|--------|
| **OpenCV cv2.fisheye** | 免费、准确、支持Python | 需要编写脚本 | ⭐⭐⭐⭐⭐ |
| **Kalibr (ROS)** | 支持多相机联合标定 | 配置复杂 | ⭐⭐⭐⭐ |
| **MATLAB Camera Calibrator** | 图形界面友好 | 需要License | ⭐⭐⭐ |
| **相机厂商工具** | 针对特定相机优化 | 通用性差 | ⭐⭐⭐ |

### 10.3 典型180度鱼眼参数范围

```yaml
# 分辨率：常见范围
width: 640~4K
height: 480~2K

# 焦距：与分辨率和传感器尺寸相关
# 对于1920×1080分辨率的1/2.3英寸传感器：
fx: 400~800 像素
fy: 400~800 像素

# 主点：通常接近图像中心
cx: width/2 ± 50
cy: height/2 ± 50

# 畸变参数（OpenCV fisheye）：
# 180度鱼眼典型值
k1: -0.5 ~ 0.0  # 通常为负值
k2: -0.2 ~ 0.3  # 可正可负
k3: -0.1 ~ 0.1  # 高阶项，影响较小
k4: -0.05 ~ 0.05
```

### 10.4 鱼眼投影模型数学详解

以下是各种鱼眼投影模型的完整数学描述：

**坐标系定义：**
- `(X, Y, Z)`：相机坐标系下的3D点
- `r_3d = sqrt(X² + Y²)`：点到光轴的距离
- `θ = atan2(r_3d, Z)`：入射角（0~π）
- `r`：像素平面上的径向距离（归一化）
- `(u, v)`：最终像素坐标

**投影公式：**

1. **等距投影 (Equidistant)**
   ```
   r = θ
   u = fx * r * (X/r_3d) + cx
   v = fy * r * (Y/r_3d) + cy
   ```
   - 特点：角度均匀分布，最常见
   - 适用：大多数工业鱼眼相机

2. **等立体角投影 (Equisolid)**
   ```
   r = 2 * sin(θ/2)
   u = fx * r * (X/r_3d) + cx
   v = fy * r * (Y/r_3d) + cy
   ```
   - 特点：保持立体角均匀性
   - 适用：全景成像、天文观测

3. **正交投影 (Orthographic)**
   ```
   r = sin(θ)
   ```
   - 特点：边缘压缩较强
   - 适用：特殊光学设计

4. **立体投影 (Stereographic)**
   ```
   r = 2 * tan(θ/2)
   ```
   - 特点：保持角度关系，边缘放大
   - 适用：保角变换需求

5. **Kannala-Brandt (OpenCV)**
   ```
   θ_d = θ * (1 + k1*θ² + k2*θ⁴ + k3*θ⁶ + k4*θ⁸)
   r = θ_d
   u = fx * r * (X/r_3d) + cx
   v = fy * r * (Y/r_3d) + cy
   ```
   - 特点：多项式畸变校正，精度最高
   - 适用：OpenCV标定结果

### 10.5 鱼眼相机外参标定建议

如果鱼眼相机与激光雷达之间的外参未知，推荐使用以下方法标定：

**方法1：手眼标定 (Hand-Eye Calibration)**
```bash
# 使用Kalibr工具链
# 1. 采集同时包含Apriltag标定板的图像和雷达数据
# 2. 运行Kalibr标定
kalibr_calibrate_cameras_lidar \
  --bag calibration.bag \
  --topics /camera/image_raw /lidar/points \
  --target april_6x6.yaml \
  --models fisheye

# 输出：T_cam_lidar (相机←雷达变换矩阵)
```

**方法2：公共特征点配准**
```bash
# 1. 在场景中放置3-5个明显特征点（如彩色球）
# 2. 分别在雷达点云和鱼眼图像中标注特征点
# 3. 使用PnP求解外参
python estimate_extrinsic_pnp.py \
  --lidar-points lidar_features.txt \
  --image-pixels image_features.txt \
  --camera-params fisheye_params.json
```

**方法3：经验初值 + 人工微调**
```yaml
# 假设相机在雷达前方10cm，高度对齐，光轴平行
# 初始外参（雷达→相机）：
extrinsic_matrix_4x4: [
  0.0, -1.0, 0.0, 0.0,   # X_cam = -Y_lidar
  0.0,  0.0,-1.0, 0.0,   # Y_cam = -Z_lidar
  1.0,  0.0, 0.0, 0.1,   # Z_cam = X_lidar + 10cm
  0.0,  0.0, 0.0, 1.0
]

# 然后在实际场景中观察投影效果，微调平移参数
```

### 10.6 鱼眼相机性能对比

| 指标 | 针孔相机 | 180度鱼眼 | 优势方 |
|-----|---------|----------|--------|
| **视场覆盖率** | 60-90° | ≥180° | 🐠 鱼眼 |
| **中心区域精度** | 高 | 中 | 📷 针孔 |
| **边缘区域可见性** | 低 | 高 | 🐠 鱼眼 |
| **畸变校正复杂度** | 低 | 高 | 📷 针孔 |
| **计算开销** | 低 | 中 | 📷 针孔 |
| **标定难度** | 低 | 中 | 📷 针孔 |
| **大范围格子监控** | 困难 | 容易 | 🐠 鱼眼 |
| **边缘图像质量** | 好 | 一般 | 📷 针孔 |

**推荐使用场景：**
- **鱼眼相机**：格子架尺寸 ≥ 3x3，或格子分布在相机两侧
- **针孔相机**：格子架尺寸 ≤ 2x2，且都在相机正前方

### 10.7 常见问题与解决方案（鱼眼相机专用）

| 问题 | 症状 | 根本原因 | 解决方案 |
|-----|------|---------|---------|
| **格子中心准确但边缘偏差20+ cm** | 靠近图像边缘的格子整体偏移 | 鱼眼模型选择错误 | 依次尝试5种模型，选择边缘误差最小的 |
| **所有格子整体偏移10cm** | 平移但无变形 | `input_is_cell_centers` 参数错误 | 确认采点方式，修改参数 |
| **边缘格子严重拉伸/压缩** | 格子形状变形 | 畸变参数k1-k4不准确 | 重新标定获取正确k1-k4 |
| **左右两侧格子不对称** | 左边准确右边偏，或反之 | 主点cx偏移 | 微调cx值，或重新标定 |
| **上下两侧格子不对称** | 上边准确下边偏 | 主点cy偏移 | 微调cy值 |
| **所有格子向某方向偏** | 整体平移（非10cm级） | 外参T_c_s平移部分错误 | 重新标定外参或微调xyz |
| **格子翻转/镜像** | 格子位置对称错位 | 外参T_c_s旋转部分错误 | 检查坐标系定义，重新标定 |
| **ROS话题有数据但无投影** | 图像正常但无格子 | 时间戳不同步或外参巨大错误 | 检查时间戳，检查外参数量级 |

### 10.8 进阶：自定义鱼眼投影模型

如果你的鱼眼相机使用非标准投影模型，可以在 `camera_models.py` 中添加自定义类：

```python
class CustomFisheye(FisheyeCamera):
    """自定义鱼眼投影模型"""
    
    def __init__(self, intrinsics: CameraIntrinsics):
        super().__init__(intrinsics, CameraModelType.CUSTOM)
    
    def _theta_to_r(self, theta: float) -> float:
        """
        自定义投影函数：r = f(θ)
        例如：r = a * theta + b * theta^3
        """
        a = 1.0  # 根据相机规格书调整
        b = 0.1
        return a * theta + b * (theta ** 3)
    
    def _r_to_theta(self, r: float) -> float:
        """
        反函数：θ = f^(-1)(r)
        如果无解析解，可使用数值求解
        """
        # 使用牛顿迭代法求解
        theta = r  # 初值
        for _ in range(10):
            r_est = self._theta_to_r(theta)
            # 数值导数
            dr_dtheta = (self._theta_to_r(theta + 1e-6) - r_est) / 1e-6
            theta = theta - (r_est - r) / dr_dtheta
        return theta
```

然后在 `create_camera_model()` 函数中注册新模型。

---

## 11. 参考资料

### 鱼眼相机理论
- Kannala, J., & Brandt, S. S. (2006). "A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses." IEEE TPAMI.
- Scaramuzza, D., et al. (2006). "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion."

### OpenCV鱼眼标定文档
- OpenCV Fisheye Camera Model: https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html
- OpenCV Camera Calibration Tutorial: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

### 工具与数据集
- Kalibr (多相机-IMU标定): https://github.com/ethz-asl/kalibr
- 棋盘格生成器: https://calib.io/pages/camera-calibration-pattern-generator

---

**文档版本：** v2.0 (支持180度鱼眼相机配置)  
**最后更新：** 2026-03-07  
**作者：** AR定位九宫格系统开发团队

如果后续你希望，我可以再补一版“最小调参流程图”（按现场调试顺序：先话题、再外参、再角点、最后畸变模型）。
