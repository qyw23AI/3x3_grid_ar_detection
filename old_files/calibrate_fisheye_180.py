#!/usr/bin/env python3
"""
180度鱼眼相机标定工具
适用镜头：K103A-M12-F2.0 (焦距3.6mm, 180°FOV, -99.57%畸变)

使用步骤：
1. 准备10x7棋盘格标定板（方格30mm x 30mm）
2. 采集20-30张不同角度的标定图像，保存到 calibration_images/ 目录
3. 运行本脚本：python3 calibrate_fisheye_180.py
4. 将生成的 fisheye_calibration.json 配置到 ar_grid.params.yaml
"""

import cv2
import numpy as np
import glob
import json
import os
from pathlib import Path

# ==================== 配置参数 ====================
CHECKERBOARD = (10, 7)      # 棋盘格内角点数量 (列, 行)
SQUARE_SIZE = 0.03          # 方格边长（米）
IMAGE_DIR = "calibration_images"  # 标定图像目录
OUTPUT_JSON = "fisheye_calibration.json"  # 输出配置文件

# ==================== 准备3D标定板坐标 ====================
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D点在世界坐标系
imgpoints = []  # 2D点在图像坐标系

print("="*70)
print("180度鱼眼相机标定工具")
print("="*70)
print(f"棋盘格设置: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} 内角点")
print(f"方格尺寸: {SQUARE_SIZE*1000:.1f}mm x {SQUARE_SIZE*1000:.1f}mm")
print(f"图像目录: {IMAGE_DIR}/")
print("="*70)

# ==================== 加载标定图像 ====================
image_pattern = os.path.join(IMAGE_DIR, "*.jpg")
images = sorted(glob.glob(image_pattern))

if not images:
    image_pattern = os.path.join(IMAGE_DIR, "*.png")
    images = sorted(glob.glob(image_pattern))

if not images:
    print(f"\n❌ 错误：未在 {IMAGE_DIR}/ 目录找到图像文件（.jpg/.png）")
    print("请先采集标定图像，保存到 calibration_images/ 目录")
    exit(1)

print(f"\n找到 {len(images)} 张标定图像")
print("-"*70)

# 创建输出目录
output_dir = os.path.join(IMAGE_DIR, "detected")
os.makedirs(output_dir, exist_ok=True)

# ==================== 提取角点 ====================
img_shape = None
success_count = 0

for idx, fname in enumerate(images):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    if img_shape is None:
        img_shape = gray.shape[::-1]
        print(f"图像分辨率: {img_shape[0]}x{img_shape[1]}")
        print("-"*70)
    
    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + 
        cv2.CALIB_CB_NORMALIZE_IMAGE +
        cv2.CALIB_CB_FAST_CHECK
    )
    
    if ret:
        # 亚像素精度优化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        objpoints.append(objp)
        imgpoints.append(corners_refined)
        success_count += 1
        
        # 绘制检测结果
        img_with_corners = cv2.drawChessboardCorners(img.copy(), CHECKERBOARD, corners_refined, ret)
        
        # 保存检测结果
        output_path = os.path.join(output_dir, f"detected_{idx:03d}.jpg")
        cv2.imwrite(output_path, img_with_corners)
        
        print(f"✓ [{idx+1:2d}/{len(images)}] {os.path.basename(fname):30s} - 角点检测成功")
    else:
        print(f"✗ [{idx+1:2d}/{len(images)}] {os.path.basename(fname):30s} - 角点检测失败")

print("-"*70)
print(f"角点检测完成: {success_count}/{len(images)} 张图像有效")

if success_count < 10:
    print(f"\n⚠️  警告：有效图像数量少于10张，标定结果可能不准确")
    print("建议：")
    print("  - 采集更多不同角度、距离的标定图像")
    print("  - 确保棋盘格占据画面30-80%")
    print("  - 覆盖图像中心、四角、边缘区域")
    response = input("\n是否继续标定？(y/n): ")
    if response.lower() != 'y':
        exit(0)

# ==================== 执行鱼眼标定 ====================
print("\n" + "="*70)
print("开始鱼眼相机标定...")
print("="*70)

K = np.zeros((3, 3))  # 内参矩阵
D = np.zeros((4, 1))  # 畸变系数
rvecs = []            # 旋转向量
tvecs = []            # 平移向量

calibration_flags = (
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +  # 重新计算外参
    cv2.fisheye.CALIB_FIX_SKEW +             # 固定偏斜系数为0
    cv2.fisheye.CALIB_CHECK_COND             # 检查条件数
)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

try:
    ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, img_shape,
        K, D, rvecs, tvecs,
        flags=calibration_flags,
        criteria=criteria
    )
except cv2.error as e:
    print(f"\n❌ 标定失败：{e}")
    print("可能原因：")
    print("  - 标定图像质量不佳")
    print("  - 棋盘格在某些图像中检测不准确")
    print("  - 图像数量不足或角度覆盖不全")
    exit(1)

# ==================== 输出标定结果 ====================
print("\n" + "="*70)
print("✓ 鱼眼相机标定成功")
print("="*70)

print("\n【相机内参矩阵 K】")
print(K)

print("\n【内参参数】")
print(f"  fx (焦距X) = {K[0, 0]:8.2f} 像素")
print(f"  fy (焦距Y) = {K[1, 1]:8.2f} 像素")
print(f"  cx (主点X) = {K[0, 2]:8.2f} 像素  (图像中心: {img_shape[0]/2:.1f})")
print(f"  cy (主点Y) = {K[1, 2]:8.2f} 像素  (图像中心: {img_shape[1]/2:.1f})")

print("\n【畸变系数 D】")
print(D.ravel())
print(f"  k1 (一阶) = {D[0, 0]:9.6f}")
print(f"  k2 (二阶) = {D[1, 0]:9.6f}")
print(f"  k3 (三阶) = {D[2, 0]:9.6f}")
print(f"  k4 (四阶) = {D[3, 0]:9.6f}")

print(f"\n【重投影误差】")
print(f"  RMS error = {ret:.4f} 像素")

if ret < 0.5:
    print("  ✓ 优秀（< 0.5像素）")
elif ret < 1.0:
    print("  ✓ 良好（0.5-1.0像素）")
elif ret < 2.0:
    print("  ⚠️  可接受（1.0-2.0像素），建议增加标定图像")
else:
    print("  ❌ 较差（> 2.0像素），强烈建议重新标定")

# ==================== 计算重投影误差统计 ====================
total_error = 0
errors = []

for i in range(len(objpoints)):
    imgpoints_reprojected, _ = cv2.fisheye.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], K, D
    )
    error = cv2.norm(imgpoints[i], imgpoints_reprojected, cv2.NORM_L2) / len(imgpoints_reprojected)
    errors.append(error)
    total_error += error

mean_error = total_error / len(objpoints)
max_error = max(errors)
min_error = min(errors)

print(f"\n【每张图像的重投影误差】")
print(f"  平均误差: {mean_error:.4f} 像素")
print(f"  最大误差: {max_error:.4f} 像素")
print(f"  最小误差: {min_error:.4f} 像素")

# ==================== 保存配置文件 ====================
config = {
    "camera_model": "fisheye_kannala_brandt",
    "image_width": int(img_shape[0]),
    "image_height": int(img_shape[1]),
    "fx": float(K[0, 0]),
    "fy": float(K[1, 1]),
    "cx": float(K[0, 2]),
    "cy": float(K[1, 2]),
    "k1": float(D[0, 0]),
    "k2": float(D[1, 0]),
    "k3": float(D[2, 0]),
    "k4": float(D[3, 0]),
    "calibration_date": str(np.datetime64('now')),
    "rms_error": float(ret),
    "num_images": success_count
}

with open(OUTPUT_JSON, 'w') as f:
    json.dump(config, f, indent=2)

print("\n" + "="*70)
print(f"✓ 配置已保存到: {OUTPUT_JSON}")
print("="*70)

# ==================== 使用说明 ====================
print("\n【下一步操作】")
print("\n1. 在 ar_grid.params.yaml 中配置：")
print("   ```yaml")
print("   /**:")
print("     ros__parameters:")
print(f'       camera_model: "fisheye_kannala_brandt"')
print(f'       camera_json: "{os.path.abspath(OUTPUT_JSON)}"')
print("   ```")

print("\n2. 或者手动填写参数：")
print("   ```yaml")
print("   /**:")
print("     ros__parameters:")
print(f'       camera_model: "fisheye_kannala_brandt"')
print("       camera:")
print(f"         width: {img_shape[0]}")
print(f"         height: {img_shape[1]}")
print(f"         fx: {K[0, 0]:.2f}")
print(f"         fy: {K[1, 1]:.2f}")
print(f"         cx: {K[0, 2]:.2f}")
print(f"         cy: {K[1, 2]:.2f}")
print(f"         k1: {D[0, 0]:.6f}")
print(f"         k2: {D[1, 0]:.6f}")
print(f"         k3: {D[2, 0]:.6f}")
print(f"         k4: {D[3, 0]:.6f}")
print("   ```")

print("\n3. 启动节点验证标定结果：")
print("   ```bash")
print("   cd /home/r1/Slam/ar_calculate")
print("   colcon build --packages-select ar_grid_detector")
print("   source install/setup.bash")
print("   ros2 launch ar_grid_detector ar_grid.launch.py")
print("   rqt_image_view /ar_grid/image")
print("   ```")

print("\n" + "="*70)
print("标定完成！")
print("="*70)
