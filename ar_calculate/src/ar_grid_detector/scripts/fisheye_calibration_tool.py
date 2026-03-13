#!/usr/bin/env python3
"""
180度鱼眼相机OpenCV标定工具
用于获取鱼眼相机内参和畸变参数

使用步骤：
1. 准备棋盘格标定板（建议10x7或更大）
2. 采集20-30张不同角度、位置的标定图像，保存到指定文件夹
3. 运行本脚本，获取标定结果
4. 将结果填入 ar_grid.params.yaml 或保存为 camera_json

依赖：
pip install opencv-python numpy
"""

import cv2
import numpy as np
import glob
import argparse
import json
from pathlib import Path


def calibrate_fisheye(images_path, chessboard_size, square_size, output_json=None):
    """
    执行鱼眼相机标定
    
    Args:
        images_path: 标定图像路径（通配符，如 'calibration_images/*.jpg'）
        chessboard_size: 棋盘格内角点数量 (cols, rows)，例如 (10, 7)
        square_size: 棋盘格方格边长（单位：米），例如 0.03
        output_json: 输出JSON文件路径（可选）
    
    Returns:
        dict: 标定结果字典
    """
    
    print("=" * 70)
    print("180度鱼眼相机标定工具")
    print("=" * 70)
    
    # 准备3D物体点
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    # 存储角点
    objpoints = []  # 3D点
    imgpoints = []  # 2D像素点
    
    # 加载图像
    images = glob.glob(images_path)
    if len(images) == 0:
        print(f"❌ 错误：未找到图像，请检查路径 {images_path}")
        return None
    
    print(f"\n📷 找到 {len(images)} 张图像")
    print(f"🎯 棋盘格配置：{chessboard_size[0]}x{chessboard_size[1]}，方格边长 {square_size*1000}mm")
    print(f"\n正在检测棋盘格角点...\n")
    
    img_shape = None
    valid_count = 0
    
    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            print(f"⚠️  警告：无法读取 {fname}")
            continue
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]
        
        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(
            gray, chessboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS
        )
        
        if ret:
            # 亚像素精细化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            valid_count += 1
            print(f"✅ [{valid_count:2d}] {Path(fname).name}")
        else:
            print(f"❌ [{idx+1:2d}] {Path(fname).name} - 未检测到棋盘格")
    
    if valid_count < 10:
        print(f"\n❌ 错误：有效图像不足（{valid_count}/10），标定可能不准确")
        print("   建议：采集更多不同角度、位置的标定图像")
        return None
    
    print(f"\n✅ 有效标定图像：{valid_count}/{len(images)}")
    
    # 执行鱼眼标定
    print(f"\n🔄 正在执行鱼眼标定（使用OpenCV fisheye模型）...\n")
    
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = []
    tvecs = []
    
    # 标定标志
    calibration_flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC +
        cv2.fisheye.CALIB_FIX_SKEW +
        cv2.fisheye.CALIB_CHECK_COND
    )
    
    # 迭代终止条件
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-6)
    
    try:
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints, imgpoints, img_shape, K, D,
            rvecs, tvecs,
            flags=calibration_flags,
            criteria=criteria
        )
    except cv2.error as e:
        print(f"❌ 标定失败：{e}")
        print("   可能原因：标定图像质量不佳或角点检测不准确")
        return None
    
    # 提取参数
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    k1, k2, k3, k4 = D[0, 0], D[1, 0], D[2, 0], D[3, 0]
    width, height = img_shape
    
    # 输出结果
    print("=" * 70)
    print("📊 标定结果")
    print("=" * 70)
    print(f"✅ RMS重投影误差：{rms:.4f} 像素")
    print(f"   （建议 < 1.0，越小越好）")
    print()
    print(f"📐 图像分辨率：")
    print(f"   width  = {width}")
    print(f"   height = {height}")
    print()
    print(f"🎯 内参矩阵：")
    print(f"   fx = {fx:.4f}")
    print(f"   fy = {fy:.4f}")
    print(f"   cx = {cx:.4f}  （理想值≈{width/2:.1f}）")
    print(f"   cy = {cy:.4f}  （理想值≈{height/2:.1f}）")
    print()
    print(f"🌀 畸变参数（Kannala-Brandt模型）：")
    print(f"   k1 = {k1:.6f}")
    print(f"   k2 = {k2:.6f}")
    print(f"   k3 = {k3:.6f}")
    print(f"   k4 = {k4:.6f}")
    print("=" * 70)
    
    # 评估质量
    print("\n📋 标定质量评估：")
    if rms < 0.5:
        print("   ✅ 优秀（RMS < 0.5）")
    elif rms < 1.0:
        print("   ✅ 良好（RMS < 1.0）")
    elif rms < 2.0:
        print("   ⚠️  可接受（RMS < 2.0），建议重新标定")
    else:
        print("   ❌ 质量不佳（RMS >= 2.0），请重新采集图像")
    
    if abs(cx - width/2) > 50 or abs(cy - height/2) > 50:
        print("   ⚠️  主点偏移较大，请检查相机或重新标定")
    
    # 准备输出结果
    result = {
        "camera_model": "fisheye_kannala_brandt",
        "calibration_rms": float(rms),
        "image_width": int(width),
        "image_height": int(height),
        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),
        "k1": float(k1),
        "k2": float(k2),
        "k3": float(k3),
        "k4": float(k4),
        "p1": 0.0,
        "p2": 0.0
    }
    
    # 输出YAML格式配置
    print("\n" + "=" * 70)
    print("📝 YAML配置片段（复制到ar_grid.params.yaml）：")
    print("=" * 70)
    print(f"""
camera_model: "fisheye_kannala_brandt"
camera_json: ""

camera:
  width: {width}
  height: {height}
  fx: {fx:.4f}
  fy: {fy:.4f}
  cx: {cx:.4f}
  cy: {cy:.4f}
  k1: {k1:.6f}
  k2: {k2:.6f}
  k3: {k3:.6f}
  k4: {k4:.6f}
  p1: 0.0
  p2: 0.0
""")
    
    # 保存JSON
    if output_json:
        with open(output_json, 'w') as f:
            json.dump(result, f, indent=2)
        print(f"✅ 标定结果已保存到：{output_json}")
        print(f"   可通过 camera_json 参数加载")
    
    print("\n" + "=" * 70)
    print("📖 使用建议：")
    print("=" * 70)
    print("1. 将上述YAML配置复制到 ar_grid.params.yaml")
    print("2. 确保 camera_model 设置为 'fisheye_kannala_brandt'")
    print("3. 如果保存了JSON文件，设置 camera_json 路径")
    print("4. 重新编译并启动节点：")
    print("   colcon build --packages-select ar_grid_detector")
    print("   ros2 launch ar_grid_detector ar_grid.launch.py")
    print("=" * 70)
    
    return result


def main():
    parser = argparse.ArgumentParser(
        description='180度鱼眼相机OpenCV标定工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例：
  python fisheye_calibration_tool.py \\
    --images "calibration_images/*.jpg" \\
    --cols 10 --rows 7 \\
    --square-size 0.03 \\
    --output fisheye_params.json

标定流程：
  1. 准备棋盘格标定板（可从网上下载打印，建议A3纸）
  2. 采集20-30张标定图像，覆盖相机各个区域和角度：
     - 中心区域、边缘区域、四个角落
     - 不同倾斜角度
     - 标定板占图像50%-80%
  3. 运行本脚本获取标定参数
  4. 将结果配置到 ar_grid_detector
        """
    )
    
    parser.add_argument(
        '--images', type=str, required=True,
        help='标定图像路径（通配符），例如 "calib/*.jpg"'
    )
    parser.add_argument(
        '--cols', type=int, default=10,
        help='棋盘格列数（内角点数量）'
    )
    parser.add_argument(
        '--rows', type=int, default=7,
        help='棋盘格行数（内角点数量）'
    )
    parser.add_argument(
        '--square-size', type=float, default=0.03,
        help='棋盘格方格边长（单位：米）'
    )
    parser.add_argument(
        '--output', type=str, default=None,
        help='输出JSON文件路径（可选）'
    )
    
    args = parser.parse_args()
    
    result = calibrate_fisheye(
        images_path=args.images,
        chessboard_size=(args.cols, args.rows),
        square_size=args.square_size,
        output_json=args.output
    )
    
    if result is None:
        exit(1)


if __name__ == '__main__':
    main()
