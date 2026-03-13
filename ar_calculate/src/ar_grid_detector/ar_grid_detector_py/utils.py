"""
工具函数模块
Utility Functions Module
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    四元数转旋转矩阵
    
    Args:
        qx, qy, qz, qw: 四元数分量
    
    Returns:
        3x3 旋转矩阵
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is too small")

    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)

    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)

    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22],
    ], dtype=np.float64)


def rot_matrix_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """
    旋转矩阵转四元数
    
    Args:
        R: 3x3 旋转矩阵
    
    Returns:
        (qx, qy, qz, qw) 四元数
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    
    return (qx, qy, qz, qw)


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """
    创建4x4齐次变换矩阵
    
    Args:
        rotation: 3x3 旋转矩阵
        translation: 3x1 平移向量
    
    Returns:
        4x4 变换矩阵
    """
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.flatten()
    return transform


def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    从欧拉角（RPY）创建旋转矩阵
    
    Args:
        roll: 绕X轴旋转角度（弧度）
        pitch: 绕Y轴旋转角度（弧度）
        yaw: 绕Z轴旋转角度（弧度）
    
    Returns:
        3x3 旋转矩阵
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ], dtype=np.float64)
    
    ry = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ], dtype=np.float64)
    
    rz = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    
    return rz @ ry @ rx


def rpy_from_rot(R: np.ndarray) -> Tuple[float, float, float]:
    """
    从旋转矩阵提取欧拉角（RPY）
    
    Args:
        R: 3x3 旋转矩阵
    
    Returns:
        (roll, pitch, yaw) 弧度
    """
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    
    return (roll, pitch, yaw)


def transform_point(transform: np.ndarray, point: np.ndarray) -> np.ndarray:
    """
    使用4x4变换矩阵变换3D点
    
    Args:
        transform: 4x4 变换矩阵
        point: 3D点 [x, y, z]
    
    Returns:
        变换后的3D点 [x, y, z]
    """
    point_h = np.array([point[0], point[1], point[2], 1.0], dtype=np.float64)
    result_h = transform @ point_h
    return result_h[:3]


def inverse_transform(transform: np.ndarray) -> np.ndarray:
    """
    计算4x4变换矩阵的逆
    
    Args:
        transform: 4x4 变换矩阵
    
    Returns:
        逆变换矩阵
    """
    R = transform[:3, :3]
    t = transform[:3, 3]
    
    R_inv = R.T
    t_inv = -R_inv @ t
    
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = R_inv
    result[:3, 3] = t_inv
    
    return result


def point_in_polygon(point: Tuple[float, float], polygon: list) -> bool:
    """
    判断点是否在多边形内（射线法）
    
    Args:
        point: (x, y) 待检测点
        polygon: [(x1,y1), (x2,y2), ...] 多边形顶点
    
    Returns:
        是否在多边形内
    """
    x, y = point
    n = len(polygon)
    inside = False
    
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    
    return inside


def compute_polygon_area(polygon: list) -> float:
    """
    计算多边形面积（鞋带公式）
    
    Args:
        polygon: [(x1,y1), (x2,y2), ...] 多边形顶点
    
    Returns:
        面积（像素平方）
    """
    n = len(polygon)
    if n < 3:
        return 0.0
    
    area = 0.0
    j = n - 1
    for i in range(n):
        area += (polygon[j][0] + polygon[i][0]) * (polygon[j][1] - polygon[i][1])
        j = i
    
    return abs(area) / 2.0


def clip_polygon_to_rect(polygon: list, rect: Tuple[float, float, float, float]) -> list:
    """
    使用 Sutherland-Hodgman 算法将多边形裁剪到矩形区域
    
    Args:
        polygon: [(x1,y1), ...] 多边形顶点
        rect: (x_min, y_min, x_max, y_max) 矩形边界
    
    Returns:
        裁剪后的多边形顶点列表
    """
    x_min, y_min, x_max, y_max = rect
    
    def clip_edge(polygon, x1, y1, x2, y2):
        """对单条边进行裁剪"""
        if not polygon:
            return []
        
        result = []
        for i in range(len(polygon)):
            curr = polygon[i]
            prev = polygon[i - 1]
            
            # 计算点相对于边的位置
            def inside(p):
                return (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1) >= 0
            
            def intersection(p1, p2):
                dc = (p1[0] - p2[0], p1[1] - p2[1])
                dp = (x1 - x2, y1 - y2)
                n1 = (x2 - x1) * (p1[1] - y1) - (y2 - y1) * (p1[0] - x1)
                n2 = dc[0] * dp[1] - dc[1] * dp[0]
                if abs(n2) < 1e-10:
                    return p1
                t = n1 / n2
                return (p1[0] + t * dc[0], p1[1] + t * dc[1])
            
            curr_inside = inside(curr)
            prev_inside = inside(prev)
            
            if curr_inside:
                if not prev_inside:
                    result.append(intersection(prev, curr))
                result.append(curr)
            elif prev_inside:
                result.append(intersection(prev, curr))
        
        return result
    
    # 依次对四条边进行裁剪
    result = polygon
    result = clip_edge(result, x_min, y_min, x_max, y_min)  # 下边
    result = clip_edge(result, x_max, y_min, x_max, y_max)  # 右边
    result = clip_edge(result, x_max, y_max, x_min, y_max)  # 上边
    result = clip_edge(result, x_min, y_max, x_min, y_min)  # 左边
    
    return result
