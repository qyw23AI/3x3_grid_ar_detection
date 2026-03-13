#!/usr/bin/env python3
"""
Launch file for AR grid detector with 180-degree fisheye camera configuration
启动文件：使用180度鱼眼相机配置的AR格子检测器
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ar_grid_detector')
    
    # 使用鱼眼相机专用配置文件（180°鱼眼，USB输入）
    params_file = os.path.join(
        pkg_share, 
        'config', 
        'ar_grid_fisheye_180.params.yaml'
    )
    
    ar_grid_node = Node(
        package='ar_grid_detector',
        executable='ar_grid_node.py',
        name='ar_grid_node',
        output='screen',
        parameters=[params_file],
        # remappings=[
        #     ('/camera/camera/color/image_raw', '/your_fisheye_camera/image_raw'),
        # ]
    )
    
    return LaunchDescription([
        ar_grid_node
    ])
