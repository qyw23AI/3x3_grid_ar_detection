#!/usr/bin/env python3
"""
雷达位姿录制工具
用于通过 SLAM 实时标定九宫格关键点位置

使用方法:
1. 启动 SLAM 系统: ./start_all_with_offset.sh
2. 运行此脚本: python3 record_lidar_poses.py
3. 按照提示将雷达移动到九宫格关键点位置
4. 根据选择的点位模式，记录该模式需要的所有关键点
5. 脚本自动生成 JSON 配置文件
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
import json
from pathlib import Path
from datetime import datetime
from typing import Optional
import threading
import time


class LidarPoseRecorder(Node):
    def __init__(self):
        super().__init__('lidar_pose_recorder')
        
        # 订阅里程计话题
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aft_mapped_to_init',
            self.pose_callback,
            10
        )
        
        self.current_pose = None
        self.recorded_poses = {}
        self.grid_points = {1, 2, 3, 4, 5, 6, 7, 8, 9}
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("         雷达位姿录制工具")
        self.get_logger().info("=" * 60)
        self.get_logger().info("订阅话题: /aft_mapped_to_init")
        self.get_logger().info("")
        
    def pose_callback(self, msg: PoseStamped):
        """接收位姿更新"""
        # 提取平移和旋转
        pos = msg.pose.position
        self.current_pose = {
            'position': [pos.x, pos.y, pos.z],
            'quaternion': [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ],
            'frame_id': msg.header.frame_id,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        
    def get_current_pose(self):
        """获取当前位姿"""
        if self.current_pose is None:
            self.get_logger().warn("⚠️  还未收到位姿信息，请确保 SLAM 系统正常运行")
            return None
        return self.current_pose
    
    def print_current_pose(self):
        """打印当前位姿"""
        pose = self.get_current_pose()
        if pose:
            pos = pose['position']
            self.get_logger().info(f"当前位置: X={pos[0]:7.4f}  Y={pos[1]:7.4f}  Z={pos[2]:7.4f}")
    
    def record_point(self, point_id: int):
        """记录一个点的位姿"""
        pose = self.get_current_pose()
        if pose is None:
            return False
        
        self.recorded_poses[point_id] = pose['position']
        self.get_logger().info(f"✓ 已记录点 {point_id}: {pose['position']}")
        return True
    
    def generate_json_config(
        self,
        point_mode: str,
        grid_size: float,
        size_mode: str = "cell",
        output_file: Optional[str] = None,
    ):
        """
        生成 JSON 配置文件
        
        Args:
            point_mode: "1-3-7" 或 "3-9-7"
            grid_size: 网格大小（米）
            size_mode: "cell" 或 "total"
            output_file: 输出文件路径
        """
        if point_mode not in ["1-3-7", "3-9-7"]:
            self.get_logger().error(f"❌ point_mode 必须是 '1-3-7' 或 '3-9-7'")
            return False
        
        # 确定需要的点
        if point_mode == "1-3-7":
            required_points = [1, 3, 7]
        else:  # "3-9-7"
            required_points = [3, 9, 7]
        
        # 检查是否都已记录
        missing = [p for p in required_points if p not in self.recorded_poses]
        if missing:
            self.get_logger().error(f"❌ 缺少点 {missing}，无法生成配置")
            return False
        
        # 生成 JSON 数据
        points_dict = {}
        for p in required_points:
            points_dict[f'p{p}'] = self.recorded_poses[p]
        
        config = {
            "point_mode": point_mode,
            "size_mode": size_mode,
            "grid_size": grid_size,
            "points": points_dict,
            "recorded_at": datetime.now().isoformat(),
            "all_recorded_points": {
                f'p{k}': v for k, v in self.recorded_poses.items()
            }
        }
        
        # 确定输出文件
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"nine_grid_calibration_{timestamp}.json"
        
        output_path = Path(output_file)
        
        # 写入 JSON 文件
        try:
            with open(output_path, 'w') as f:
                json.dump(config, f, indent=2)
            self.get_logger().info(f"✓ 配置文件已生成: {output_path.absolute()}")
            return str(output_path.absolute())
        except Exception as e:
            self.get_logger().error(f"❌ 写入配置文件失败: {e}")
            return False
    
    def print_recorded_points(self):
        """打印已记录的所有点"""
        if not self.recorded_poses:
            self.get_logger().warn("⚠️  还未记录任何点")
            return
        
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("           已记录的点位")
        self.get_logger().info("=" * 60)
        for point_id in sorted(self.recorded_poses.keys()):
            pos = self.recorded_poses[point_id]
            self.get_logger().info(
                f"点 {point_id}: X={pos[0]:7.4f}  Y={pos[1]:7.4f}  Z={pos[2]:7.4f}"
            )


def interactive_mode(recorder: LidarPoseRecorder):
    """交互模式"""
    while True:
        print("\n" + "=" * 60)
        print("          选择操作")
        print("=" * 60)
        print("1. 查看当前位姿")
        print("2. 记录点位 (输入点号 1-9)")
        print("3. 查看已记录点位")
        print("4. 生成配置文件")
        print("5. 退出")
        
        choice = input("\n请输入选项 (1-5): ").strip()
        
        if choice == "1":
            recorder.print_current_pose()
        
        elif choice == "2":
            try:
                point_id = int(input("输入点号 (1-9): "))
                if 1 <= point_id <= 9:
                    recorder.print_current_pose()
                    confirm = input(f"确认记录点 {point_id}? (y/n): ").strip().lower()
                    if confirm == 'y':
                        recorder.record_point(point_id)
                else:
                    print("❌ 点号范围应在 1-9 之间")
            except ValueError:
                print("❌ 输入格式错误")
        
        elif choice == "3":
            recorder.print_recorded_points()
        
        elif choice == "4":
            print("\n选择点位模式:")
            print("  1: 使用 1-3-7 (左上、右上、左下)")
            print("  2: 使用 3-9-7 (右上、右下、左下)")
            mode_choice = input("请输入 (1 或 2): ").strip()
            
            if mode_choice == "1":
                point_mode = "1-3-7"
            elif mode_choice == "2":
                point_mode = "3-9-7"
            else:
                print("❌ 输入无效")
                continue
            
            try:
                grid_size = float(input(f"输入网格大小 (保持 cell 模式，单位米): "))
                output_file = input("输入输出文件名 (默认自动生成): ").strip()
                if not output_file:
                    output_file = None
                
                result = recorder.generate_json_config(point_mode, grid_size, output_file=output_file)
                if result:
                    print(f"\n后续步骤:")
                    print(f"  1. 查看生成的 JSON 文件: {result}")
                    print(f"  2. 运行命令生成 YAML:")
                    print(f"     python3 nine_grid.py \\")
                    print(f"       --input-json {result} \\")
                    print(f"       --output-yaml nine_grid_points.yaml")
            except ValueError:
                print("❌ 输入格式错误")
        
        elif choice == "5":
            print("\n退出程序")
            break
        
        else:
            print("❌ 选项无效，请输入 1-5")


def main():
    rclpy.init()
    recorder = LidarPoseRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    stop_event = threading.Event()
    ros_thread = None
    
    # 运行交互模式
    try:
        # 在单独线程中运行可控执行器，便于安全退出
        def spin_loop():
            while rclpy.ok() and not stop_event.is_set():
                executor.spin_once(timeout_sec=0.1)

        ros_thread = threading.Thread(target=spin_loop)
        ros_thread.start()
        
        # 等待第一个位姿消息
        print("\n等待位姿消息...", end="", flush=True)
        for _ in range(30):
            if recorder.current_pose is not None:
                print(" ✓")
                break
            time.sleep(0.1)
        
        if recorder.current_pose is None:
            print("\n❌ 无法接收位姿消息，请确保 SLAM 系统正常运行")
            return
        
        # 进入交互模式
        interactive_mode(recorder)
    
    except KeyboardInterrupt:
        print("\n\n程序被中断")
    finally:
        stop_event.set()
        if ros_thread is not None and ros_thread.is_alive():
            ros_thread.join(timeout=1.0)
        executor.remove_node(recorder)
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
