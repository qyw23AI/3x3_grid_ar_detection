#!/usr/bin/env python3
"""
雷达位姿录制工具（可变点数采点）

功能：
1. 订阅 /aft_mapped_in_map（Odometry）
2. 按用户指定顺序采集任意数量的点
3. 每个点都要求终端确认后才开始采样
4. 每个点采样 n 次（可调），并执行异常值剔除后求均值
5. 输出 JSON 文件

示例：
  python3 record_lidar_poses.py --samples 6 --interval 0.08
"""

import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, TypedDict

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PoseData(TypedDict):
    position: List[float]
    quaternion: List[float]
    frame_id: str
    timestamp: float


class LidarPoseRecorder(Node):
    def __init__(self, topic_name: str = "/aft_mapped_in_map"):
        super().__init__('lidar_pose_recorder')

        # 仅订阅 Odometry 类型的位姿消息
        self.odom_subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odom_callback,
            10,
        )
        self.get_logger().info("已使用 Odometry 创建订阅")
        self.topic_name = topic_name
        self.current_pose: Optional[PoseData] = None
        self.recorded_poses: Dict[int, List[float]] = {}

        self.get_logger().info("=" * 60)
        self.get_logger().info("   雷达位姿录制工具（可变点数采点）")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"订阅话题: {self.topic_name}")
        self.get_logger().info("")

    # 已移除 PoseStamped 回调，当前仅使用 Odometry 回调（`odom_callback`）

    def odom_callback(self, msg: Odometry):
        """接收 Odometry 消息并提取位姿（兼容性处理）。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.current_pose = {
            'position': [p.x, p.y, p.z],
            'quaternion': [q.x, q.y, q.z, q.w],
            'frame_id': msg.header.frame_id,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        }

    def get_current_pose(self) -> Optional[PoseData]:
        """获取当前位姿。"""
        if self.current_pose is None:
            self.get_logger().warn("⚠️  还未收到位姿信息，请确保 SLAM 正常发布 Odometry")
            return None
        return self.current_pose

    def print_current_pose(self):
        """打印当前位姿。"""
        pose = self.get_current_pose()
        if pose:
            pos = pose['position']
            self.get_logger().info(f"当前位置: X={pos[0]:7.4f}  Y={pos[1]:7.4f}  Z={pos[2]:7.4f}")

    @staticmethod
    def _reject_outliers_mad(samples: np.ndarray, z_thresh: float = 2.5) -> np.ndarray:
        """基于 MAD 的鲁棒异常值剔除。"""
        # 样本太少时，MAD 和鲁棒统计都不稳定，直接返回原始数据。
        if samples.shape[0] < 4:
            return samples

        # 先按“每一列一个维度”的方式求中位数，得到一个鲁棒中心点。
        med = np.median(samples, axis=0)
        # 计算每个样本到中位数中心的欧氏距离，作为离群程度的基础。
        dist = np.linalg.norm(samples - med, axis=1)
        # 计算距离序列的 MAD（Median Absolute Deviation），衡量“距离值”本身的离散程度。
        mad = np.median(np.abs(dist - np.median(dist)))

        # 如果 MAD 接近 0，说明样本非常集中，无法再可靠地区分异常值。
        if mad < 1e-12:
            return samples

        # 将距离转换为鲁棒 z 分数：数值越大，越可能是异常点。
        robust_z = 0.6745 * (dist - np.median(dist)) / mad
        # 保留鲁棒 z 分数绝对值不超过阈值的样本。
        keep_mask = np.abs(robust_z) <= z_thresh
        # 根据掩码过滤掉异常样本。
        filtered = samples[keep_mask]

        # 如果过滤后样本过少，说明剔除过于激进；此时宁可保留原始样本。
        if filtered.shape[0] < max(3, samples.shape[0] // 2):
            return samples
        # 返回过滤后的鲁棒样本集合。
        return filtered

    def collect_point_average(
        self,
        point_id: int,
        sample_count: int,
        sample_interval_sec: float,
        outlier_z_thresh: float,
    ) -> Tuple[bool, Optional[List[float]], Dict[str, object]]:
        """采集单个点的 n 次样本，做异常值剔除并返回均值。"""
        # 用于保存当前点的原始采样序列，每个元素都是 [x, y, z]。
        raw_samples: List[List[float]] = []

        # 循环采样 sample_count 次，每次采样都读取当前最新位姿。
        for _ in range(sample_count):
            # 获取当前订阅到的最新位姿。
            pose = self.get_current_pose()
            if pose is None:
                # 如果没有位姿，直接返回失败，并说明已经采了多少次。
                return False, None, {
                    "reason": "no_pose",
                    "raw_count": len(raw_samples),
                    "kept_count": 0,
                }
            # 只保存位姿中的位置部分，不把四元数用于均值计算。
            raw_samples.append([float(v) for v in pose['position']])
            # 等待一段时间后再采下一次，形成多次独立采样。
            time.sleep(sample_interval_sec)

        # 将 Python 列表转换为 NumPy 数组，方便后续做统计计算。
        raw_np = np.asarray(raw_samples, dtype=np.float64)
        # 调用 MAD 方法剔除异常值，得到更稳定的有效样本集。
        filtered_np = self._reject_outliers_mad(raw_np, z_thresh=outlier_z_thresh)
        # 对过滤后的样本求均值，作为该点最终代表位姿。
        mean_xyz = filtered_np.mean(axis=0)

        # 将结果转回普通 Python 列表，便于 JSON 序列化。
        result = [float(mean_xyz[0]), float(mean_xyz[1]), float(mean_xyz[2])]
        # 保存该点最终结果，供后续 JSON 输出使用。
        self.recorded_poses[point_id] = result

        # 记录采样统计信息，便于检查剔除效果和数据稳定性。
        stats = {
            "raw_count": int(raw_np.shape[0]),
            "kept_count": int(filtered_np.shape[0]),
            "rejected_count": int(raw_np.shape[0] - filtered_np.shape[0]),
            # 原始样本在三个坐标轴上的标准差，反映抖动程度。
            "raw_std_xyz": [float(v) for v in raw_np.std(axis=0)],
            # 剔除异常值后样本的标准差，通常应更小，更能代表稳定结果。
            "kept_std_xyz": [float(v) for v in filtered_np.std(axis=0)],
        }
        # 返回成功标志、最终均值、以及统计信息。
        return True, result, stats

    def generate_json_config(
        self,
        sample_count: int,
        sample_interval_sec: float,
        outlier_z_thresh: float,
        per_point_stats: Dict[int, Dict[str, object]],
        output_file: Optional[str] = None,
    ) -> Optional[str]:
        """生成并保存 JSON 配置。"""
        # 不再强制要求特定点，而是使用所有记录的点
        if not self.recorded_poses:
            self.get_logger().error("❌ 没有记录任何点，无法生成配置文件")
            return None

        # 按照点ID顺序生成点字典
        sorted_points = sorted(self.recorded_poses.keys())
        points_dict = {f'p{p}': self.recorded_poses[p] for p in sorted_points}
        
        config = {
            "point_mode": "custom",
            "total_points": len(sorted_points),
            "topic": self.topic_name,
            "sampling": {
                "samples": int(sample_count),
                "interval_sec": float(sample_interval_sec),
                "outlier_z_thresh": float(outlier_z_thresh),
            },
            "points": points_dict,
            "recorded_at": datetime.now().isoformat(),
            "per_point_stats": {
                f"p{k}": v for k, v in per_point_stats.items()
            }
        }

        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"grid_calibration_{timestamp}.json"

        output_path = Path(output_file)

        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2)
            self.get_logger().info(f"✓ 配置文件已生成: {output_path.absolute()}")
            return str(output_path.absolute())
        except Exception as e:
            self.get_logger().error(f"❌ 写入配置文件失败: {e}")
            return None


def _ask_confirm(prompt: str) -> bool:
    ans = input(prompt).strip().lower()
    return ans in ('y', 'yes')


def _wait_pose_ready(recorder: LidarPoseRecorder, wait_sec: float) -> bool:
    print("\n等待位姿消息...", end="", flush=True)
    t0 = time.time()
    while time.time() - t0 < wait_sec:
        if recorder.current_pose is not None:
            print(" ✓")
            return True
        time.sleep(0.05)
    print(" ✗")
    return False


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="可变点数的雷达位姿采点工具")
    parser.add_argument("--topic", default="/aft_mapped_in_map", help="位姿订阅话题（例如 Odometry）")
    parser.add_argument("--samples", type=int, default=6, help="每个点采样次数 n")
    parser.add_argument("--interval", type=float, default=0.08, help="采样间隔（秒）")
    parser.add_argument("--outlier-z", type=float, default=2.5, help="MAD 异常值剔除阈值")
    parser.add_argument("--startup-timeout", type=float, default=5.0, help="等待首帧位姿超时（秒）")
    parser.add_argument("--output", default="", help="输出 json 文件路径（默认自动命名）")
    return parser.parse_args()


def main():
    args = _parse_args()
    if args.samples < 3:
        raise ValueError("--samples 必须 >= 3")
    if args.interval <= 0.0:
        raise ValueError("--interval 必须 > 0")
    if args.outlier_z <= 0.0:
        raise ValueError("--outlier-z 必须 > 0")

    rclpy.init()
    recorder = LidarPoseRecorder(topic_name=args.topic)
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    stop_event = threading.Event()
    ros_thread = None

    try:
        def spin_loop():
            while rclpy.ok() and not stop_event.is_set():
                executor.spin_once(timeout_sec=0.1)

        ros_thread = threading.Thread(target=spin_loop)
        ros_thread.start()

        if not _wait_pose_ready(recorder, wait_sec=float(args.startup_timeout)):
            print("\n❌ 无法接收位姿消息，请检查话题与 SLAM 发布状态")
            return

        recorded_points = []
        per_point_stats: Dict[int, Dict[str, object]] = {}
        point_counter = 1

        print("\n开始采集点位姿，您可以采集任意数量的点")
        print(f"每点采样次数: {args.samples}, 采样间隔: {args.interval:.3f}s, 异常阈值: {args.outlier_z}")

        while True:
            print("\n" + "=" * 60)
            print(f"请将雷达移动到点 {point_counter} 位置")
            

            if not _ask_confirm(f"确认开始采集点 {point_counter} ? (y/n): "):
                if _ask_confirm("放弃本次采集并退出? (y/n): "):
                    print("已取消采集")
                    return
                recorder.print_current_pose()
                continue

            ok, averaged_xyz, stats = recorder.collect_point_average(
                point_id=point_counter,
                sample_count=int(args.samples),
                sample_interval_sec=float(args.interval),
                outlier_z_thresh=float(args.outlier_z),
            )
            if not ok or averaged_xyz is None:
                print(f"❌ 点 {point_counter} 采样失败：{stats}")
                return

            per_point_stats[point_counter] = stats
            print(
                f"✓ 点 {point_counter} 完成: avg=[{averaged_xyz[0]:.6f}, {averaged_xyz[1]:.6f}, {averaged_xyz[2]:.6f}] "
                f"(raw={stats['raw_count']}, kept={stats['kept_count']}, rejected={stats['rejected_count']})"
            )

            # 询问是否采集下一个点
            if not _ask_confirm("是否采集下一个点? (y/n): "):
                print("结束采集")
                break

            point_counter += 1

        output_file = args.output.strip() if args.output else None
        saved = recorder.generate_json_config(
            sample_count=int(args.samples),
            sample_interval_sec=float(args.interval),
            outlier_z_thresh=float(args.outlier_z),
            per_point_stats=per_point_stats,
            output_file=output_file,
        )
        if saved is None:
            print("❌ JSON 生成失败")
            return

        print("\n采集完成。")
        print(f"JSON 文件: {saved}")

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
