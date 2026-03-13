#!/usr/bin/env python3
"""
九宫格标定快速转换工具
从 JSON 配置文件直接生成九的 YAML 配置

使用方法:
  python3 calibrate_nine_grid.py <json_file> [options]

示例:
  python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json
  python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json --grid-size 0.3
  python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json --output-yaml my_grid.yaml
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Optional


def load_json_config(json_file: str) -> dict:
    """加载 JSON 配置文件"""
    try:
        with open(json_file, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"❌ 文件不存在: {json_file}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"❌ JSON 格式错误: {e}")
        sys.exit(1)


def verify_config(config: dict) -> bool:
    """验证配置文件的有效性"""
    required_keys = ['point_mode', 'size_mode', 'grid_size', 'points']
    
    for key in required_keys:
        if key not in config:
            print(f"❌ 配置文件缺少必要字段: {key}")
            return False
    
    point_mode = config['point_mode']
    points = config['points']
    
    if point_mode == "1-3-7":
        required_points = {'p1', 'p3', 'p7'}
    elif point_mode == "3-9-7":
        required_points = {'p3', 'p9', 'p7'}
    else:
        print(f"❌ 未知的 point_mode: {point_mode}")
        return False
    
    missing = required_points - set(points.keys())
    if missing:
        print(f"❌ 缺少必要的点位: {missing}")
        return False
    
    print("✓ 配置文件验证成功")
    return True


def print_config_summary(config: dict):
    """打印配置摘要"""
    print("\n" + "=" * 60)
    print("           配置摘要")
    print("=" * 60)
    print(f"点位模式: {config['point_mode']}")
    print(f"大小模式: {config['size_mode']}")
    print(f"网格大小: {config['grid_size']}")
    print("\n关键点位:")
    for point_key, coords in config['points'].items():
        print(f"  {point_key}: X={coords[0]:7.4f}  Y={coords[1]:7.4f}  Z={coords[2]:7.4f}")
    
    if 'recorded_at' in config:
        print(f"\n标定时间: {config['recorded_at']}")
    
    if 'all_recorded_points' in config:
        print(f"\n所有已记录的点数: {len(config['all_recorded_points'])}")


def run_nine_grid_script(json_file: str, output_yaml: str, working_dir: Optional[str] = None) -> bool:
    """运行 nine_grid.py 脚本"""
    if working_dir:
        work_dir = Path(working_dir)
    else:
        work_dir = Path.cwd()
    
    nine_grid_script = work_dir / "nine_grid.py"
    
    if not nine_grid_script.exists():
        print(f"❌ 找不到脚本: {nine_grid_script}")
        print(f"   请确保在 AR 计算文件夹中运行此脚本")
        return False
    
    cmd = [
        "python3",
        str(nine_grid_script),
        "--input-json", json_file,
        "--output-yaml", output_yaml
    ]
    
    print(f"\n运行命令:")
    print(f"  {' '.join(cmd)}\n")
    
    try:
        result = subprocess.run(cmd, cwd=str(work_dir), capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"✓ YAML 配置生成成功!")
            print(f"\n输出文件: {work_dir / output_yaml}")
            
            # 显示生成的输出
            if result.stdout:
                print(f"\n脚本输出:")
                print(result.stdout)
            
            return True
        else:
            print(f"❌ 脚本执行失败:")
            print(result.stderr)
            return False
    
    except Exception as e:
        print(f"❌ 执行命令时出错: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="九宫格标定快速转换工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基础用法 (使用 JSON 中的默认网格大小)
  python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json
  
  # 覆盖网格大小
  python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json --grid-size 0.3
  
  # 指定输出文件名
  python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json --output-yaml my_grid.yaml
  
  # 完整配置
  python3 calibrate_nine_grid.py nine_grid_calibration_20260306_165000.json \\
    --grid-size 0.25 \\
    --output-yaml nine_grid_points_final.yaml
        """
    )
    
    parser.add_argument(
        'json_file',
        help='输入的 JSON 配置文件 (由 record_lidar_poses.py 生成)'
    )
    
    parser.add_argument(
        '--grid-size',
        type=float,
        help='覆盖 JSON 中的网格大小 (单位: 米)',
        default=None
    )
    
    parser.add_argument(
        '--output-yaml',
        help='输出 YAML 文件名 (默认: nine_grid_points.yaml)',
        default='nine_grid_points.yaml'
    )
    
    parser.add_argument(
        '--working-dir',
        help='工作目录 (默认: 当前目录)',
        default=None
    )
    
    args = parser.parse_args()
    
    # 加载并验证配置
    print("=" * 60)
    print("       九宫格标定快速转换工具")
    print("=" * 60)
    print(f"\n加载配置文件: {args.json_file}")
    
    config = load_json_config(args.json_file)
    
    if not verify_config(config):
        sys.exit(1)
    
    print_config_summary(config)
    
    # 覆盖网格大小 (如果指定)
    if args.grid_size is not None:
        config['grid_size'] = args.grid_size
        print(f"\n✓ 覆盖网格大小为: {args.grid_size}")
    
    # 确认继续
    print("\n" + "=" * 60)
    confirm = input("是否继续生成 YAML 配置? (y/n): ").strip().lower()
    
    if confirm != 'y':
        print("已取消")
        sys.exit(0)
    
    # 更新 JSON 文件 (如果覆盖了网格大小)
    if args.grid_size is not None:
        with open(args.json_file, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"✓ 已更新 JSON 文件")
    
    # 运行 nine_grid.py
    if run_nine_grid_script(args.json_file, args.output_yaml, args.working_dir):
        print("\n" + "=" * 60)
        print("           标定完成!")
        print("=" * 60)
        print(f"\n✓ YAML 配置文件已生成: {args.output_yaml}")
        print(f"\n后续步骤:")
        print(f"  1. 查看生成的配置:")
        print(f"     cat {args.output_yaml}")
        print(f"\n  2. 启动完整系统 (启用 AR 叠加):")
        print(f"     ENABLE_AR_OVERLAY=1 ./start_all_with_offset.sh")
        print(f"\n  3. 在另一个终端查看 AR 输出:")
        print(f"     ros2 run rqt_image_view rqt_image_view /ar_overlay/image")
    else:
        print("\n❌ 配置生成失败，请检查错误信息")
        sys.exit(1)


if __name__ == '__main__':
    main()
