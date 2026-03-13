from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Sequence, Tuple, Union
import numpy as np

Point3 = Union[Sequence[float], np.ndarray]
GridSize = Union[float, int, Tuple[float, float], List[float]]


def _to_vec3(p: Point3, name: str) -> np.ndarray:
    arr = np.asarray(p, dtype=np.float64).reshape(-1)
    if arr.size != 3:
        raise ValueError(f"{name} must contain exactly 3 values, got {arr.size}")
    return arr


def _normalize(v: np.ndarray, name: str, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError(f"{name} norm is too small ({n:.3e}), cannot normalize")
    return v / n


def calculate_nine_grid_centers(
    p1: Point3,
    p3: Point3,
    p7: Point3,
    grid_size: GridSize,
    size_mode: str = "cell",
) -> Dict[int, np.ndarray]:
    """
    根据 1/3/7 三个关键点，构建严格正交的九宫格并输出 9 个格子中心点的世界坐标。

    参数:
        p1, p3, p7:
            三个关键点坐标，分别对应 1号(左上)、3号(右上)、7号(左下)格子的中心点，形如 [x, y, z]。
        grid_size:
            九宫格尺寸。
            - size_mode='cell' 时: 代表单格中心间距 L（建议单位: 米）。
            - size_mode='total' 时: 代表整体宽高 (W, H)，对应 1->3 为 W，1->7 为 H。
              若传入标量，则视为 W=H=该值。
        size_mode:
            'cell' 或 'total'。

    返回:
        dict[int, np.ndarray]
            键为 1..9，值为对应格子中心点 np.ndarray([x, y, z])。

    算法:
        1) ux = normalize(p3 - p1)
        2) uz = normalize(ux x (p7 - p1))
        3) uy = normalize(uz x ux)
        4) R = [ux uy uz]
        5) P_world = R @ P_local + p1
    """
    p1 = _to_vec3(p1, "p1")
    p3 = _to_vec3(p3, "p3")
    p7 = _to_vec3(p7, "p7")

    vx = p3 - p1
    vy_temp = p7 - p1

    ux = _normalize(vx, "vx = p3 - p1")
    uz = _normalize(np.cross(ux, vy_temp), "uz = ux x (p7 - p1)")
    uy = _normalize(np.cross(uz, ux), "uy = uz x ux")

    if size_mode not in {"cell", "total"}:
        raise ValueError("size_mode must be 'cell' or 'total'")

    if size_mode == "cell":
        Lx = float(grid_size)
        Ly = float(grid_size)
    else:
        if isinstance(grid_size, (tuple, list, np.ndarray)):
            if len(grid_size) != 2:
                raise ValueError("When size_mode='total', grid_size must be scalar or length-2 sequence")
            total_w, total_h = float(grid_size[0]), float(grid_size[1])
        else:
            total_w = total_h = float(grid_size)
        Lx = total_w / 2.0
        Ly = total_h / 2.0

    if Lx <= 0 or Ly <= 0:
        raise ValueError(f"grid size must be positive, got Lx={Lx}, Ly={Ly}")

    R = np.column_stack((ux, uy, uz))

    local_points = {
        1: np.array([0.0, 0.0, 0.0]),
        2: np.array([Lx, 0.0, 0.0]),
        3: np.array([2.0 * Lx, 0.0, 0.0]),
        4: np.array([0.0, Ly, 0.0]),
        5: np.array([Lx, Ly, 0.0]),
        6: np.array([2.0 * Lx, Ly, 0.0]),
        7: np.array([0.0, 2.0 * Ly, 0.0]),
        8: np.array([Lx, 2.0 * Ly, 0.0]),
        9: np.array([2.0 * Lx, 2.0 * Ly, 0.0]),
    }

    world_points = {idx: (R @ pl + p1) for idx, pl in local_points.items()}
    return world_points


def calculate_nine_grid_centers_as_list(
    p1: Point3,
    p3: Point3,
    p7: Point3,
    grid_size: GridSize,
    size_mode: str = "cell",
) -> List[List[float]]:
    """返回按 1..9 顺序排列的普通 Python list 结果，便于直接序列化/打印。"""
    pts = calculate_nine_grid_centers(p1, p3, p7, grid_size, size_mode=size_mode)
    return [pts[i].tolist() for i in range(1, 10)]


def calculate_nine_grid_centers_from_3_9_7(
    p3: Point3,
    p9: Point3,
    p7: Point3,
    grid_size: GridSize,
    size_mode: str = "cell",
) -> Dict[int, np.ndarray]:
    """
    使用更易采集的三个关键点：3号(右上)、9号(右下)、7号(左下)，输出 1..9 格子中心点。

    说明：
      - x 方向定义为 1->3（从左到右）
      - y 方向定义为 1->7（从上到下）
      - 输入点对应编号固定为 3/9/7
    """
    p3_v = _to_vec3(p3, "p3")
    p9_v = _to_vec3(p9, "p9")
    p7_v = _to_vec3(p7, "p7")

    # 由 3/9/7 反推 1 号点：
    # p3 = p1 + 2Lx*ux
    # p7 = p1 + 2Ly*uy
    # p9 = p1 + 2Lx*ux + 2Ly*uy
    # => p1 = p3 + p7 - p9
    p1_v = p3_v + p7_v - p9_v

    return calculate_nine_grid_centers(
        p1=p1_v,
        p3=p3_v,
        p7=p7_v,
        grid_size=grid_size,
        size_mode=size_mode,
    )


def calculate_nine_grid_centers_from_3_9_7_as_list(
    p3: Point3,
    p9: Point3,
    p7: Point3,
    grid_size: GridSize,
    size_mode: str = "cell",
) -> List[List[float]]:
    """与 calculate_nine_grid_centers_from_3_9_7 相同，但返回 list 格式。"""
    pts = calculate_nine_grid_centers_from_3_9_7(p3, p9, p7, grid_size, size_mode=size_mode)
    return [pts[i].tolist() for i in range(1, 10)]


def save_nine_grid_to_yaml(
    output_path: Union[str, Path],
    p1: Point3,
    p3: Point3,
    p7: Point3,
    grid_size: GridSize,
    size_mode: str = "cell",
) -> Path:
    """
    计算九宫格中心点并保存为 YAML 文件，供后续流程直接读取。

    YAML 结构示例:
        meta:
            size_mode: cell
            grid_size: 0.2
        input_points:
            p1: [x, y, z]
            p3: [x, y, z]
            p7: [x, y, z]
        grid_points:
            1: [x, y, z]
            ...
            9: [x, y, z]
    """
    p1_v = _to_vec3(p1, "p1")
    p3_v = _to_vec3(p3, "p3")
    p7_v = _to_vec3(p7, "p7")
    points = calculate_nine_grid_centers(p1_v, p3_v, p7_v, grid_size, size_mode=size_mode)

    def fmt_list(vals: np.ndarray) -> str:
        return "[" + ", ".join(f"{float(v):.10f}" for v in vals.tolist()) + "]"

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "meta:",
        f"  size_mode: {size_mode}",
        f"  grid_size: {grid_size}",
        "input_points:",
        f"  p1: {fmt_list(p1_v)}",
        f"  p3: {fmt_list(p3_v)}",
        f"  p7: {fmt_list(p7_v)}",
        "grid_points:",
    ]

    for idx in range(1, 10):
        lines.append(f"  {idx}: {fmt_list(points[idx])}")

    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out


def save_nine_grid_to_yaml_from_3_9_7(
    output_path: Union[str, Path],
    p3: Point3,
    p9: Point3,
    p7: Point3,
    grid_size: GridSize,
    size_mode: str = "cell",
) -> Path:
    """使用 3/9/7 三个关键点计算并保存 YAML。"""
    p3_v = _to_vec3(p3, "p3")
    p9_v = _to_vec3(p9, "p9")
    p7_v = _to_vec3(p7, "p7")
    p1_v = p3_v + p7_v - p9_v

    return save_nine_grid_to_yaml(
        output_path=output_path,
        p1=p1_v,
        p3=p3_v,
        p7=p7_v,
        grid_size=grid_size,
        size_mode=size_mode,
    )


def load_nine_grid_config_from_json(json_path: Union[str, Path]) -> Dict[str, object]:
    """
    从 JSON 文件读取九宫格输入配置。

    支持两种点位模式:
      - point_mode = "3-9-7": points 里必须提供 p3/p9/p7
      - point_mode = "1-3-7": points 里必须提供 p1/p3/p7

    JSON 基本结构:
    {
      "point_mode": "3-9-7",
      "size_mode": "cell",
      "grid_size": 0.2,
      "points": {
        "p3": [1.4, 2.0, 0.5],
        "p9": [1.4, 2.4, 0.5],
        "p7": [1.0, 2.4, 0.5]
      }
    }
    """
    path = Path(json_path)
    data = json.loads(path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("JSON root must be an object")

    point_mode = str(data.get("point_mode", "3-9-7")).strip()
    size_mode = str(data.get("size_mode", "cell")).strip()
    grid_size = data.get("grid_size", None)
    points = data.get("points", None)

    if point_mode not in {"1-3-7", "3-9-7"}:
        raise ValueError("point_mode must be '1-3-7' or '3-9-7'")
    if size_mode not in {"cell", "total"}:
        raise ValueError("size_mode must be 'cell' or 'total'")
    if grid_size is None:
        raise ValueError("grid_size is required")
    if not isinstance(points, dict):
        raise ValueError("points must be an object")

    if point_mode == "3-9-7":
        required_keys = ("p3", "p9", "p7")
    else:
        required_keys = ("p1", "p3", "p7")

    for key in required_keys:
        if key not in points:
            raise ValueError(f"points.{key} is required when point_mode='{point_mode}'")

    return {
        "point_mode": point_mode,
        "size_mode": size_mode,
        "grid_size": grid_size,
        "points": points,
    }


def calculate_nine_grid_centers_from_json_as_list(json_path: Union[str, Path]) -> List[List[float]]:
    """从 JSON 配置读取输入并返回 1..9 的中心点 list。"""
    config = load_nine_grid_config_from_json(json_path)
    point_mode = config["point_mode"]
    size_mode = config["size_mode"]
    grid_size = config["grid_size"]
    points = config["points"]

    if point_mode == "3-9-7":
        return calculate_nine_grid_centers_from_3_9_7_as_list(
            p3=points["p3"],
            p9=points["p9"],
            p7=points["p7"],
            grid_size=grid_size,
            size_mode=size_mode,
        )

    return calculate_nine_grid_centers_as_list(
        p1=points["p1"],
        p3=points["p3"],
        p7=points["p7"],
        grid_size=grid_size,
        size_mode=size_mode,
    )


def save_nine_grid_to_yaml_from_json(
    json_path: Union[str, Path],
    output_path: Union[str, Path],
) -> Path:
    """从 JSON 读取输入并保存 YAML 输出。"""
    config = load_nine_grid_config_from_json(json_path)
    point_mode = config["point_mode"]
    size_mode = config["size_mode"]
    grid_size = config["grid_size"]
    points = config["points"]

    if point_mode == "3-9-7":
        return save_nine_grid_to_yaml_from_3_9_7(
            output_path=output_path,
            p3=points["p3"],
            p9=points["p9"],
            p7=points["p7"],
            grid_size=grid_size,
            size_mode=size_mode,
        )

    return save_nine_grid_to_yaml(
        output_path=output_path,
        p1=points["p1"],
        p3=points["p3"],
        p7=points["p7"],
        grid_size=grid_size,
        size_mode=size_mode,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Nine-grid center calculator")
    parser.add_argument(
        "--input-json",
        type=str,
        default=None,
        help="JSON file containing point_mode/size_mode/grid_size/points",
    )
    parser.add_argument(
        "--output-yaml",
        type=str,
        default=None,
        help="Output YAML path. If omitted and --input-json is set, use <json_name>_points.yaml",
    )
    args = parser.parse_args()

    if args.input_json:
        json_file = Path(args.input_json)
        if args.output_yaml:
            out_yaml = Path(args.output_yaml)
        else:
            out_yaml = json_file.with_name(f"{json_file.stem}_points.yaml")

        result = calculate_nine_grid_centers_from_json_as_list(json_file)
        for i, p in enumerate(result, start=1):
            print(f"grid {i}: {p}")

        yaml_path = save_nine_grid_to_yaml_from_json(json_file, out_yaml)
        print(f"saved yaml: {yaml_path}")
        raise SystemExit(0)

    # 示例：使用 3(右上)、9(右下)、7(左下) 三个点
    p3_demo = [1.4, 2.0, 0.5]
    p9_demo = [1.4, 2.4, 0.5]
    p7_demo = [1.0, 2.4, 0.5]

    result = calculate_nine_grid_centers_from_3_9_7_as_list(
        p3=p3_demo,
        p9=p9_demo,
        p7=p7_demo,
        grid_size=0.2,
        size_mode="cell",
    )

    for i, p in enumerate(result, start=1):
        print(f"grid {i}: {p}")

    yaml_path = save_nine_grid_to_yaml_from_3_9_7(
        output_path=Path(__file__).with_name("nine_grid_points.yaml"),
        p3=p3_demo,
        p9=p9_demo,
        p7=p7_demo,
        grid_size=0.2,
        size_mode="cell",
    )
    print(f"saved yaml: {yaml_path}")
