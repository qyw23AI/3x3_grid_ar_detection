#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
import json
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Point32
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ar_grid_detector_py.camera_models import CameraModelType, create_camera_from_params
from ar_grid_detector_py.grid_generator import (
    GridFrame,
    GridFrameGenerator,
    convert_cell_centers_to_corners,
    convert_legacy_nine_grid_points,
)
from ar_grid_detector.msg import GridCell, GridCellArray
from ar_grid_detector_py.utils import (
    inverse_transform,
    make_transform,
    quat_to_rot_matrix,
    rot_from_rpy,
)


class ArGridNode(Node):
    def __init__(self) -> None:
        super().__init__("ar_grid_node")

        self._declare_params()
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        self._latest_t_w_s = np.eye(4, dtype=np.float64)
        self._latest_odom_time = None
        self._odom_received_once = False
        self._window_error_logged = False
        self._last_diag_log_time = 0.0
        self._last_pose_age_warn_time = 0.0
        self._odom_history = deque(maxlen=int(self.get_parameter("odom_sync.max_history").value))

        self.t_c_s = self._load_extrinsic_t_c_s()
        self.camera_model = self._load_camera_model()
        self.grid_frame = self._load_or_generate_grid_frame()

        odom_topic = str(self.get_parameter("odom_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        visible_cells_topic = str(self.get_parameter("visible_cells_topic").value)

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, output_image_topic, 1)
        self.visible_cells_pub = self.create_publisher(GridCellArray, visible_cells_topic, 5)

        self.get_logger().info(f"Camera model: {self.get_parameter('camera_model').value}")
        self.get_logger().info(f"Grid size: rows={self.grid_frame.rows}, cols={self.grid_frame.cols}, total={self.grid_frame.total_cells}")
        self.get_logger().info(f"Subscribed odom={odom_topic}, image={image_topic}")
        self.get_logger().info(f"Publishing overlay image to {output_image_topic}")
        self.get_logger().info(f"Publishing visible cells to {visible_cells_topic}")

    def _declare_params(self) -> None:
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("image_topic", "/fisheye_camera/image_raw")
        self.declare_parameter("output_image_topic", "/ar_grid/image")
        self.declare_parameter("visible_cells_topic", "/ar_grid/visible_cells")

        self.declare_parameter("camera_model", "pinhole")
        self.declare_parameter("camera_json", "")
        self.declare_parameter("camera.fx", 606.5)
        self.declare_parameter("camera.fy", 605.77)
        self.declare_parameter("camera.cx", 325.77)
        self.declare_parameter("camera.cy", 256.54)
        self.declare_parameter("camera.width", 640)
        self.declare_parameter("camera.height", 480)
        self.declare_parameter("camera.k1", 0.0)
        self.declare_parameter("camera.k2", 0.0)
        self.declare_parameter("camera.k3", 0.0)
        self.declare_parameter("camera.k4", 0.0)
        self.declare_parameter("camera.p1", 0.0)
        self.declare_parameter("camera.p2", 0.0)

        self.declare_parameter("grid.use_yaml", False)
        self.declare_parameter("grid.yaml_path", "")
        self.declare_parameter("grid.use_legacy_nine_points", False)
        self.declare_parameter("grid.legacy_nine_points_yaml", "/home/r1/Slam/ar_calculate/nine_grid_points.yaml")
        self.declare_parameter("grid.rows", 3)
        self.declare_parameter("grid.cols", 3)
        self.declare_parameter("grid.cell_width", 0.3)
        self.declare_parameter("grid.cell_height", 0.3)
        self.declare_parameter("grid.strict_size_check", False)
        self.declare_parameter("grid.size_tolerance", 1e-4)
        self.declare_parameter("grid.corner_top_left", [0.0, 0.0, 0.0])
        self.declare_parameter("grid.corner_top_right", [0.9, 0.0, 0.0])
        self.declare_parameter("grid.corner_bottom_left", [0.0, 0.9, 0.0])
        self.declare_parameter("grid.input_is_cell_centers", False)

        self.declare_parameter("draw.cell_border_color_bgr", [255, 0, 0])
        self.declare_parameter("draw.cell_border_thickness", 2)
        self.declare_parameter("draw.cell_center_color_bgr", [0, 0, 255])
        self.declare_parameter("draw.cell_center_radius", 4)
        self.declare_parameter("draw.cell_center_thickness", -1)
        self.declare_parameter("draw.label_color_bgr", [0, 255, 255])
        self.declare_parameter("draw.show_labels", True)
        self.declare_parameter("draw.show_status_text", True)
        self.declare_parameter("draw.show_window", True)
        self.declare_parameter("draw.window_name", "AR Grid Overlay")
        self.declare_parameter("draw.curve_samples_per_edge", 16)

        self.declare_parameter("extrinsic_rpy_xyz", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("extrinsic_matrix_4x4", Parameter.Type.DOUBLE_ARRAY)

        self.declare_parameter("odom_sync.use_closest_by_stamp", True)
        self.declare_parameter("odom_sync.max_history", 400)
        self.declare_parameter("odom_sync.max_pose_age_sec", 0.25)

    def _load_camera_model(self):
        camera_json = str(self.get_parameter("camera_json").value).strip()
        if camera_json:
            self._load_camera_from_json(Path(camera_json))

        model_str = str(self.get_parameter("camera_model").value).strip().lower()
        return create_camera_from_params(
            model_str,
            fx=float(self.get_parameter("camera.fx").value),
            fy=float(self.get_parameter("camera.fy").value),
            cx=float(self.get_parameter("camera.cx").value),
            cy=float(self.get_parameter("camera.cy").value),
            width=int(self.get_parameter("camera.width").value),
            height=int(self.get_parameter("camera.height").value),
            k1=float(self.get_parameter("camera.k1").value),
            k2=float(self.get_parameter("camera.k2").value),
            k3=float(self.get_parameter("camera.k3").value),
            k4=float(self.get_parameter("camera.k4").value),
            p1=float(self.get_parameter("camera.p1").value),
            p2=float(self.get_parameter("camera.p2").value),
        )

    def _is_fisheye_model_selected(self) -> bool:
        model_str = str(self.get_parameter("camera_model").value).strip().lower()
        return model_str.startswith("fisheye_")

    def _draw_sampled_world_edge(
        self,
        image: np.ndarray,
        start_w: np.ndarray,
        end_w: np.ndarray,
        t_c_w: np.ndarray,
        color: Tuple[int, int, int],
        thickness: int,
        samples_per_edge: int,
    ) -> None:
        samples = max(2, int(samples_per_edge))
        points: List[Tuple[int, int]] = []

        for i in range(samples + 1):
            alpha = float(i) / float(samples)
            p_w = (1.0 - alpha) * start_w + alpha * end_w
            p_w_h = np.array([p_w[0], p_w[1], p_w[2], 1.0], dtype=np.float64)
            p_c_h = t_c_w @ p_w_h
            p_c = p_c_h[:3]

            if p_c[2] <= 1e-6:
                continue

            uv = self.camera_model.project_point(p_c)
            if uv is None:
                continue

            pt = (int(round(uv[0])), int(round(uv[1])))
            if not points or pt != points[-1]:
                points.append(pt)

        for idx in range(len(points) - 1):
            cv2.line(
                image,
                points[idx],
                points[idx + 1],
                color,
                thickness,
                lineType=cv2.LINE_AA,
            )

    def _load_camera_from_json(self, json_path: Path) -> None:
        if not json_path.exists():
            raise FileNotFoundError(f"camera_json not found: {json_path}")

        data = json.loads(json_path.read_text(encoding="utf-8"))
        camera_matrix = data.get("camera_matrix", {})
        distortion = data.get("distortion_coefficients", {})

        if "fx" in camera_matrix:
            fx = camera_matrix["fx"]
            fy = camera_matrix["fy"]
            cx = camera_matrix["cx"]
            cy = camera_matrix["cy"]
        elif "data" in camera_matrix:
            m = np.asarray(camera_matrix["data"], dtype=np.float64).reshape(3, 3)
            fx, fy, cx, cy = float(m[0, 0]), float(m[1, 1]), float(m[0, 2]), float(m[1, 2])
        else:
            raise ValueError("camera_json missing camera_matrix")

        self.set_parameters([
            Parameter("camera.fx", value=float(fx)),
            Parameter("camera.fy", value=float(fy)),
            Parameter("camera.cx", value=float(cx)),
            Parameter("camera.cy", value=float(cy)),
        ])

        image_width = data.get("image_width")
        image_height = data.get("image_height")
        if image_width is not None and image_height is not None:
            self.set_parameters([
                Parameter("camera.width", value=int(image_width)),
                Parameter("camera.height", value=int(image_height)),
            ])

        dist_data = distortion.get("data", [])
        if len(dist_data) >= 4:
            self.set_parameters([
                Parameter("camera.k1", value=float(dist_data[0])),
                Parameter("camera.k2", value=float(dist_data[1])),
                Parameter("camera.k3", value=float(dist_data[2])),
                Parameter("camera.k4", value=float(dist_data[3])),
            ])
            if len(dist_data) >= 6:
                self.set_parameters([
                    Parameter("camera.p1", value=float(dist_data[4])),
                    Parameter("camera.p2", value=float(dist_data[5])),
                ])

    def _load_extrinsic_t_c_s(self) -> np.ndarray:
        # 外参语义：T_c_s 表示“传感器系 -> 相机系”的刚体变换。
        # 后续投影统一使用：P_c = T_c_s * T_s_w * P_w。
        # 若这里的坐标轴约定和里程计/相机约定不一致，会造成整体偏移、旋转或镜像。
        try:
            matrix_raw = list(self.get_parameter("extrinsic_matrix_4x4").value)
        except ParameterUninitializedException:
            matrix_raw = []

        if len(matrix_raw) == 16:
            return np.asarray(matrix_raw, dtype=np.float64).reshape(4, 4)
        if len(matrix_raw) not in (0,):
            raise ValueError("extrinsic_matrix_4x4 must have 16 numbers or be empty")

        rpy_xyz_raw = list(self.get_parameter("extrinsic_rpy_xyz").value)
        if len(rpy_xyz_raw) != 6:
            raise ValueError("extrinsic_rpy_xyz must have 6 numbers: [roll, pitch, yaw, x, y, z]")

        roll, pitch, yaw, x, y, z = [float(v) for v in rpy_xyz_raw]
        rot = rot_from_rpy(roll, pitch, yaw)
        trans = np.array([x, y, z], dtype=np.float64)
        return make_transform(rot, trans)

    def _load_or_generate_grid_frame(self) -> GridFrame:
        use_legacy_nine_points = bool(self.get_parameter("grid.use_legacy_nine_points").value)
        if use_legacy_nine_points:
            legacy_yaml_path = Path(str(self.get_parameter("grid.legacy_nine_points_yaml").value))
            return self._load_grid_from_legacy_yaml(legacy_yaml_path)

        use_yaml = bool(self.get_parameter("grid.use_yaml").value)
        if use_yaml:
            yaml_path_str = str(self.get_parameter("grid.yaml_path").value).strip()
            if yaml_path_str:
                yaml_path = Path(yaml_path_str)
                if yaml_path.is_file():
                    return self._load_grid_from_yaml(yaml_path)
                self.get_logger().warn(
                    f"grid.use_yaml=true but grid.yaml_path is not a file: '{yaml_path}'. "
                    "Falling back to three-corner grid generation."
                )
            else:
                self.get_logger().warn(
                    "grid.use_yaml=true but grid.yaml_path is empty. "
                    "Falling back to three-corner grid generation."
                )

        rows = int(self.get_parameter("grid.rows").value)
        cols = int(self.get_parameter("grid.cols").value)
        cell_width = float(self.get_parameter("grid.cell_width").value)
        cell_height = float(self.get_parameter("grid.cell_height").value)
        strict_size_check = bool(self.get_parameter("grid.strict_size_check").value)
        size_tolerance = float(self.get_parameter("grid.size_tolerance").value)
        input_is_cell_centers = bool(self.get_parameter("grid.input_is_cell_centers").value)
        corner_top_left = np.asarray(self.get_parameter("grid.corner_top_left").value, dtype=np.float64)
        corner_top_right = np.asarray(self.get_parameter("grid.corner_top_right").value, dtype=np.float64)
        corner_bottom_left = np.asarray(self.get_parameter("grid.corner_bottom_left").value, dtype=np.float64)

        if corner_top_left.size != 3 or corner_top_right.size != 3 or corner_bottom_left.size != 3:
            raise ValueError("grid corners must be [x,y,z]")

        # 参数语义说明：
        # - input_is_cell_centers=False：corner_* 直接表示“外框角点”
        # - input_is_cell_centers=True：corner_* 实际表示“左上/右上/左下格子的中心点”
        #   需要先反推外框角点，再统一进入角点驱动的网格生成流程。
        if input_is_cell_centers:
            self.get_logger().info(
                f"Converting cell centers to corners (rows={rows}, cols={cols})"
            )
            corner_top_left, corner_top_right, corner_bottom_left = convert_cell_centers_to_corners(
                center_top_left=corner_top_left,
                center_top_right=corner_top_right,
                center_bottom_left=corner_bottom_left,
                rows=rows,
                cols=cols,
            )
            self.get_logger().info(
                f"Converted corners: TL={corner_top_left}, TR={corner_top_right}, BL={corner_bottom_left}"
            )

        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left=corner_top_left,
            corner_top_right=corner_top_right,
            corner_bottom_left=corner_bottom_left,
            rows=rows,
            cols=cols,
            cell_width=cell_width,
            cell_height=cell_height,
            strict_size_check=strict_size_check,
            size_tolerance=size_tolerance,
        )

    def _load_grid_from_legacy_yaml(self, yaml_path: Path) -> GridFrame:
        if not yaml_path.exists():
            raise FileNotFoundError(f"legacy nine grid yaml not found: {yaml_path}")

        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if "grid_points" not in data:
            raise ValueError(f"legacy yaml missing 'grid_points': {yaml_path}")

        raw_grid_points = data["grid_points"]
        nine_points: Dict[int, np.ndarray] = {}
        for idx in range(1, 10):
            key_i = idx
            key_s = str(idx)
            if key_i in raw_grid_points:
                raw = raw_grid_points[key_i]
            elif key_s in raw_grid_points:
                raw = raw_grid_points[key_s]
            else:
                raise ValueError(f"legacy grid_points missing index {idx}")
            arr = np.asarray(raw, dtype=np.float64).reshape(-1)
            if arr.size != 3:
                raise ValueError(f"legacy grid_points[{idx}] must contain 3 values")
            nine_points[idx] = arr

        corner_top_left, corner_top_right, corner_bottom_left = convert_legacy_nine_grid_points(nine_points)
        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left=corner_top_left,
            corner_top_right=corner_top_right,
            corner_bottom_left=corner_bottom_left,
            rows=3,
            cols=3,
            cell_width=None,
            cell_height=None,
            strict_size_check=False,
            size_tolerance=1e-4,
        )

    def _stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _select_pose_for_image(self, image_stamp_sec: float) -> Tuple[np.ndarray, Optional[float]]:
        with self._lock:
            latest_pose = self._latest_t_w_s.copy()
            history = list(self._odom_history)

        if not history:
            return latest_pose, None

        use_closest = bool(self.get_parameter("odom_sync.use_closest_by_stamp").value)
        if not use_closest:
            odom_stamp_sec, pose = history[-1]
            return pose.copy(), abs(image_stamp_sec - odom_stamp_sec)

        odom_stamp_sec, pose = min(history, key=lambda item: abs(item[0] - image_stamp_sec))
        return pose.copy(), abs(image_stamp_sec - odom_stamp_sec)

    def _load_grid_from_yaml(self, yaml_path: Path) -> GridFrame:
        if not yaml_path.exists():
            raise FileNotFoundError(f"grid yaml not found: {yaml_path}")

        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if "grid" not in data:
            raise ValueError("grid yaml missing root key 'grid'")

        cfg = data["grid"]
        rows = int(cfg["rows"])
        cols = int(cfg["cols"])
        cell_width = float(cfg.get("cell_width", 0.0))
        cell_height = float(cfg.get("cell_height", 0.0))
        strict_size_check = bool(cfg.get("strict_size_check", False))
        size_tolerance = float(cfg.get("size_tolerance", 1e-4))
        corner_top_left = np.asarray(cfg["corner_top_left"], dtype=np.float64)
        corner_top_right = np.asarray(cfg["corner_top_right"], dtype=np.float64)
        corner_bottom_left = np.asarray(cfg["corner_bottom_left"], dtype=np.float64)

        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left=corner_top_left,
            corner_top_right=corner_top_right,
            corner_bottom_left=corner_bottom_left,
            rows=rows,
            cols=cols,
            cell_width=cell_width if cell_width > 0 else None,
            cell_height=cell_height if cell_height > 0 else None,
            strict_size_check=strict_size_check,
            size_tolerance=size_tolerance,
        )

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # 从里程计消息构造 T_w_s（sensor 在 world 中的位姿）：
        # P_w = T_w_s * P_s
        # 后续会求逆得到 T_s_w，用于把世界点变换到传感器系。
        rot_w_s = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        trans_w_s = np.array([p.x, p.y, p.z], dtype=np.float64)
        t_w_s = make_transform(rot_w_s, trans_w_s)

        with self._lock:
            self._latest_t_w_s = t_w_s
            self._latest_odom_time = msg.header.stamp
            self._odom_history.append((self._stamp_to_sec(msg.header.stamp), t_w_s.copy()))

        if not self._odom_received_once:
            self.get_logger().info("Received first odom message")
            self._odom_received_once = True

    def image_callback(self, msg: Image) -> None:
        with self._lock:
            latest_odom_time = self._latest_odom_time

        image_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        t_w_s, pose_age_sec = self._select_pose_for_image(image_stamp_sec)

        max_pose_age_sec = float(self.get_parameter("odom_sync.max_pose_age_sec").value)
        if pose_age_sec is not None and pose_age_sec > max_pose_age_sec:
            now = time.time()
            if now - self._last_pose_age_warn_time > 1.5:
                self.get_logger().warn(
                    f"Pose/Image timestamp gap is large: {pose_age_sec * 1000.0:.1f} ms. "
                    "Projection may jitter. Check sensor sync or odom/image rates."
                )
                self._last_pose_age_warn_time = now

        # ---------------- AR 变换主链路 ----------------
        # 已知：T_w_s（sensor 在 world 中）
        # 1) 求逆得到 T_s_w（world -> sensor）
        # 2) 与外参 T_c_s（sensor -> camera）相乘，得到 T_c_w（world -> camera）
        # 最终：P_c = T_c_w * P_w = T_c_s * T_s_w * P_w
        t_s_w = inverse_transform(t_w_s)
        t_c_w = self.t_c_s @ t_s_w

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = image.shape[:2]

        border_color = tuple(int(v) for v in self.get_parameter("draw.cell_border_color_bgr").value)
        border_thickness = int(self.get_parameter("draw.cell_border_thickness").value)
        center_color = tuple(int(v) for v in self.get_parameter("draw.cell_center_color_bgr").value)
        center_radius = int(self.get_parameter("draw.cell_center_radius").value)
        center_thickness = int(self.get_parameter("draw.cell_center_thickness").value)
        label_color = tuple(int(v) for v in self.get_parameter("draw.label_color_bgr").value)
        show_labels = bool(self.get_parameter("draw.show_labels").value)
        show_status_text = bool(self.get_parameter("draw.show_status_text").value)
        curve_samples_per_edge = int(self.get_parameter("draw.curve_samples_per_edge").value)
        use_curved_grid = self._is_fisheye_model_selected()

        cells_msg = GridCellArray()
        cells_msg.header = msg.header
        cells_msg.rows = self.grid_frame.rows
        cells_msg.cols = self.grid_frame.cols
        cells_msg.total_cells = self.grid_frame.total_cells
        cells_msg.visible_cells = 0
        cells_msg.cells = []
        cells_msg.visible_cell_ids = []

        visible_count = 0
        front_count = 0

        for cell_id in sorted(self.grid_frame.cells.keys()):
            cell = self.grid_frame.cells[cell_id]

            # 1) 格子中心点：world -> camera
            center_w_h = np.array([
                cell.center_world[0],
                cell.center_world[1],
                cell.center_world[2],
                1.0,
            ], dtype=np.float64)
            center_c_h = t_c_w @ center_w_h
            center_c = center_c_h[:3]

            corners_c: List[np.ndarray] = []
            corners_px: List[Tuple[float, float]] = []

            corners_front = True
            for cw in cell.corners_world:
                # 2) 格子四角点：world -> camera
                p_w_h = np.array([cw[0], cw[1], cw[2], 1.0], dtype=np.float64)
                p_c_h = t_c_w @ p_w_h
                p_c = p_c_h[:3]
                corners_c.append(p_c)

                # 3) 深度判定：z<=0 说明点在相机后方，无法成像
                if p_c[2] <= 1e-6:
                    corners_front = False
                    break

                # 4) 相机模型投影：camera 3D -> pixel 2D
                uv = self.camera_model.project_point(p_c)
                if uv is None:
                    corners_front = False
                    break
                corners_px.append(uv)

            if center_c[2] > 1e-6:
                front_count += 1

            center_uv = self.camera_model.project_point(center_c) if center_c[2] > 1e-6 else None
            center_in_image = (
                center_uv is not None and 0 <= center_uv[0] < w and 0 <= center_uv[1] < h
            )
            has_valid_corners = corners_front and len(corners_px) == 4
            any_corner_in_image = has_valid_corners and any(
                0 <= u < w and 0 <= v < h for (u, v) in corners_px
            )

            # 5) 可见性策略：
            # - 四角都需可投影（排除在相机后方或畸变模型非法点）
            # - 且中心或任一角点落在图像范围内
            visible = bool(has_valid_corners and (center_in_image or any_corner_in_image))
            if visible:
                visible_count += 1

            cell_msg = GridCell()
            cell_msg.header = msg.header
            cell_msg.cell_id = int(cell.cell_id)
            cell_msg.row = int(cell.row)
            cell_msg.col = int(cell.col)
            cell_msg.is_visible = bool(visible)

            cell_msg.position_world_frame = Point(
                x=float(cell.center_world[0]),
                y=float(cell.center_world[1]),
                z=float(cell.center_world[2]),
            )

            if visible:
                cell_msg.position_camera_frame = Point(
                    x=float(center_c[0]),
                    y=float(center_c[1]),
                    z=float(center_c[2]),
                )
                ordered = [(0, 0), (0, 0), (0, 0), (0, 0)]
                ordered[0] = (float(corners_px[0][0]), float(corners_px[0][1]))
                ordered[1] = (float(corners_px[1][0]), float(corners_px[1][1]))
                ordered[2] = (float(corners_px[2][0]), float(corners_px[2][1]))
                ordered[3] = (float(corners_px[3][0]), float(corners_px[3][1]))

                cell_msg.corners_pixel = [
                    Point32(x=float(ordered[0][0]), y=float(ordered[0][1]), z=0.0),
                    Point32(x=float(ordered[1][0]), y=float(ordered[1][1]), z=0.0),
                    Point32(x=float(ordered[2][0]), y=float(ordered[2][1]), z=0.0),
                    Point32(x=float(ordered[3][0]), y=float(ordered[3][1]), z=0.0),
                ]

                center_u = sum(p[0] for p in ordered) / 4.0
                center_v = sum(p[1] for p in ordered) / 4.0
                cell_msg.center_pixel = Point32(x=float(center_u), y=float(center_v), z=0.0)

                cv_pts = np.array([[int(round(u)), int(round(v))] for (u, v) in ordered], dtype=np.int32)
                if use_curved_grid:
                    world_corners = cell.corners_world
                    self._draw_sampled_world_edge(
                        image,
                        world_corners[0],
                        world_corners[1],
                        t_c_w,
                        border_color,
                        border_thickness,
                        curve_samples_per_edge,
                    )
                    self._draw_sampled_world_edge(
                        image,
                        world_corners[1],
                        world_corners[2],
                        t_c_w,
                        border_color,
                        border_thickness,
                        curve_samples_per_edge,
                    )
                    self._draw_sampled_world_edge(
                        image,
                        world_corners[2],
                        world_corners[3],
                        t_c_w,
                        border_color,
                        border_thickness,
                        curve_samples_per_edge,
                    )
                    self._draw_sampled_world_edge(
                        image,
                        world_corners[3],
                        world_corners[0],
                        t_c_w,
                        border_color,
                        border_thickness,
                        curve_samples_per_edge,
                    )
                else:
                    cv2.polylines(image, [cv_pts], isClosed=True, color=border_color, thickness=border_thickness, lineType=cv2.LINE_AA)

                c_u_i, c_v_i = int(round(center_u)), int(round(center_v))
                cv2.circle(image, (c_u_i, c_v_i), center_radius, center_color, center_thickness, lineType=cv2.LINE_AA)
                if show_labels:
                    cv2.putText(
                        image,
                        str(cell.cell_id),
                        (c_u_i + 5, c_v_i - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        label_color,
                        2,
                        lineType=cv2.LINE_AA,
                    )

                cells_msg.visible_cell_ids.append(int(cell.cell_id))
                cell_msg.depth = float(center_c[2])
            else:
                cell_msg.position_camera_frame = Point(x=0.0, y=0.0, z=0.0)
                cell_msg.corners_pixel = [
                    Point32(x=0.0, y=0.0, z=0.0),
                    Point32(x=0.0, y=0.0, z=0.0),
                    Point32(x=0.0, y=0.0, z=0.0),
                    Point32(x=0.0, y=0.0, z=0.0),
                ]
                cell_msg.center_pixel = Point32(x=0.0, y=0.0, z=0.0)
                cell_msg.depth = 0.0
            cells_msg.cells.append(cell_msg)

        cells_msg.visible_cells = int(visible_count)
        self.visible_cells_pub.publish(cells_msg)

        if show_status_text:
            has_odom = latest_odom_time is not None
            status = (
                f"odom={'ok' if has_odom else 'missing'} "
                f"front={front_count}/{self.grid_frame.total_cells} "
                f"visible={visible_count}/{self.grid_frame.total_cells} "
                f"dt_ms={(pose_age_sec * 1000.0):.1f}" if pose_age_sec is not None else
                f"odom={'ok' if has_odom else 'missing'} front={front_count}/{self.grid_frame.total_cells} visible={visible_count}/{self.grid_frame.total_cells} dt_ms=n/a"
            )
            cv2.putText(
                image,
                status,
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0) if has_odom else (0, 0, 255),
                2,
                lineType=cv2.LINE_AA,
            )

            if not has_odom:
                cv2.putText(
                    image,
                    "No odom data on configured topic",
                    (10, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                    lineType=cv2.LINE_AA,
                )

        now = time.time()
        if now - self._last_diag_log_time > 2.0 and visible_count == 0:
            self.get_logger().warn(
                "No visible cells projected. Check odom topic, extrinsic, camera model/intrinsics, and frame conventions."
            )
            self._last_diag_log_time = now

        out_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        out_msg.header = msg.header
        # self.image_pub.publish(out_msg)

        if bool(self.get_parameter("draw.show_window").value):
            window_name = str(self.get_parameter("draw.window_name").value)
            try:
                cv2.imshow(window_name, image)
                cv2.waitKey(1)
            except cv2.error as exc:
                if not self._window_error_logged:
                    self.get_logger().warn(
                        f"OpenCV window display failed (headless/GUI unavailable): {exc}. "
                        "Set draw.show_window:=false to suppress this warning."
                    )
                    self._window_error_logged = True


def main() -> None:
    rclpy.init()
    node = ArGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
