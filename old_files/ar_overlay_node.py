from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
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

    return np.array(
        [
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22],
        ],
        dtype=np.float64,
    )


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ],
        dtype=np.float64,
    )
    ry = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ],
        dtype=np.float64,
    )
    rz = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return rz @ ry @ rx


class ArOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__("ar_overlay_node")

        self.declare_parameter("grid_yaml", "/home/r1/slam/src/ar_calculate/nine_grid_points.yaml")
        self.declare_parameter("camera_json", "/home/r1/slam/src/ar_calculate/d435.json")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/ar_overlay/image")
        self.declare_parameter("draw_radius", 6)
        self.declare_parameter("draw_thickness", -1)
        self.declare_parameter("point_color_bgr", [0, 0, 255])
        self.declare_parameter("show_labels", True)
        self.declare_parameter("draw_grid_lines", True)
        self.declare_parameter("line_color_bgr", [255, 0, 0])
        self.declare_parameter("line_thickness", 2)
        self.declare_parameter("show_status_text", True)
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "AR Overlay")
        self.declare_parameter("extrinsic_rpy_xyz", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("extrinsic_matrix_4x4", Parameter.Type.DOUBLE_ARRAY)

        grid_yaml = Path(str(self.get_parameter("grid_yaml").value))
        camera_json = Path(str(self.get_parameter("camera_json").value))

        self.grid_points_w = self._load_grid_points(grid_yaml)
        self.fx, self.fy, self.cx, self.cy = self._load_camera_intrinsics(camera_json)

        self.t_c_s = self._load_extrinsic_t_c_s()

        self._lock = threading.Lock()
        self._latest_t_w_s = np.eye(4, dtype=np.float64)
        self._latest_odom_time = None
        self._odom_received_once = False

        self.bridge = CvBridge()
        self._window_error_logged = False
        self._last_diag_log_time = 0.0

        odom_topic = str(self.get_parameter("odom_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)

        self.get_logger().info(f"Loaded 9 grid points from {grid_yaml}")
        self.get_logger().info(
            f"Camera intrinsics: fx={self.fx:.3f}, fy={self.fy:.3f}, cx={self.cx:.3f}, cy={self.cy:.3f}"
        )
        self.get_logger().info(f"Subscribed odom={odom_topic}, image={image_topic}")
        self.get_logger().info(f"Publishing overlay image to {output_image_topic}")
        self.get_logger().info(f"Using T_C_S:\n{self.t_c_s}")

    def _load_grid_points(self, yaml_path: Path) -> Dict[int, np.ndarray]:
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        if "grid_points" not in data:
            raise ValueError(f"Missing 'grid_points' in {yaml_path}")

        grid_points = data["grid_points"]
        output: Dict[int, np.ndarray] = {}
        for idx in range(1, 10):
            key_int = idx
            key_str = str(idx)
            if key_int in grid_points:
                raw = grid_points[key_int]
            elif key_str in grid_points:
                raw = grid_points[key_str]
            else:
                raise ValueError(f"grid_points missing index {idx}")

            arr = np.asarray(raw, dtype=np.float64).reshape(-1)
            if arr.size != 3:
                raise ValueError(f"grid_points[{idx}] must contain 3 values")
            output[idx] = arr

        return output

    def _load_camera_intrinsics(self, json_path: Path) -> Tuple[float, float, float, float]:
        data = json.loads(json_path.read_text(encoding="utf-8"))

        camera_matrix = data.get("camera_matrix", {})
        fx = camera_matrix.get("fx")
        fy = camera_matrix.get("fy")
        cx = camera_matrix.get("cx")
        cy = camera_matrix.get("cy")

        if None in (fx, fy, cx, cy):
            matrix_data = camera_matrix.get("data")
            if matrix_data is None:
                raise ValueError("camera_matrix must contain fx/fy/cx/cy or data")
            matrix = np.asarray(matrix_data, dtype=np.float64).reshape(3, 3)
            fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
            cx, cy = float(matrix[0, 2]), float(matrix[1, 2])

        return float(fx), float(fy), float(cx), float(cy)

    def _load_extrinsic_t_c_s(self) -> np.ndarray:
        try:
            matrix_raw = list(self.get_parameter("extrinsic_matrix_4x4").value)
        except ParameterUninitializedException:
            matrix_raw = []
        if len(matrix_raw) == 16:
            matrix = np.asarray(matrix_raw, dtype=np.float64).reshape(4, 4)
            return matrix
        if len(matrix_raw) not in (0,):
            raise ValueError("extrinsic_matrix_4x4 must have 16 numbers or be empty")

        rpy_xyz_raw = list(self.get_parameter("extrinsic_rpy_xyz").value)
        if len(rpy_xyz_raw) != 6:
            raise ValueError("extrinsic_rpy_xyz must have 6 numbers: [roll, pitch, yaw, x, y, z]")

        roll, pitch, yaw, x, y, z = [float(v) for v in rpy_xyz_raw]
        rot = rot_from_rpy(roll, pitch, yaw)
        trans = np.array([x, y, z], dtype=np.float64)
        return make_transform(rot, trans)

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        rot_w_s = quat_to_rot_matrix(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        trans_w_s = np.array([position.x, position.y, position.z], dtype=np.float64)
        t_w_s = make_transform(rot_w_s, trans_w_s)

        with self._lock:
            self._latest_t_w_s = t_w_s
            self._latest_odom_time = msg.header.stamp

        if not self._odom_received_once:
            self.get_logger().info("Received first odom message.")
            self._odom_received_once = True

    def image_callback(self, msg: Image) -> None:
        with self._lock:
            t_w_s = self._latest_t_w_s.copy()
            latest_odom_time = self._latest_odom_time

        t_s_w = np.linalg.inv(t_w_s)
        t_c_w = self.t_c_s @ t_s_w

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        draw_radius = int(self.get_parameter("draw_radius").value)
        draw_thickness = int(self.get_parameter("draw_thickness").value)
        point_color = tuple(int(v) for v in self.get_parameter("point_color_bgr").value)
        show_labels = bool(self.get_parameter("show_labels").value)
        draw_grid_lines = bool(self.get_parameter("draw_grid_lines").value)
        line_color = tuple(int(v) for v in self.get_parameter("line_color_bgr").value)
        line_thickness = int(self.get_parameter("line_thickness").value)
        show_status_text = bool(self.get_parameter("show_status_text").value)

        h, w = image.shape[:2]
        projected_pixels: Dict[int, Tuple[int, int]] = {}
        front_count = 0

        for idx in range(1, 10):
            p_w = self.grid_points_w[idx]
            p_w_h = np.array([p_w[0], p_w[1], p_w[2], 1.0], dtype=np.float64)
            p_c = t_c_w @ p_w_h

            x_c, y_c, z_c = p_c[0], p_c[1], p_c[2]
            if z_c <= 1e-6:
                continue
            front_count += 1

            u = self.fx * (x_c / z_c) + self.cx
            v = self.fy * (y_c / z_c) + self.cy

            u_i, v_i = int(round(u)), int(round(v))
            if not (0 <= u_i < w and 0 <= v_i < h):
                continue

            projected_pixels[idx] = (u_i, v_i)

            cv2.circle(image, (u_i, v_i), draw_radius, point_color, draw_thickness, lineType=cv2.LINE_AA)
            if show_labels:
                cv2.putText(
                    image,
                    str(idx),
                    (u_i + 6, v_i - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    lineType=cv2.LINE_AA,
                )

        if draw_grid_lines:
            edges = [
                (1, 2), (2, 3),
                (4, 5), (5, 6),
                (7, 8), (8, 9),
                (1, 4), (4, 7),
                (2, 5), (5, 8),
                (3, 6), (6, 9),
            ]
            for a, b in edges:
                if a in projected_pixels and b in projected_pixels:
                    cv2.line(
                        image,
                        projected_pixels[a],
                        projected_pixels[b],
                        line_color,
                        line_thickness,
                        lineType=cv2.LINE_AA,
                    )

        if show_status_text:
            has_odom = latest_odom_time is not None
            status = (
                f"odom={'ok' if has_odom else 'missing'} "
                f"front={front_count}/9 visible={len(projected_pixels)}/9"
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
        if now - self._last_diag_log_time > 2.0 and len(projected_pixels) == 0:
            self.get_logger().warn(
                "No projected grid points are visible in current frame. "
                "Check odom topic, extrinsic, and coordinate-frame conventions."
            )
            self._last_diag_log_time = now

        out_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

        if bool(self.get_parameter("show_window").value):
            window_name = str(self.get_parameter("window_name").value)
            try:
                cv2.imshow(window_name, image)
                cv2.waitKey(1)
            except cv2.error as exc:
                if not self._window_error_logged:
                    self.get_logger().warn(
                        f"OpenCV window display failed (headless/GUI unavailable): {exc}. "
                        "Set show_window:=false to suppress this warning."
                    )
                    self._window_error_logged = True


def main() -> None:
    rclpy.init()
    node = ArOverlayNode()
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
