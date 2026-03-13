"""
AR Grid Detector Package

支持针孔相机和鱼眼相机的灵活格子架检测与定位系统
Flexible grid frame detection and localization system supporting pinhole and fisheye cameras
"""

from .camera_models import (
    CameraModel,
    CameraModelType,
    CameraIntrinsics,
    PinholeCamera,
    FisheyeCamera,
    EquidistantFisheye,
    EquisolidFisheye,
    OrthographicFisheye,
    StereographicFisheye,
    KannalaBrandtFisheye,
    create_camera_model,
    create_camera_from_params,
)

from .grid_generator import (
    GridCell,
    GridFrame,
    GridFrameGenerator,
    convert_legacy_nine_grid_points,
)

from .utils import (
    quat_to_rot_matrix,
    rot_matrix_to_quat,
    make_transform,
    rot_from_rpy,
    rpy_from_rot,
    transform_point,
    inverse_transform,
    point_in_polygon,
    compute_polygon_area,
    clip_polygon_to_rect,
)

__all__ = [
    # Camera models
    "CameraModel",
    "CameraModelType",
    "CameraIntrinsics",
    "PinholeCamera",
    "FisheyeCamera",
    "EquidistantFisheye",
    "EquisolidFisheye",
    "OrthographicFisheye",
    "StereographicFisheye",
    "KannalaBrandtFisheye",
    "create_camera_model",
    "create_camera_from_params",
    # Grid generator
    "GridCell",
    "GridFrame",
    "GridFrameGenerator",
    "convert_legacy_nine_grid_points",
    # Utilities
    "quat_to_rot_matrix",
    "rot_matrix_to_quat",
    "make_transform",
    "rot_from_rpy",
    "rpy_from_rot",
    "transform_point",
    "inverse_transform",
    "point_in_polygon",
    "compute_polygon_area",
    "clip_polygon_to_rect",
]

__version__ = "1.0.0"
