"""
格子架生成器模块
Grid Frame Generator Module

根据三个角点（左上、右上、左下）和格子参数生成任意大小的格子架
Generate arbitrary grid frames from three corner points (top-left, top-right, bottom-left) and grid parameters
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class GridCell:
    """单个格子单元"""
    cell_id: int           # 格子编号 (行优先: row * cols + col)
    row: int               # 行索引 (从0开始)
    col: int               # 列索引 (从0开始)
    center_world: np.ndarray  # 格子中心世界坐标 [x, y, z]
    corners_world: List[np.ndarray]  # 四个角点世界坐标 [左上, 右上, 右下, 左下]
    
    def __post_init__(self):
        self.center_world = np.asarray(self.center_world, dtype=np.float64)
        self.corners_world = [np.asarray(c, dtype=np.float64) for c in self.corners_world]


@dataclass
class GridFrame:
    """格子架数据结构"""
    rows: int                    # 行数
    cols: int                    # 列数
    cell_width: float            # 单元格宽度 (米)
    cell_height: float           # 单元格高度 (米)
    cells: Dict[int, GridCell] = field(default_factory=dict)  # 所有格子 {cell_id: GridCell}
    
    # 定义格子架的三个角点 (世界坐标)
    corner_top_left: np.ndarray = field(default_factory=lambda: np.zeros(3))
    corner_top_right: np.ndarray = field(default_factory=lambda: np.zeros(3))
    corner_bottom_left: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    @property
    def total_cells(self) -> int:
        return self.rows * self.cols
    
    def get_cell(self, row: int, col: int) -> Optional[GridCell]:
        """根据行列获取格子"""
        cell_id = row * self.cols + col
        return self.cells.get(cell_id)
    
    def get_cell_by_id(self, cell_id: int) -> Optional[GridCell]:
        """根据ID获取格子"""
        return self.cells.get(cell_id)
    
    def get_all_corners_world(self) -> List[np.ndarray]:
        """获取所有格子角点的世界坐标列表（用于批量投影）"""
        corners = []
        for cell in self.cells.values():
            corners.extend(cell.corners_world)
        return corners
    
    def get_all_centers_world(self) -> List[Tuple[int, np.ndarray]]:
        """获取所有格子中心的世界坐标列表 [(cell_id, center), ...]"""
        return [(cell.cell_id, cell.center_world) for cell in self.cells.values()]


class GridFrameGenerator:
    """
    格子架生成器
    
    通过三个角点（左上、右上、左下）定义格子架平面，
    再按 rows/cols 将“外框”均匀切分生成所有格子。
    
    坐标系约定:
    - 左上角(top_left)为原点
    - 右方向为列增加方向 (X正向)
    - 下方向为行增加方向 (Y正向)
    """
    
    @staticmethod
    def generate_from_three_corners(
        corner_top_left: np.ndarray,
        corner_top_right: np.ndarray,
        corner_bottom_left: np.ndarray,
        rows: int,
        cols: int,
        cell_width: Optional[float] = None,
        cell_height: Optional[float] = None,
        strict_size_check: bool = False,
        size_tolerance: float = 1e-4,
    ) -> GridFrame:
        """
        从三个角点生成格子架
        
        Args:
            corner_top_left: 左上角世界坐标 [x, y, z]
            corner_top_right: 右上角世界坐标 [x, y, z]
            corner_bottom_left: 左下角世界坐标 [x, y, z]
            rows: 行数（垂直方向格子数）
            cols: 列数（水平方向格子数）
            cell_width: 单元格宽度（米），仅用于和角点推导结果做一致性校验
            cell_height: 单元格高度（米），仅用于和角点推导结果做一致性校验
            strict_size_check: 为 True 时，尺寸不一致将抛异常
            size_tolerance: 尺寸校验容差（米）
        
        Returns:
            GridFrame 对象
        """
        tl = np.asarray(corner_top_left, dtype=np.float64)
        tr = np.asarray(corner_top_right, dtype=np.float64)
        bl = np.asarray(corner_bottom_left, dtype=np.float64)
        
        # 计算外框两个主方向向量（注意：这里假设输入是“外角点”）
        vec_right = tr - tl  # 向右方向（沿列增加）
        vec_down = bl - tl   # 向下方向（沿行增加）
        
        # 计算整体尺寸
        total_width = np.linalg.norm(vec_right)
        total_height = np.linalg.norm(vec_down)
        
        # 单位方向向量
        unit_right = vec_right / total_width if total_width > 1e-10 else np.array([1, 0, 0])
        unit_down = vec_down / total_height if total_height > 1e-10 else np.array([0, 1, 0])
        
        # 由外框角点和行列数直接推导单格尺寸：
        # cell_width  = ||TR-TL|| / cols
        # cell_height = ||BL-TL|| / rows
        inferred_cell_width = total_width / cols
        inferred_cell_height = total_height / rows

        # 外部传入尺寸仅用于校验，不参与几何构建
        if cell_width is not None and abs(cell_width - inferred_cell_width) > size_tolerance and strict_size_check:
            raise ValueError(
                f"grid.cell_width ({cell_width}) != inferred width ({inferred_cell_width}) from three corners"
            )
        if cell_height is not None and abs(cell_height - inferred_cell_height) > size_tolerance and strict_size_check:
            raise ValueError(
                f"grid.cell_height ({cell_height}) != inferred height ({inferred_cell_height}) from three corners"
            )

        cell_width = inferred_cell_width
        cell_height = inferred_cell_height
        
        # 创建格子架对象
        grid_frame = GridFrame(
            rows=rows,
            cols=cols,
            cell_width=cell_width,
            cell_height=cell_height,
            corner_top_left=tl,
            corner_top_right=tr,
            corner_bottom_left=bl
        )
        
        # 生成所有格子
        for row in range(rows):
            for col in range(cols):
                cell_id = row * cols + col
                
                # 计算格子四个角点
                # 左上角
                corner_tl = tl + unit_right * (col * cell_width) + unit_down * (row * cell_height)
                # 右上角
                corner_tr = tl + unit_right * ((col + 1) * cell_width) + unit_down * (row * cell_height)
                # 右下角
                corner_br = tl + unit_right * ((col + 1) * cell_width) + unit_down * ((row + 1) * cell_height)
                # 左下角
                corner_bl = tl + unit_right * (col * cell_width) + unit_down * ((row + 1) * cell_height)
                
                # 计算中心点
                center = (corner_tl + corner_tr + corner_br + corner_bl) / 4.0
                
                cell = GridCell(
                    cell_id=cell_id,
                    row=row,
                    col=col,
                    center_world=center,
                    corners_world=[corner_tl, corner_tr, corner_br, corner_bl]
                )
                
                grid_frame.cells[cell_id] = cell
        
        return grid_frame
    
    @staticmethod
    def generate_nine_grid(
        corner_top_left: np.ndarray,
        corner_top_right: np.ndarray,
        corner_bottom_left: np.ndarray,
        cell_width: Optional[float] = None,
        cell_height: Optional[float] = None
    ) -> GridFrame:
        """
        生成标准九宫格（3x3）
        
        这是一个便捷方法，等同于调用 generate_from_three_corners 并设置 rows=3, cols=3
        """
        return GridFrameGenerator.generate_from_three_corners(
            corner_top_left, corner_top_right, corner_bottom_left,
            rows=3, cols=3,
            cell_width=cell_width, cell_height=cell_height
        )


def convert_cell_centers_to_corners(
    center_top_left: np.ndarray,
    center_top_right: np.ndarray,
    center_bottom_left: np.ndarray,
    rows: int,
    cols: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    将三个格子中心坐标转换为格子架的三个外角点坐标
    
    用于支持用户在"格子中心位置"采集坐标的场景。
    用户采集的是左上格子中心、右上格子中心、左下格子中心，
    本函数将其转换为格子架的外角点，以便后续生成完整格子架。
    
        对于 rows x cols 的格子架：
    - 从左上格子中心到右上格子中心，跨越 (cols-1) 个格子
    - 从左上格子中心到左下格子中心，跨越 (rows-1) 个格子

        几何意义：
        - 该函数用于把“中心点语义”转换为“外角点语义”，
            以便与 generate_from_three_corners 的角点驱动逻辑保持一致。
    
    Args:
        center_top_left: 左上格子的中心坐标 [x, y, z]
        center_top_right: 右上格子的中心坐标 [x, y, z]
        center_bottom_left: 左下格子的中心坐标 [x, y, z]
        rows: 格子架行数
        cols: 格子架列数
    
    Returns:
        (corner_top_left, corner_top_right, corner_bottom_left) 格子架外角点
    """
    ctl = np.asarray(center_top_left, dtype=np.float64)
    ctr = np.asarray(center_top_right, dtype=np.float64)
    cbl = np.asarray(center_bottom_left, dtype=np.float64)
    
    # 从左上中心到右上中心的向量，跨越 (cols-1) 个格子
    vec_right_total = ctr - ctl
    # 从左上中心到左下中心的向量，跨越 (rows-1) 个格子
    vec_down_total = cbl - ctl
    
    # 单个格子的宽度和高度向量
    if cols > 1:
        cell_width_vec = vec_right_total / (cols - 1)
    else:
        # 只有一列时，无法从水平方向推断宽度，使用默认或抛异常
        cell_width_vec = np.zeros(3)
    
    if rows > 1:
        cell_height_vec = vec_down_total / (rows - 1)
    else:
        # 只有一行时，无法从垂直方向推断高度，使用默认或抛异常
        cell_height_vec = np.zeros(3)
    
    # 格子架的外角点 = 对应格子中心 - 半个格子尺寸（向外偏移）
    # 左上外角 = 左上格子中心 - 0.5*宽度向量 - 0.5*高度向量
    corner_top_left = ctl - cell_width_vec / 2 - cell_height_vec / 2
    # 右上外角 = 右上格子中心 + 0.5*宽度向量 - 0.5*高度向量
    corner_top_right = ctr + cell_width_vec / 2 - cell_height_vec / 2
    # 左下外角 = 左下格子中心 - 0.5*宽度向量 + 0.5*高度向量
    corner_bottom_left = cbl - cell_width_vec / 2 + cell_height_vec / 2
    
    return corner_top_left, corner_top_right, corner_bottom_left


def convert_legacy_nine_grid_points(nine_points: Dict[int, np.ndarray]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    将旧版九宫格点格式（9个独立点）转换为三角点格式
    
    旧格式: {1: [x,y,z], 2: [x,y,z], ..., 9: [x,y,z]}
    编号规则:
        1  2  3
        4  5  6
        7  8  9
    
    Args:
        nine_points: 旧版九个点的字典
    
    Returns:
        (corner_top_left, corner_top_right, corner_bottom_left)
    """
    p1 = np.asarray(nine_points[1], dtype=np.float64)  # 左上格子中心
    p3 = np.asarray(nine_points[3], dtype=np.float64)  # 右上格子中心
    p7 = np.asarray(nine_points[7], dtype=np.float64)  # 左下格子中心
    
    # 对于 3x3 九宫格，使用通用转换函数
    return convert_cell_centers_to_corners(p1, p3, p7, rows=3, cols=3)
