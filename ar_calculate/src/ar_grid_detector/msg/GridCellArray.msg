# 格子数组消息
# Grid cell array message

std_msgs/Header header

# 格子架的行数和列数
# Number of rows and columns in the grid frame
int32 rows
int32 cols

# 总格子数
# Total number of cells
int32 total_cells

# 可见格子数
# Number of visible cells
int32 visible_cells

# 所有格子信息（包括不可见的）
# All cell information (including invisible ones)
ar_grid_detector/GridCell[] cells

# 可见格子ID列表（方便快速索引）
# List of visible cell IDs (for quick indexing)
int32[] visible_cell_ids
