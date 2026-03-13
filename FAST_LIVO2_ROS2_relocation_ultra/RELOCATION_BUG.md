# FAST-LIVO2 重定位模式崩溃问题

## 问题描述
启用 `locate_in_prior_map: true` 后，FAST-LIVO 进程会出现段错误崩溃（exit code -11）。

## 根本原因
重定位逻辑存在**死锁缺陷**：

### 代码问题位置
`src/LIVMapper.cpp`:
```cpp
void LIVMapper::img_cbk(...) {
  if (locate_in_prior_map && !initial_pose_received) {
    return;  // ❌ 拒绝所有图像
  }
}

void LIVMapper::imu_cbk(...) {
  if (locate_in_prior_map && !initial_pose_received) {
    return;  // ❌ 拒绝所有IMU数据
  }
}
```

### 死锁流程
1. FAST-LIVO 启动，等待初始位姿（`Waiting for initial pose...`）
2. 在收到位姿前，**拒绝处理所有传感器数据**
3. TEASER-GICP 需要 FAST-LIVO 提供当前点云才能计算位姿
4. FAST-LIVO 不处理数据 → 无法提供有效点云
5. 当 TEASER-GICP 勉强发送初始位姿时
6. FAST-LIVO 内部状态未初始化（从未处理过传感器数据）
7. 在 `BuildVoxelMap()` 访问未初始化结构 → **段错误崩溃**

## 表现症状
- 日志显示 `Waiting for initial pose...`
- 从不显示 `Get image, its header time: xxx`
- 收到初始位姿后立即崩溃：`process has died [exit code -11]`

## 临时解决方案
**关闭重定位模式**，使用纯 SLAM 模式：
```yaml
locate_in_prior_map: false
```

## 正确的修复方案
修改逻辑，让 FAST-LIVO 在等待初始位姿期间：
1. 仍然处理传感器数据（图像、IMU、激光雷达）
2. 只延迟发布里程计和地图更新
3. 收到初始位姿后，用先验地图初始化并继续正常运行

### 建议代码修改
```cpp
// 在 img_cbk, imu_cbk 等回调中
// ❌ 错误做法：
if (locate_in_prior_map && !initial_pose_received) return;

// ✅ 正确做法：仍然处理数据，只是不发布结果
// 正常处理传感器数据...
if (locate_in_prior_map && !initial_pose_received) {
  // 不发布 odom/map，但继续累积传感器数据
  return;  // 仅在发布前 return
}
```

## 日期
2026年3月6日

## 影响范围
- 所有使用 `locate_in_prior_map: true` 的配置
- 依赖重定位功能的应用场景
