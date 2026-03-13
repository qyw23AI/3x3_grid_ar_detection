# 方法 B：SLAM 实时标定 - 快速参考卡片

## 🚀 快速流程（5 分钟）

### 步骤 1-2：启动系统 & 启动工具

**终端 1:**
```bash
cd ar_calculate
ENABLE_AR_OVERLAY=0 ./start_all_with_offset.sh
```

等待系统启动完成，看到 `✓ /aft_mapped_to_init` 消息。

**终端 2:**
```bash
cd ar_calculate
source install/local_setup.bash
python3 record_lidar_poses.py
```

工具启动完成后应显示 `等待位姿消息... ✓`

---

### 步骤 3-5：记录点位 & 生成配置

#### 快速路径记录（选择 1-3-7 模式）

在位姿工具的交互菜单中：

```
菜单输入: 2
点号: 1
当前位置: X=1.0123  Y=2.0456  Z=0.5234
确认? (y/n): y
✓ 已记录点 1
```

重复上述步骤记录点 3、点 7。

#### 查看已记录点位

```
菜单输入: 3
```

验证所有 3 个点都已正确记录。

#### 生成配置

```
菜单输入: 4
选择点位模式: 1
输入网格大小 (米): 0.2
输入输出文件名: 
✓ 配置文件已生成
```

#### 转换为 YAML

**终端 2（继续）:**
```bash
python3 calibrate_nine_grid.py nine_grid_calibration_XXXXXXXX.json
是否继续生成 YAML 配置? (y/n): y
✓ YAML 配置生成成功!
```

---

### 步骤 6-7：验证 & 启动完整系统

#### 查看生成的九宫格配置

```bash
cat nine_grid_points.yaml
```

应该显示 9 个点的世界坐标。

#### 启动完整 AR 系统

**终端 1（停止之前的进程后）:**
```bash
ENABLE_AR_OVERLAY=1 ./start_all_with_offset.sh
```

**终端 3（新建）:**
```bash
ros2 run rqt_image_view rqt_image_view /ar_overlay/image
```

✅ 完成！九宫格应该正确显示在相机画面上。

---

## 📋 交互菜单速查表

| 操作 | 菜单选项 | 说明 |
|------|--------|------|
| 查看当前位置 | 1 | 显示雷达当前位置坐标 |
| 记录点位 | 2 | 记录一个点的位姿 (input: 点号 1-9) |
| 查看已记录点位 | 3 | 列出所有已记录的点 |
| 生成配置文件 | 4 | 保存为 JSON 配置文件 |
| 退出程序 | 5 | 停止工具 |

---

## 🔍 常见问题排查

### 问题 1：位姿工具无法启动

**症状：** `No module named 'rclpy'` 或类似错误

**解决：**
```bash
source install/local_setup.bash
python3 record_lidar_poses.py
```

---

### 问题 2：无法接收位姿消息

**症状：** `等待位姿消息... ` 一直没有显示 ✓

**检查清单：**
1. SLAM 系统是否完全启动？
   ```bash
   ros2 topic list | grep aft_mapped
   ```
   应该看到 `/aft_mapped_to_init`

2. 检查 FAST-LIVO2 日志：
   ```bash
   tail -f ar_calculate/log/one_click/*livo*.log
   ```

3. 如果仍无法接收，检查 network：
   ```bash
   ros2 doctor --report
   ```

---

### 问题 3：记录的点位坐标变化很大

**原因：** SLAM 还在初始化或漂移

**解决：**
1. 等待更长时间让 SLAM 稳定（1-2 分钟）
2. 确保雷达有充分的特征（不要在空白墙面）
3. 稍微移动一下雷达让 SLAM 更新

---

### 问题 4：转换脚本报错 JSON 格式错误

**原因：** 点位未完整记录

**排查：**
```bash
# 打开生成的 JSON 文件查看
cat nine_grid_calibration_*.json | jq '.'

# 如果格式错误，重新用工具记录
python3 record_lidar_poses.py
```

---

### 问题 5：生成的九宫格位置不对

**可能的原因：**

| 现象 | 原因 | 解决方案 |
|------|------|--------|
| 九宫格完全在视野外 | 相机位置与雷达差距大 | 检查相机与雷达的安装位置 |
| 九宫格歪斜 | 标定点位放置不正确 | 重新执行标定，确保点放在正确位置 |
| 九宫格微小偏差 | 正常（精度±5cm） | 微调外参矩阵或网格大小 |

---

## 🎛️ 高级选项

### 覆盖网格大小

```bash
python3 calibrate_nine_grid.py nine_grid_calibration_*.json --grid-size 0.3
```

### 指定输出文件名

```bash
python3 calibrate_nine_grid.py nine_grid_calibration_*.json \
  --grid-size 0.25 \
  --output-yaml my_custom_grid.yaml
```

### 自动化脚本（批量标定）

```bash
#!/bin/bash
# 记录位姿并自动生成配置

cd /home/r1/Slam/ar_calculate

# 启动位姿工具并等待用户交互
python3 record_lidar_poses.py

# 自动转换最新的 JSON 文件
LATEST_JSON=$(ls -t nine_grid_calibration_*.json | head -1)
python3 calibrate_nine_grid.py "$LATEST_JSON"

echo "标定完成！"
```

---

## 📚 完整文档

详细说明请参考: `/home/r1/Slam/AR定位九宫格系统使用指南.md` 中的**第 4.1 节 - 方法 B**

---

## ✅ 标定完成清单

- [ ] SLAM 系统正常运行
- [ ] 位姿工具能接收 `/aft_mapped_to_init` 消息
- [ ] 已记录点 1、3、7（或 3、9、7）
- [ ] 生成了 JSON 配置文件
- [ ] 转换为 YAML 配置成功
- [ ] 查看 `nine_grid_points.yaml` 包含 9 个点
- [ ] 启动完整系统时九宫格正确显示

---

**需要帮助？**
- 查看日志：`cat ar_calculate/log/one_click/*.log`
- 查看完整文档：`cat AR定位九宫格系统使用指南.md`
- 测试单个话题：`ros2 topic echo /aft_mapped_to_init`
