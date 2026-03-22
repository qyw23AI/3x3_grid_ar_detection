# 1 关于雷达点云保存使用[avia_relocation.yaml](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)时

img_en设置为1，savePCD()才会进行保存

# 2 关于使用[avia.yaml](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)不发布里程计信息

需要将img_en设置为0，才会发布

## **原因：img_en 作用（快速总结）**

* **声明 & 配置：** 默认在代码中为 1（见 [LIVMapper.h](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)），但不同配置文件可覆盖，例如 [avia_relocation.yaml**:10**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 和 [avia.yaml**:7**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)。
  * 代码声明： [LIVMapper.h**:140**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)
  * 参数读取/默认： [LIVMapper.cpp**:83**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)
* **决定 SLAM 模式（LIVO / ONLY_LIO / ONLY_LO）：**
  * 逻辑：`slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;`
  * 位置： [LIVMapper.cpp**:257**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)
  * 含义：img_en=1 且 lidar_en=1 → 启用 LiDAR+VIO（LIVO）；img_en=0 且 imu_en=true → 只用 LIO（ONLY_LIO）。
* **是否处理/订阅图像（直接影响启动与同步）：**
  * image 回调会在 [img_en==0](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 时直接返回（不处理）： [LIVMapper.cpp**:1117**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)
  * 在数据同步函数中若 img_en==1 则要求 img_buffer 非空，否则会阻塞/跳过处理： [LIVMapper.cpp**:1179**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)
  * 影响：img_en=1 时若图像流缺失或延迟，会导致 sync_packages 无法通过，进而停在等待图像（看起来像“卡住”或不发布里程计）。
* **PCD 保存 / 彩色点云：**
  * [savePCD()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 在 img_en==1 时会保存带 RGB 的 PCD（否则保存 intensity 版本）。若想输出彩色 PCD 要启用 img_en： [LIVMapper.cpp**:733**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)
  * 项目说明/issue 也提到：img_en=1 时才会触发 savePCD。
* **发布点云 / 可视化行为差异：**
  * 在某些发布路径上，代码针对 img_en 有分支（例如不启用图像时直接发布全分辨率点云）： [LIVMapper.cpp**:694**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)
* **实际建议（基于你遇到的问题）：**
  * 如果没有相机或图像不稳定：把配置文件中的 [common.img_en](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 设为 `0`（例如 [avia_relocation.yaml**:10**](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)），这样会跑 ONLY_LIO，避免因等待图像导致处理/里程计“卡住”或退出。
  * 如果需要视觉前端（VIO、彩色 PCD 等）：保持 [img_en=1](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)，并确保图像话题稳定、时间同步正确；否则要处理图像丢失或同步超时的逻辑（或修改代码以在图像缺失时降级为 LIO）。

# 3 start_all.sh：Ctrl-C 清理与重复启动导致第三次很卡

问题表现：多次用 `start_all.sh` 启动/关闭后，第 3 次左右再启动或关闭时系统明显变慢，出现残留的 ROS2 节点或后台进程，需要手动杀进程才能恢复。

原因分析：原脚本把每个组件在子 shell 中后台启动并记录子 shell 的 PID，但按 Ctrl-C 时只 `kill` 了这些父 shell 的 PID。由于 `ros2 launch` 与节点可能会衍生子进程或被 re-parent，单独 kill 父 shell 无法终止子孙进程，导致孤儿进程（占用端口、共享内存、文件句柄），长期累积会让系统“卡”。

已做的修复（已提交到仓库）：

- 在 `ar_calculate/start_all.sh` 中，后台启动用 `exec ${cmd}` 以减少父 shell 层级；
- 在 `cleanup()` 中，优先对记录的 PID 发送到对应的进程组（`kill -TERM -PID`，然后必要时 `kill -KILL -PID`），以确保同时终止子进程与孙进程。

为何有效：向进程组发送信号能一次性影响该组下所有进程，配合 `exec` 能让记录的 PID 更接近运行的目标进程，从而显著提高 Ctrl-C 后清理干净资源的成功率。

进一步建议（可选）：

- 使用 `setsid` 启动每个组件并把 PGID/PID 写到单独文件，便于定位与清理（例如：`setsid bash -lc "..." >log 2>&1 & echo $! > /tmp/${name}_${TS}.pid`）；
- 为关键组件（fast_livo、teaser_gicp、livox）写 PID 文件并在 `cleanup()` 中读取并终止它们；
- 如果仍出现残留，可用下列命令列出并手动清理：

```bash
ps -ef | grep -E 'ros2|launch|fast_livo|teaser_gicp|ar_grid_detector|livox' | grep -v grep
pgrep -a -f ros2
```

我可以继续帮你：

- 将 `start_all.sh` 增强为 `setsid` + 写 PID 文件（我可以提交补丁），或
- 在你机器上运行上面调试命令并协助清理残留进程。

# 4 关于雷达重定位和fast_livo同时使用，fast_livo退出

重定位和fast_livo要同时启动，但是会导致出问题，

[run()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 里在未收到初始位姿时直接 `return`，见 run 逻辑。且 [spin_some()](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 在它后面，导致 `/icp_result` 回调根本没机会执行，[initial_pose_received](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html) 不会变 true。结果：`fastlivo_mapping` 退出或不进入正常 LIO 流程，自然不会发布 `/aft_mapped_in_map`。

同时在回调里也把未收到初始位姿时的 LiDAR 数据直接丢弃了，见 livox 回调门控。

## 暂时的解决方案：额外启动一个fast_livo2节点

# 5 关于里程计发布频率远低于图像发布频率的解决

## 查看/aft_mapped_in_map 、/camera/camera/color/image_raw发布频率的命令

查看频率（Hz）

ros2 topic hz /aft_mapped_in_map

ros2 topic hz /camera/camera/color/image_raw

## 方法一：加大Mid360的发布频率，以此来提高里程计发布频率，来进行对齐

FAST_LIVO2_relocation_revise/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py 中修改发布频率的参数

publish_freq=10.0# freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.

注意这里调高频率会加大雷达的负担

调整到和图像差不多的频率就可以了，25HZ的频率差不多。

## 方法二：图像比里程计快，也就是大部分图像会在两帧里程计数据间，可以使用插值的方法
