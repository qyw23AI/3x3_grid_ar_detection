# 完善和检查使用aft_mapped_in_map的AR功能，现在问题是（大致完成）

1：雷达的稳定发布aft_mapped_in_map （勉强解决）

2：时间戳对齐

* 明确odem、image频率，时间戳

查看/aft_mapped_in_map 、/camera/camera/color/image_raw发布频率的命令

查看频率（Hz）

ros2topichz/aft_mapped_in_map

ros2topichz/camera/camera/color/image_raw

快速抓一条消息（检查 header.stamp 等）

ros2topicecho/aft_mapped_in_map-n1

ros2topicecho/camera/camera/color/image_raw-n1

查看话题详情（类型、QoS、发布者）

ros2topicinfo/aft_mapped_in_map

ros2topicinfo/camera/camera/color/image_raw

# 完善采点建九宫格

# 实现模拟电机给相机加旋转角(360)、或者平移的功能层,这算是一个和雷达之间角度方面的外参变换，控制电机转动相机一个角度，或者给相机一个移动，这毫无疑问会影响到相机，但是不会影响到雷达，因为二者分开固定在一辆车上，所以需要将这个外参变换直接作为一个可通过控制电机改变的，可控的固定外参，这样就能反映角度的同时，不影响AR使用订阅的aft_mapped_in_map，反映到AR叠加中。
