# 测试数据与结果

## 测试 bag

本地测试 bag 放置位置：

```text
src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main/bag/move_slow
```

包含：

```text
metadata.yaml
rosbag2_2026_06_23-17_52_32_0.db3
```

其中 `.db3` 文件约 442 MB，超过 GitHub 普通文件限制，因此本仓库 `.gitignore` 默认忽略 `*.db3`。建议后续使用 GitHub Release 附件上传，或使用 Git LFS 管理。

播放示例：

```bash
ros2 bag play src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main/bag/move_slow
```

## 图片

图片目录：

```text
src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main/photo
```

包含：

- `imu and lidar frame in the airy.png`：RSAIRY 雷达与内置 IMU 坐标关系图。
- `result_publish_static_transformation.png`：发布静态 TF 后的视觉结果。
- `result_modify_imu_iput_data.png`：修改 IMU 输入后的视觉结果。

## 标定结果

`lidar_align` 输出结果保存在：

```text
src/lidar_align/results
```

这些结果用于记录自动标定尝试。由于本次数据运动激励不足，结果主要用于分析，不作为最终 FAST-LIO 外参。
