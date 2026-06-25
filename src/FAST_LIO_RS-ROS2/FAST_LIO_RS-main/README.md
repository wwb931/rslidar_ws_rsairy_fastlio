# FAST_LIO_RS-ROS2

这是基于 `FAST_LIO_ZVISION-ROS2` 改造的 ROS2 FAST-LIO 版本，用于适配速腾聚创 RoboSense RSAIRY 激光雷达。

完整适配修改记录见 [适配说明.md](适配说明.md)。

## 关键配置

默认配置文件：

```text
config/rsairy.yaml
```

默认话题：

```text
LiDAR: /rslidar_points
IMU:   /rslidar_imu_data
```

默认运行：

```bash
ros2 launch fast_lio mapping_rsairy.launch.py
```

## 速腾驱动要求

已按 `rslidar_ws/rslidar_ws/src/rslidar_sdk` 中的点云格式做适配。建议将速腾驱动 `CMakeLists.txt` 中的：

```cmake
set(POINT_TYPE XYZI)
```

改为：

```cmake
set(POINT_TYPE XYZIRT)
```

或：

```cmake
set(POINT_TYPE XYZIRTF)
```

这样 `/rslidar_points` 会输出 `ring(uint16)` 和 `timestamp(float64)`，FAST-LIO 可以使用真实逐点时间进行运动去畸变。

如果仍使用默认 `XYZI`，程序会按点序估算点时间，但建图精度会受运动畸变影响。

## 使用前需要确认

- RSAIRY 线数：默认 `scan_line: 96`，48/192 线型号请修改 `config/rsairy.yaml`。
- LiDAR-IMU 外参：默认是单位外参占位，实车使用必须替换为标定结果。
- IMU 话题：如果使用外置 IMU，请将 `imu_topic` 改为实际话题。
