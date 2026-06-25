# FAST-LIO RSAIRY 适配

算法目录：

```text
src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main
```

## 新增配置

文件：

```text
config/rsairy.yaml
```

关键参数：

```yaml
common:
  lid_topic: "/rslidar_points"
  imu_topic: "/rslidar_imu_data"

preprocess:
  lidar_type: 6
  scan_line: 96
  scan_rate: 10
  timestamp_unit: 0
  blind: 0.3
```

`lidar_type: 6` 对应新增的 RoboSense 类型。

## 点云预处理修改

文件：

```text
src/preprocess.h
src/preprocess.cpp
```

主要修改：

- `enum LID_TYPE` 增加 `RS = 6`。
- 增加 RoboSense 点类型和 `rs_handler()`。
- `Preprocess::process(sensor_msgs::msg::PointCloud2...)` 中增加 `case RS`。
- `rs_handler()` 按字段名解析 `PointCloud2`，兼容 `XYZIRT/XYZIRTF`。
- 使用 `timestamp/time/t` 计算逐点相对时间，并写入 FAST-LIO 使用的 `curvature`。

逐点时间逻辑：

```text
curvature = (point_timestamp - frame_start_timestamp) * 1000.0
```

如果没有逐点时间字段，则退化为按点序和 `scan_rate` 估算。

## 当前外参

当前版本配合 IMU 输入坐标修正使用：

```yaml
extrinsic_R: [0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0,
              0.0, 0.0, 1.0]
```

如果不修改 IMU 输入，按原始 RSAIRY IMU 坐标，雷达到 IMU 的旋转应使用：

```yaml
extrinsic_R: [0.0, -1.0, 0.0,
              -1.0, 0.0, 0.0,
              0.0, 0.0, -1.0]
```

## 启动

```bash
ros2 launch fast_lio mapping_rsairy.launch.py
```
