# FAST_LIO_RS-ROS2 适配说明

本目录 `FAST_LIO_RS-ROS2/FAST_LIO_RS-main` 参考 `FAST_LIO_ZVISION-ROS2/FAST_LIO_ZVISION-main` 的方式，从 ZVISION ROS2 适配版本复制并改造，用于接入速腾聚创 RoboSense RSAIRY 激光雷达。

## 参考的速腾驱动信息

已查看 `rslidar_ws/rslidar_ws/src/rslidar_sdk` 中的速腾 ROS2 驱动源码和配置：

- `CMakeLists.txt` 中 `POINT_TYPE` 可选 `XYZI`、`XYZIRT`、`XYZIF`、`XYZIRTF`。
- `config/config.yaml` 中当前雷达类型为 `lidar_type: RSAIRY`，默认点云话题为 `/rslidar_points`，IMU 话题为 `/rslidar_imu_data`。
- `src/source/source_pointcloud_ros.hpp` 中 ROS2 `PointCloud2` 字段输出格式如下：
  - `XYZI`: `x(float32), y(float32), z(float32), intensity(float32)`
  - `XYZIRT`: 额外包含 `ring(uint16), timestamp(float64)`
  - `XYZIRTF`: 在 `XYZIRT` 基础上额外包含 `feature(uint8)`
- `src/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp` 中驱动内部点类型定义包含 `PointXYZI`、`PointXYZIRT`、`PointXYZIF`、`PointXYZIRTF`。
- `src/rs_driver/src/rs_driver/driver/decoder/decoder_RSAIRY.hpp` 中 RSAIRY 支持 48/96/192 线型号，驱动默认模型为 96 线，并根据 DIFOP/安装方式解析 firing time、ring 和 timestamp。

FAST-LIO 需要逐点相对时间做运动去畸变，因此建议速腾驱动编译为 `POINT_TYPE XYZIRT` 或 `XYZIRTF`。如果仍使用默认 `XYZI`，本适配会退化为按点序和 `scan_rate` 估算 offset time，精度会低于驱动提供逐点时间的情况。

## 新增/修改内容

### 1. 新建工程目录

新增目录：

```text
算法源码/FAST_LIO_RS-ROS2/FAST_LIO_RS-main
```

该目录以 `FAST_LIO_ZVISION-ROS2/FAST_LIO_ZVISION-main` 为基线复制，保留原有 ROS2 `fast_lio` 包结构、CMake、消息、RViz 和原有配置。

### 2. 新增 RoboSense 点云类型和 lidar_type

修改文件：

```text
src/preprocess.h
```

主要改动：

- 在 `enum LID_TYPE` 中新增：

```cpp
RS = 6
```

- 新增 `robosense_ros::Point` 点类型定义，对应速腾 `XYZIRT/XYZIRTF` 中 FAST-LIO 需要使用的字段：

```cpp
x, y, z, intensity, ring, timestamp
```

- 新增 `rs_handler()` 预处理函数声明。

### 3. 新增 RSAIRY 点云预处理逻辑

修改文件：

```text
src/preprocess.cpp
```

主要改动：

- 在 `Preprocess::process(sensor_msgs::msg::PointCloud2...)` 中新增 `case RS`，转入 `rs_handler()`。
- 新增 `rs_handler()`，按字段名解析 `PointCloud2`，不依赖字段顺序。
- 兼容速腾驱动 `XYZIRT/XYZIRTF` 点云：
  - `x/y/z`: `float32`
  - `intensity`: 优先按 `float32` 读取，同时兼容 `uint8`
  - `ring`: `uint16`
  - `timestamp`: `float64`，也兼容 `float32`
- 使用一帧内有效点的最小 `timestamp` 作为帧起始时间，计算每个点的相对时间：

```text
curvature = (point_timestamp - frame_start_timestamp) * 1000.0
```

FAST-LIO 原始逻辑使用 `curvature` 保存点相对帧起始的毫秒级 offset time，用于 IMU 去畸变。

- 当点云没有 `timestamp/time/t` 字段时，按点序和 `SCAN_RATE` 估算：

```text
curvature = point_index / point_count * (1000.0 / scan_rate)
```

### 4. 新增 RSAIRY 配置文件

新增文件：

```text
config/rsairy.yaml
```

关键参数：

```yaml
common:
    lid_topic:  "/rslidar_points"
    imu_topic:  "/rslidar_imu_data"

preprocess:
    lidar_type: 6
    scan_line: 96
    scan_rate: 10
    timestamp_unit: 0
    blind: 0.3
```

说明：

- `lidar_type: 6` 对应新增的 `RS`。
- `scan_line` 默认按 RSAIRY 96 线配置；如果使用 48/192 线型号，请改为对应线数。
- `timestamp_unit` 对 `rs_handler()` 中的 `timestamp(float64 秒)` 不再作为缩放依据，保留该参数是为了兼容 FAST-LIO 原参数结构。
- `extrinsic_T` 和 `extrinsic_R` 当前为单位外参占位，实际使用时应替换为 LiDAR 到 IMU 的标定外参。

### 5. 新增 ROS2 launch 文件

新增文件：

```text
launch/mapping_rsairy.launch.py
```

运行 FAST-LIO 时默认加载 `config/rsairy.yaml`：

```bash
ros2 launch fast_lio mapping_rsairy.launch.py
```

## 速腾驱动侧建议配置

为获得更好的去畸变效果，建议在速腾驱动中将：

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

然后重新编译 `rslidar_ws`。这样 `/rslidar_points` 会带 `ring` 和逐点 `timestamp` 字段，FAST-LIO_RS 可直接计算逐点 offset time。

## 使用流程

1. 编译并启动速腾驱动，确认 `/rslidar_points` 正常发布。
2. 如果使用 RSAIRY 内置 IMU，确认驱动开启 IMU 解析并发布 `/rslidar_imu_data`；如果使用外部 IMU，将 `rsairy.yaml` 中 `imu_topic` 改为实际 IMU 话题。
3. 将 `FAST_LIO_RS-main` 放入 ROS2 工作空间 `src` 下编译。
4. 启动：

```bash
ros2 launch fast_lio mapping_rsairy.launch.py
```

5. 根据实车安装修改 `config/rsairy.yaml` 中的外参、线数、盲区、滤波和协方差参数。

## 注意事项

- 当前适配侧重点是点云字段兼容和逐点时间接入，未修改 FAST-LIO 的核心建图/滤波逻辑。
- `feature_extract_enable` 默认保持 `false`，与 ZVISION 适配版本一致。
- 如果 `/rslidar_points` 仍为 `XYZI` 格式，程序可以运行，但没有真实逐点时间，运动畸变补偿效果会受影响。
- RSAIRY 48/96/192 线型号请同步修改 `scan_line`，并确认驱动已通过 DIFOP 获取正确雷达型号和安装模式。
