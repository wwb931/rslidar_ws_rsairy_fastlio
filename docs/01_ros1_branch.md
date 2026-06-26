# ROS1 分支说明

本分支 `ros1` 用于保存 RSAIRY + FAST-LIO 的 ROS1 版本。ROS2 Humble 版本保留在 `main` 分支。

## 分支选择

本工程采用同一个 GitHub 仓库、不同分支管理：

- `main`：ROS2 Humble 适配版本。
- `ros1`：ROS1 Noetic 适配版本。

这样做的好处是两个版本共享同一个项目入口，但源码目录和编译方式互不干扰。

## 主要目录

- `src/rslidar_sdk`：速腾聚创 RSAIRY 驱动。
- `src/rslidar_msg`：速腾聚创 ROS1 消息包。
- `src/FAST_LIO_RS-ROS1/FAST_LIO_RS-main`：ROS1 版 FAST-LIO RSAIRY 适配。
- `src/lidar_align`：ROS1 版 LiDAR-IMU 自动标定尝试工具。

## 主要适配修改

### 1. rslidar_sdk

驱动保留 ROS1/ROS2 兼容结构，在 ROS1 环境下由 catkin 编译。

关键配置：

- `POINT_TYPE` 设置为 `XYZIRT`，保留逐点时间。
- `ENABLE_IMU_DATA_PARSE` 设置为 `ON`。
- `lidar_type` 设置为 `RSAIRY`。
- 点云话题为 `/rslidar_points`。
- IMU 话题为 `/rslidar_imu_data`。
- MSOP/DIFOP/IMU 端口分别为 `6699/7788/6688`。

### 2. FAST-LIO 预处理

在 `src/FAST_LIO_RS-ROS1/FAST_LIO_RS-main/src/preprocess.h` 中新增：

- `RS` 雷达类型，编号为 `6`。
- RoboSense 点云结构声明。
- `rs_handler()` 函数声明。

在 `src/FAST_LIO_RS-ROS1/FAST_LIO_RS-main/src/preprocess.cpp` 中新增：

- `lidar_type == RS` 的分发逻辑。
- `rs_handler()`，按字段名读取 `x/y/z/intensity/ring/timestamp`。
- 支持 `timestamp`、`time`、`t` 三种逐点时间字段名。
- 若没有逐点时间字段，则按扫描频率估算当前帧内相对时间。

### 3. FAST-LIO IMU 输入修正

在 `src/FAST_LIO_RS-ROS1/FAST_LIO_RS-main/src/laserMapping.cpp` 的 `imu_cbk()` 中，对 IMU 数据做：

```cpp
x -> -x
y ->  y
z -> -z
```

角速度和线加速度都做同样处理。该处理等价于绕 Y 轴旋转 180 度，仍保持右手坐标系。

### 4. FAST-LIO RSAIRY 配置

新增配置文件：

```text
src/FAST_LIO_RS-ROS1/FAST_LIO_RS-main/config/rsairy.yaml
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

mapping:
    extrinsic_R: [0.0, 1.0, 0.0,
                 -1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0]
```

### 5. FAST-LIO RSAIRY 启动文件

新增启动文件：

```text
src/FAST_LIO_RS-ROS1/FAST_LIO_RS-main/launch/mapping_rsairy.launch
```

启动命令：

```bash
roslaunch fast_lio mapping_rsairy.launch
```

## 编译

推荐环境：

```text
Ubuntu 20.04
ROS Noetic
```

编译：

```bash
cd ~/rslidar_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 启动

启动驱动：

```bash
roslaunch rslidar_sdk start.launch
```

启动 FAST-LIO：

```bash
roslaunch fast_lio mapping_rsairy.launch
```

## 自动标定说明

`src/lidar_align` 保留为 ROS1 版本，用于继续尝试 LiDAR-IMU 外参自动标定。

示例命令：

```bash
roslaunch lidar_align lidar_align.launch bag_file:=/path/to/move_slow.bag
```

注意：ROS1 只能直接读取 `.bag`，不能直接读取 ROS2 的 `.db3 + metadata.yaml` bag 目录。
