# FAST_LIO_ZVISION-ROS1 适配说明

本文说明 `FAST_LIO_ZVISION-ROS1` 相较原始 `FAST_LIO-ROS1` 为适配 ZVISION 激光雷达所做的主要改动。

## 适配目标

ZVISION ROS 驱动发布的点云为 `sensor_msgs/PointCloud2`，点字段包含 `x/y/z/intensity/ring/timestamp`。原始 FAST_LIO 已支持 Livox、Velodyne、Ouster、MARSIM 等输入，但没有直接识别 ZVISION 点云字段，也没有把 ZVISION 每点绝对时间转换为 FAST_LIO 去畸变所需的点内相对时间。因此本版本选择在 FAST_LIO 源码内新增一种雷达类型 `ZVISION`，直接解析 ZVISION 点云。

## 核心源码改动

### 1. 新增 ZVISION 雷达类型

文件：`src/preprocess.h`

- 将 `LID_TYPE` 从 `AVIA/VELO16/OUST64/MARSIM` 扩展为 `AVIA/VELO16/OUST64/MARSIM/ZVISION`。
- 约定 `preprocess/lidar_type: 5` 表示 ZVISION。
- 新增 `zvision_handler()` 入口函数。

### 2. 注册 ZVISION 点类型

文件：`src/preprocess.h`

新增 `zvision_ros::Point`，字段为：

- `float x, y, z`
- `float intensity`
- `uint8_t ring`
- `double timestamp`

并通过 `POINT_CLOUD_REGISTER_POINT_STRUCT` 注册 PCL 点类型。该结构与 ZVISION 驱动发布的 `PointCloudXYZIRT` 字段对齐。

### 3. 新增 ZVISION 点云预处理函数

文件：`src/preprocess.cpp`

新增 `Preprocess::zvision_handler()`，处理逻辑如下：

- 清空 `pl_surf/pl_corn/pl_full`。
- 根据 `PointCloud2.fields` 按字段名查找 `x/y/z/intensity/ring/timestamp` 的 offset，避免依赖字段顺序。
- 手工从 `msg->data` 按 `point_step` 解析每个点。
- 使用 `timestamp` 与帧起始时间计算点内相对时间。
- 将相对时间写入 FAST_LIO 约定的 `PointType.curvature`，单位为毫秒。
- 过滤 `blind` 距离内的近点。
- 按 `point_filter_num` 做降采样，将点加入 `pl_surf`。

这里的关键转换是：

```text
ZVISION point timestamp -> scan-relative offset time -> PointType.curvature(ms)
```

FAST_LIO 后续 IMU 去畸变阶段会用 `curvature / 1000.0` 作为每个点相对当前扫描起点的时间。

### 4. 调整预处理分发逻辑

文件：`src/preprocess.cpp`

`Preprocess::process()` 中新增 `ZVISION` 分支，使 `lidar_type: 5` 时调用 `zvision_handler()`。

注意：当前版本中原 `MARSIM` 分支位置被替换为 `ZVISION`，因此如需继续使用 `MARSIM`，应补回独立的 `case MARSIM: sim_handler(msg); break;`。

### 5. 去除对外部 Livox ROS 驱动包的强依赖

文件：

- `CMakeLists.txt`
- `package.xml`
- `src/preprocess.h`
- `src/laserMapping.cpp`

改动包括：

- 移除 `livox_ros_driver` 的 catkin/package 依赖。
- 将 Livox `CustomMsg/CustomPoint` 头文件放到 `3rdparty/livox_ros_driver` 下。
- include 方式从系统包形式改为工程内头文件形式。
- `CMakeLists.txt` 增加 `${CMAKE_CURRENT_SOURCE_DIR}/3rdparty` include 路径。

这样工程不再必须先安装 Livox ROS1 驱动也能编译。

### 6. 增加 ZVISION 配置文件

新增文件：

- `config/zvision_nz1.yaml`
- `config/zvision_nz3.yaml`
- `config/zvision_nz5.yaml`
- `config/zvision_nz5_mt.yaml`

主要参数：

- `common/lid_topic`: ZVISION 点云 topic，例如 `/zvlidar_sdk/zvlidar_points_xyzirt`。
- `common/imu_topic`: ZVISION IMU topic，例如 `/zvlidar_sdk/zvlidar_imu_msg`。
- `preprocess/lidar_type: 5`
- `preprocess/scan_line`: 按型号设置为 96 或 192。
- `preprocess/scan_rate: 10`
- `preprocess/timestamp_unit: 2`
- `preprocess/blind: 0.3`
- `mapping/extrinsic_T` 与 `mapping/extrinsic_R`: 雷达与 IMU 外参。

### 7. 增加 ZVISION 启动文件

新增文件：

- `launch/mapping_zvision_nz1.launch`
- `launch/mapping_zvision_nz3.launch`
- `launch/mapping_zvision_nz5.launch`
- `launch/mapping_zvision_nz5_mt.launch`

每个 launch 文件加载对应的 ZVISION yaml，启动 `fastlio_mapping`，并可选启动 RViz。

### 8. 调整可视化与轨迹发布

文件：`src/laserMapping.cpp`

- `publish_path()` 中取消了原来每 10 帧发布一次 path 的限制，改为每帧都追加并发布路径。
- 对核心建图流程影响较小，主要改善 RViz 中轨迹显示的实时性。

## ZVISION 适配的数据链路

```text
ZVISION driver
  -> sensor_msgs/PointCloud2: x/y/z/intensity/ring/timestamp
  -> Preprocess::zvision_handler()
  -> PointType: x/y/z/intensity/curvature(ms)
  -> IMU_Processing 去畸变
  -> FAST_LIO 建图与里程计
```

## 与外置 adapter 方案的区别

工作空间中还存在 `zvision_fastlio_adapter_work`。它采用另一种方案：不改 FAST_LIO 源码，而是把 ZVISION 点云转换成类似 Velodyne 的字段布局，让 FAST_LIO 使用 `lidar_type: 2`。

本源码版本采用内嵌适配：

- FAST_LIO 中直接新增 `lidar_type: 5`。
- 直接解析 ZVISION 的 `timestamp` 字段。
- 不需要额外 adapter 节点。
- 后续维护需要同步 FAST_LIO 源码差异。

## 使用注意

- ZVISION 驱动需要发布 `PointCloudXYZIRT` 格式，至少包含 `x/y/z/timestamp`。
- `timestamp` 必须能够反映每个点在一帧扫描内的真实采样时间，否则 IMU 去畸变会退化。
- `scan_line`、`scan_rate`、`blind`、外参需要按实际雷达型号和安装方式重新确认。
- 如果使用非 NZ1 型号，不要只复制 launch 文件名，必须检查 yaml 中线数、FOV、外参和 topic。

