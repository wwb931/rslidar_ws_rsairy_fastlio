# 速腾驱动配置

驱动目录：

```text
src/rslidar_sdk
```

## 点云类型

文件：

```text
src/rslidar_sdk/CMakeLists.txt
```

本次设置为：

```cmake
set(POINT_TYPE XYZIRT)
```

原因：FAST-LIO 需要逐点相对时间做运动去畸变。速腾默认 `XYZI` 没有逐点时间，只能按点序估算，运动时效果会变差。

`XYZIRT` 发布到 ROS2 `PointCloud2` 后包含：

```text
x(float32), y(float32), z(float32), intensity(float32), ring(uint16), timestamp(float64)
```

## IMU 解析

文件：

```text
src/rslidar_sdk/CMakeLists.txt
```

本次设置为：

```cmake
option(ENABLE_IMU_DATA_PARSE "Enable imu data parse" ON)
```

文件：

```text
src/rslidar_sdk/config/config.yaml
```

关键配置：

```yaml
common:
  msg_source: 1
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSAIRY
      msop_port: 6699
      difop_port: 7788
      imu_port: 6688
      use_lidar_clock: true

    ros:
      ros_send_point_cloud_topic: /rslidar_points
      ros_send_imu_data_topic: /rslidar_imu_data
```

## 启动

```bash
ros2 launch rslidar_sdk start.py
```

检查话题：

```bash
ros2 topic list
ros2 topic echo /rslidar_imu_data --once
ros2 topic echo /rslidar_points --once
```

## VMware 网络提示

雷达 UDP 数据不建议使用 NAT。虚拟机应使用桥接模式，桥接到连接雷达的物理网卡，并设置同网段静态 IP，例如：

```text
LiDAR: 192.168.1.200
VM:    192.168.1.102/24
```
