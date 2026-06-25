# 雷达与 IMU 坐标关系

参考图片：

```text
src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main/photo/imu and lidar frame in the airy.png
```

根据图中 RSAIRY 雷达坐标系和内置 IMU 坐标系，可整理出原始关系：

```text
X_imu = -Y_lidar
Y_imu = -X_lidar
Z_imu = -Z_lidar
```

因此原始雷达到 IMU 的旋转为：

```text
R_lidar_to_imu =
[ 0 -1  0
 -1  0  0
  0  0 -1]
```

FAST-LIO 中 `mapping.extrinsic_R` 的含义是：

```text
p_imu = R_lidar_to_imu * p_lidar + t_lidar_to_imu
```

也就是 LiDAR 到 IMU/body 的外参。

## FAST-LIO 坐标链

```text
LiDAR(raw) -> IMU/body -> camera_init/world
```

其中：

- `body` 在 FAST-LIO 中就是 IMU 坐标系。
- `camera_init` 是建图世界系，不是真实相机坐标。
- `/cloud_registered_body` 是去畸变后转到 `body` 的当前帧点云。
- `/cloud_registered` 是去畸变后再转到 `camera_init` 的当前帧点云。
- `/Odometry` 表示 `body` 在 `camera_init` 下的位姿。

完整变换：

```text
p_camera_init = R_world_body * (R_lidar_to_body * p_lidar + t_lidar_to_body) + t_world_body
```

## 修改 IMU 输入后的关系

当前测试工程在 `laserMapping.cpp::imu_cbk()` 中对 IMU 数据做了：

```text
x -> -x
y ->  y
z -> -z
```

这是绕 Y 轴 180 度：

```text
R_imu_fix =
[-1  0  0
  0  1  0
  0  0 -1]
```

修正后的雷达到 body/IMU 外参变为：

```text
R_lidar_to_body =
[ 0  1  0
 -1  0  0
  0  0  1]
```

对应 `rsairy.yaml`：

```yaml
extrinsic_R: [0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0,
              0.0, 0.0, 1.0]
```
