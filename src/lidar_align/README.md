# lidar_align ROS2 版本说明

本包用于标定 3D 激光雷达与位姿传感器之间的外参。当前版本已从 ROS1/catkin 迁移到 ROS2/ament，可读取 ROS2 bag。

算法思路：给定一组 LiDAR 点云和一组位姿数据，优化 LiDAR 与位姿传感器之间的外参，使多帧点云融合后更加清晰、重合误差更小。

> 注意：该方法需要比较充分的非平面运动。只有平面移动或缓慢单一方向运动时，标定结果可能不稳定。

## ROS2 迁移内容

- `CMakeLists.txt` 从 `catkin` 改为 `ament_cmake`。
- `package.xml` 改为 ROS2 package format 3。
- 参数读取从 `ros::NodeHandle` 改为 `rclcpp::Node`。
- ROS1 `rosbag` 读取改为 `rosbag2_cpp::Reader`。
- 消息类型改为 ROS2 命名空间，例如 `sensor_msgs::msg::PointCloud2`。
- 新增 ROS2 启动文件 `launch/lidar_align.launch.py`。
- 支持读取速腾点云中的 `timestamp/time/t` 字段，并转换为每点相对时间 `time_offset_us`。

## 依赖安装

以 Humble 为例：

```bash
sudo apt install libnlopt-dev libnlopt-cxx-dev ros-humble-pcl-conversions ros-humble-rosbag2-cpp
```

如果使用其他 ROS2 版本，把 `humble` 替换为对应版本名。

## 编译

假设工作空间为 `~/lidar_align`：

```bash
cd ~/lidar_align
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_align --symlink-install
source install/setup.bash
```

包名仍然是：

```text
lidar_align
```

所以启动和编译都使用 `lidar_align`，外层文件夹名字不影响。

## 输入数据要求

ROS2 bag 至少需要包含：

```text
sensor_msgs/msg/PointCloud2
sensor_msgs/msg/Imu
```

如果使用 CSV 位姿文件，则 bag 只需要包含点云，位姿从 CSV 读取。

当前程序会自动读取 bag 中所有 `PointCloud2` 和 `Imu` 类型消息。如果 bag 中存在多个点云话题或多个 IMU 话题，建议先重新录制一个只包含目标话题的 bag，例如：

```bash
ros2 bag record /rslidar_points /rslidar_imu_data -o rs_align_bag
```

## 运行

使用 ROS2 bag：

```bash
ros2 launch lidar_align lidar_align.launch.py bag_file:=/home/wwb/bag/rs_test
```

指定输出文件：

```bash
ros2 launch lidar_align lidar_align.launch.py \
  bag_file:=/home/wwb/bag/rs_test \
  output_pointcloud_path:=/home/wwb/bag/rs_test/aligned.ply \
  output_calibration_path:=/home/wwb/bag/rs_test/calibration.txt
```

使用 CSV 位姿：

```bash
ros2 launch lidar_align lidar_align.launch.py \
  bag_file:=/home/wwb/bag/rs_test \
  transforms_from_csv:=true \
  csv_file:=/home/wwb/poses.csv
```

默认输出路径：

```text
/tmp/lidar_align_points.ply
/tmp/lidar_align_calibration.txt
```

## 常用参数

可以直接运行节点并通过 `--ros-args -p` 传参：

```bash
ros2 run lidar_align lidar_align_node --ros-args \
  -p input_bag_path:=/home/wwb/bag/rs_test \
  -p keep_points_ratio:=0.002 \
  -p use_n_scans:=100 \
  -p max_evals:=80
```

常用参数说明：

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| `input_bag_path` | ROS2 bag 目录 | 空 |
| `use_n_scans` | 使用前 N 帧点云 | 不限制 |
| `keep_points_ratio` | 随机保留点比例，越大越慢 | `0.01` |
| `min_point_distance` | 最小点距离 | `0.0` |
| `max_point_distance` | 最大点距离 | `100.0` |
| `local` | 是否只做局部优化 | `false` |
| `inital_guess` | 初始外参 `[x,y,z,rx,ry,rz,time]` | 全 0 |
| `angular_range` | 局部角度搜索范围，单位 rad | `0.5` |
| `translation_range` | 局部平移搜索范围，单位 m | `1.0` |
| `max_time_offset` | 最大时间偏移搜索范围，单位 s | `0.1` |
| `time_cal` | 是否估计时间偏移 | `true` |
| `max_evals` | 优化迭代次数上限 | `200` |

## 输出结果怎么看

程序结束后会打印：

```text
Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor Frame to the Lidar Frame
Active Transformation Matrix from the Pose Sensor Frame to the Lidar Frame
Active Translation Vector
Active Hamiltonen Quaternion
Time offset
```

这里的方向是：

```text
Pose Sensor Frame -> Lidar Frame
```

如果要填入 FAST-LIO，一般需要的是：

```text
LiDAR -> IMU
```

因此如果 Pose Sensor Frame 就是 IMU 坐标系，需要对输出结果取逆：

```text
R_lidar_imu = R_imu_lidar^T
T_lidar_imu = -R_imu_lidar^T * T_imu_lidar
```

然后填入 FAST-LIO：

```yaml
extrinsic_T: [tx, ty, tz]
extrinsic_R: [r00, r01, r02,
              r10, r11, r12,
              r20, r21, r22]
```

## 使用建议

- 如果优化出的 `Time offset` 正好等于 `max_time_offset` 的边界，例如 `-0.1` 或 `0.1`，说明结果可能不可信。
- 如果你已知安装方向大致为 90 度标准旋转，建议使用 `local:=true` 并给定接近真实值的 `inital_guess`。
- 点云和 IMU 时间不同步、运动激励不足、bag 中混入多个点云/IMU 话题，都会导致结果发散。
