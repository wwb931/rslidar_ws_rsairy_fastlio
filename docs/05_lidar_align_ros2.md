# lidar_align ROS2 自动标定尝试

工具目录：

```text
src/lidar_align
```

## ROS2 移植内容

- `CMakeLists.txt` 从 `catkin` 改为 `ament_cmake`。
- `package.xml` 改为 ROS2 package format 3。
- 使用 `rclcpp` 替代 ROS1 NodeHandle。
- 使用 `rosbag2_cpp::Reader` 读取 ROS2 bag。
- 消息类型改为 ROS2 命名空间，例如 `sensor_msgs::msg::PointCloud2`。
- 新增 `launch/lidar_align.launch.py`。
- 读取速腾点云中的 `timestamp/time/t` 字段，转换为点级时间。

## 依赖

```bash
sudo apt install libnlopt-dev libnlopt-cxx-dev ros-humble-pcl-conversions ros-humble-rosbag2-cpp
```

## 编译

```bash
cd ~/rslidar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_align --symlink-install
source install/setup.bash
```

## 运行示例

```bash
ros2 launch lidar_align lidar_align.launch.py bag_file:=/home/wwb/bag/rs_test/move_slow
```

或直接传参：

```bash
ros2 run lidar_align lidar_align_node --ros-args \
  -p input_bag_path:=/home/wwb/bag/rs_test/move_slow \
  -p time_cal:=false \
  -p keep_points_ratio:=0.005 \
  -p use_n_scans:=100 \
  -p max_evals:=100.0
```

注意：当前 `max_evals` 参数类型是 `double`，所以传 `100.0`，不要传整数 `100`。

## 结果方向

工具输出方向为：

```text
Pose Sensor Frame -> Lidar Frame
```

如果 Pose Sensor Frame 是 IMU，FAST-LIO 需要的是：

```text
LiDAR -> IMU
```

因此要取逆：

```text
R_lidar_imu = R_imu_lidar^T
t_lidar_imu = -R_imu_lidar^T * t_imu_lidar
```

## 本次结论

本次短包自动标定结果不稳定，主要原因包括：

- 数据时间较短，约 20 秒。
- 姿态激励不足，roll/pitch/yaw 变化不充分。
- 运动方式比较单一。
- 可能存在时间同步误差。
- 场景约束不足，点云几何退化时优化不稳定。
- 优化出的 `Time offset` 曾贴近 `max_time_offset` 边界，说明时间偏移可能没有被充分约束。

最终工程没有采用自动标定结果，而是优先采用已知的 RSAIRY 雷达/IMU 坐标关系。
