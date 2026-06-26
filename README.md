# rslidar_ws_rsairy_fastlio

本仓库整理了速腾聚创 RoboSense RSAIRY 激光雷达在 ROS2 Humble 下的驱动、FAST-LIO 适配、LiDAR-IMU 外参尝试和坐标系修正工程。

## 内容

- `src/rslidar_sdk`：速腾聚创 ROS2 驱动，已按 RSAIRY 使用场景调整为 `POINT_TYPE XYZIRT`，并开启 IMU 数据解析。
- `src/rslidar_msg`：速腾驱动消息包。
- `src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main`：基于 FAST-LIO ROS2/ZVISION 适配版本改造的 RSAIRY 适配算法。
- `src/lidar_align`：从 ROS1 移植到 ROS2 的 `lidar_align`，用于尝试 LiDAR-IMU 外参自动标定。

## 编译环境与依赖

推荐环境：

```text
Ubuntu 22.04
ROS2 Humble
```

依赖项：
```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  libeigen3-dev \
  libpcl-dev \
  libpcap-dev \
  libnlopt-dev \
  libnlopt-cxx-dev \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rosbag2-cpp
```

如果是第一次使用该工作空间，建议初始化并更新 rosdep：

```bash
sudo rosdep init
rosdep update
```

然后在工作空间根目录补齐 ROS 依赖：

```bash
cd ~/rslidar_ws
rosdep install --from-paths src --ignore-src -r -y
```

注意：`sudo rosdep init` 只需要执行一次。如果提示已经初始化，可以跳过。

## 快速编译

```bash
cd ~/rslidar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动

启动速腾驱动：

```bash
ros2 launch rslidar_sdk start.py
```

启动 FAST-LIO RSAIRY：

```bash
ros2 launch fast_lio mapping_rsairy.launch.py
```

当前整理版本默认使用“修改 IMU 输入坐标 + 更新 FAST-LIO 外参”的方案。静态 TF 视觉修正节点 `camera_init_visual_tf` 也保留在代码中，可按文档作为可选方案启用。

## 文档

- [整体适配说明](docs/01_overview.md)
- [速腾驱动配置](docs/02_rslidar_driver_config.md)
- [FAST-LIO RSAIRY 适配](docs/03_fast_lio_rsairy_adapter.md)
- [雷达与 IMU 坐标关系](docs/04_lidar_imu_frame.md)
- [自动标定尝试](docs/05_lidar_align_ros2.md)
- [视觉坐标修正方案](docs/06_visual_frame_fix.md)
- [测试数据与结果](docs/07_test_data.md)

## 测试 bag

测试 bag 的 `.db3` 文件约 442 MB，超过 GitHub 普通文件上传限制，因此没有纳入普通 git 提交。仓库中保留测试数据说明，建议后续通过 GitHub Release 附件或 Git LFS 管理。
