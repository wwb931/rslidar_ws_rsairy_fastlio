# rslidar_ws_rsairy_fastlio - ROS1 branch

This branch is the ROS1 version of the RSAIRY FAST-LIO adaptation. The ROS2 Humble version remains on the `main` branch.

## Contents

- `src/rslidar_sdk`: RoboSense driver configured for RSAIRY, `POINT_TYPE XYZIRT`, and IMU parsing enabled.
- `src/rslidar_msg`: RoboSense ROS1 message package.
- `src/FAST_LIO_RS-ROS1/FAST_LIO_RS-main`: FAST-LIO ROS1 adaptation for RSAIRY.
- `src/lidar_align`: ROS1 `lidar_align` package kept for LiDAR-IMU extrinsic calibration experiments.

## Recommended Environment

```text
Ubuntu 20.04
ROS Noetic
```

Install common build dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-catkin-tools \
  python3-catkin-pkg \
  libeigen3-dev \
  libpcl-dev \
  libpcap-dev \
  libnlopt-dev \
  libnlopt-cxx-dev \
  libyaml-cpp-dev \
  ros-noetic-pcl-ros \
  ros-noetic-tf \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-rosbag \
  ros-noetic-rviz
```

## Build

Using `catkin_make`:

```bash
cd ~/rslidar_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Or using `catkin build`:

```bash
cd ~/rslidar_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

## Run

Start the RoboSense driver:

```bash
roslaunch rslidar_sdk start.launch
```

Start FAST-LIO for RSAIRY:

```bash
roslaunch fast_lio mapping_rsairy.launch
```

Run the ROS1 calibration experiment package:

```bash
roslaunch lidar_align lidar_align.launch bag_file:=/path/to/move_slow.bag
```

Note: ROS1 reads `.bag` files. ROS2 `rosbag2` folders such as `.db3 + metadata.yaml` cannot be read directly by ROS1 tools unless converted or replayed through a bridge.

## RSAIRY Adaptation Notes

FAST-LIO uses `preprocess/lidar_type: 6` for RoboSense RSAIRY in this branch.

The driver should publish:

```text
/rslidar_points
/rslidar_imu_data
```

The RSAIRY point cloud parser reads fields by name, so `XYZIRT` and `XYZIRTF` layouts are both acceptable as long as the point cloud includes `x`, `y`, `z`, `intensity`, `ring`, and a per-point time field named `timestamp`, `time`, or `t`.

For the tested engineering solution, FAST-LIO flips IMU input `x` and `z` in `laserMapping.cpp`, then uses this extrinsic rotation:

```yaml
extrinsic_R: [0.0, 1.0, 0.0,
             -1.0, 0.0, 0.0,
              0.0, 0.0, 1.0]
```

This keeps the visualization aligned with the lidar coordinate frame while preserving a right-handed IMU frame.

## Documentation

- [ROS1 branch adaptation notes](docs/01_ros1_branch.md)

## Branch Layout

- `main`: ROS2 Humble version.
- `ros1`: ROS1 Noetic version.
