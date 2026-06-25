# 视觉坐标修正方案

RSAIRY 内置 IMU 原始坐标可能导致 FAST-LIO 的 `body` 和 `camera_init` 在 RViz 中呈现上下相反。这里整理两种工程方法。

## 方法一：发布静态 TF，只修正 RViz 视觉

新增节点：

```text
src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main/src/camera_init_visual_tf.cpp
```

发布：

```text
camera_init_lidar -> camera_init
```

默认参数：

```text
roll  =  pi
pitch =  0
yaw   = -pi/2
```

对应旋转：

```text
[ 0 -1  0
 -1  0  0
  0  0 -1]
```

这样 RViz 的 Fixed Frame 选择：

```text
camera_init_lidar
```

优点：

- 不改变 FAST-LIO 内部建图和滤波。
- 风险小，适合只解决 RViz 视觉上下/方向问题。

缺点：

- 算法内部 `body` 仍然是原始 IMU 坐标习惯。

单独启动：

```bash
ros2 run fast_lio camera_init_visual_tf --ros-args \
  -p parent_frame:=camera_init_lidar \
  -p child_frame:=camera_init \
  -p roll:=3.141592653589793 \
  -p pitch:=0.0 \
  -p yaw:=-1.570796326794897
```

## 方法二：修改 IMU 输入坐标

当前测试版本采用该方法。

修改位置：

```text
src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main/src/laserMapping.cpp
```

函数：

```cpp
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
```

在复制 IMU 消息后，对加速度和角速度做同一个旋转：

```cpp
msg->linear_acceleration.x = -msg_in->linear_acceleration.x;
msg->linear_acceleration.y =  msg_in->linear_acceleration.y;
msg->linear_acceleration.z = -msg_in->linear_acceleration.z;

msg->angular_velocity.x = -msg_in->angular_velocity.x;
msg->angular_velocity.y =  msg_in->angular_velocity.y;
msg->angular_velocity.z = -msg_in->angular_velocity.z;
```

这是绕 Y 轴旋转 180 度，仍符合右手系。

同步修改：

```text
src/FAST_LIO_RS-ROS2/FAST_LIO_RS-main/config/rsairy.yaml
```

```yaml
extrinsic_R: [0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0,
              0.0, 0.0, 1.0]
```

优点：

- 建图内部 `body` 坐标更符合正常视觉直觉。
- 不需要 RViz 额外 Fixed Frame。

缺点：

- IMU 数据和 LiDAR 外参必须成对修改。
- 不应只改加速度或只改角速度。
