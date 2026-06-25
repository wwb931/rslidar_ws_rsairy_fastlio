# 整体适配说明

本次工作目标是将 FAST-LIO ROS2 适配到 RoboSense RSAIRY 激光雷达，并整理驱动、建图、外参、坐标系显示和自动标定尝试。

## 已完成内容

1. 速腾驱动侧：
   - 雷达型号配置为 `RSAIRY`。
   - 点云类型从默认 `XYZI` 改为 `XYZIRT`，提供 `ring` 和逐点 `timestamp`。
   - 开启 `ENABLE_IMU_DATA_PARSE`，配置 IMU UDP 端口 `6688`。
   - 点云话题为 `/rslidar_points`，IMU 话题为 `/rslidar_imu_data`。

2. FAST-LIO 侧：
   - 新增 RoboSense 点云预处理类型 `RS = 6`。
   - 新增 `rs_handler()`，按 `PointCloud2` 字段名解析 `x/y/z/intensity/ring/timestamp`。
   - 新增 `config/rsairy.yaml` 和 `launch/mapping_rsairy.launch.py`。
   - 支持 RSAIRY 96 线、10 Hz、`/rslidar_points`、`/rslidar_imu_data`。

3. 外参和坐标：
   - 根据 RSAIRY 雷达/IMU 坐标图整理了雷达到 IMU 的标准旋转关系。
   - 尝试过 `lidar_align` 自动标定，但短时间、单一姿态变化数据下结果不稳定。
   - 最终保留两种工程方案：发布静态 TF 修正 RViz 视觉，或修改 FAST-LIO 读取 IMU 数据时的坐标输入并同步修改 `extrinsic_R`。

4. 标定工具：
   - 将 `lidar_align` 从 ROS1/catkin 移植到 ROS2/ament。
   - 支持读取 ROS2 bag 中的 `PointCloud2` 和 `Imu`。
   - 支持速腾点云的 `timestamp/time/t` 字段作为逐点时间。

## 当前推荐方案

当前测试整理版本采用“修改 IMU 输入坐标”的方式：

```text
IMU 输入修正：x/z 取反，y 不变
R_imu_fix = RotY(180deg)
```

FAST-LIO 外参同步改为雷达到修正后 IMU/body 坐标系：

```yaml
extrinsic_R: [0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0,
              0.0, 0.0, 1.0]
```

这个方案使建图内部的 `body` 坐标更符合正常视觉方向，减少 RViz 里上下颠倒的问题。
