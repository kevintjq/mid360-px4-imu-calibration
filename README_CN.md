# Mid360/PX4 IMU 标定

[English](README.md) | 中文

这是一个面向 ROS1 / Ubuntu 20.04 的 C++ 在线外参标定工程，用于计算 Livox Mid360 与 PX4 飞控 body IMU 的相对位姿。节点只订阅两个 IMU 话题和一个点云话题，不发布任何话题，结果以 ENU 约定写入 YAML 和 TXT。

整体流程参考 [HKU-MaRS LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)：

1. 启动后先保持整套刚性组件静止；
2. 按 roll、pitch、yaw 顺序分三个阶段采集激励；当前进度条达到 100% 后才进入下一个轴；
3. 使用点云 ICP 估计 LiDAR 扫描间运动；
4. 搜索 IMU 时间偏移，依次求解旋转和平移杆臂；
5. 做残差和 ICP 质量检查，并用进度条显示各阶段状态。

## 话题

`config/online_calibration.yaml` 默认值：

```yaml
mid_imu_topic: "/livox/imu"
body_imu_topic: "/mavros/imu/data_raw"
lidar_topic: "/livox/lidar"
```

两个 IMU 必须是 `sensor_msgs/Imu`。点云支持 `sensor_msgs/PointCloud2`，以及 `livox_ros_driver` / `livox_ros_driver2` 的 `CustomMsg`。角速度单位必须是 rad/s。请用 `rostopic echo` 检查静止时加速度模长：Livox 内置 IMU 常见单位是 g，需要 `mid_accel_scale: 9.80665`；PX4 raw IMU 通常已经是 m/s²。

## Ubuntu 20.04 / ROS Noetic 编译

```bash
sudo apt update
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-topic-tools ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  libeigen3-dev
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd ..
catkin_make
source devel/setup.bash
```

## 启动

修改 `config/online_calibration.yaml` 后运行：

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

终端会显示三个激励进度条。静止阶段结束后，程序依次提示 roll（X）、pitch（Y）、yaw（Z）；当前轴的进度条没有达到 100% 时不会进入下一步。默认 `motion_timeout_s: 0.0` 表示无限等待，设备不动时程序不会带着不足数据结束；如需退出请按 Ctrl+C。点云环境需要静止且有足够几何结构，避免空白墙面和移动物体。

默认结果写入工程目录：

- `config/online_calibration_result.yaml`
- `config/online_calibration_result.txt`

目标输出格式：

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

`T_B_M` 表示把 Mid360 坐标系 `M` 中的向量变换到 PX4 body 坐标系 `B`。结果采用 ROS/MAVROS 的 ENU 侧语义，不再额外做 NED/FRD 转换。

## 数据要求与限制

需要两个时间有重叠的 IMU 数据流、每个 IMU 至少 `min_samples` 个样本，以及至少 `min_lidar_scans` 个有效点云扫描。当 ICP 或残差质量不满足阈值时，程序会拒绝输出“可信”结果。该工程是参考 LI-Init 思路实现的轻量 C++ 初始化前端，不是完整 FAST-LIO2 地图和在线精炼模块；安装刚性和充分的三轴激励仍然会直接影响精度。

## 许可证

MIT，详见 [LICENSE](LICENSE)。
