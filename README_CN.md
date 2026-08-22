# Mid360/PX4 IMU 外参标定

[English](README.md) | 中文

这是一个 ROS1 / Ubuntu 20.04 C++ 在线标定工程，用于计算 Mid360（或其他 SLAM 传感器）与 PX4 飞控 IMU 之间的刚性外参。程序只读取：

- 通用 SLAM 位姿话题（`nav_msgs/Odometry`）；
- `/mavros/imu/data_raw`（PX4 原始 IMU）；
- `/mavros/imu/data`（MAVROS ROS ENU 侧姿态）。

程序不发布任何话题，结果写入 YAML 和 TXT 文件。

## 标定思路

流程参考 [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)：先静止初始化，然后按 roll → pitch → yaw 顺序运动，每个进度条达到 100% 后进入下一轴；之后利用角速度一致性求旋转外参，利用位移二阶差分和 IMU 加速度求杆臂，最后检查残差。

这里的里程计输入是通用的，不限定 FAST-LIO。要求 `nav_msgs/Odometry` 中的位姿和位置是有效的、单位为米，并且与 IMU 时间有重叠。

固定输出 `t_B_M` 表示“SLAM/Mid360 传感器原点相对于 PX4 IMU 原点的位置”，表达在 PX4 body FLU（前、左、上）坐标系。程序同时输出一个 ENU 表达值；该值使用 `enu_reference_stamp` 时刻的 MAVROS 姿态旋转，因此飞行器姿态变化时 ENU 数值也会变化。

## 配置

编辑 `config/online_calibration.yaml`：

```yaml
slam_odom_topic: "/Odometry"
body_imu_topic: "/mavros/imu/data_raw"
body_pose_topic: "/mavros/imu/data"
```

根据驱动实际单位设置加速度和角速度 scale。默认假设加速度单位为 `m/s^2`、角速度单位为 `rad/s`。

## Ubuntu 20.04 / ROS Noetic 编译运行

```bash
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs libeigen3-dev
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd .. && catkin_make
source devel/setup.bash
roslaunch mid360_px4_calibration online_calibration.launch
```

启动后先保持整套刚性组件静止，再按照终端提示完成三轴运动。`motion_timeout_s: 0.0` 表示等待足够运动，不会因为超时自动使用不足的数据结束。

默认输出：

- `config/online_calibration_result.yaml`
- `config/online_calibration_result.txt`

## 许可证

MIT，详见 [LICENSE](LICENSE)。
