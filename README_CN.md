# Mid360/PX4 IMU 外参标定

[English](README.md) | 中文

这是一个 ROS1 / Ubuntu 20.04 C++ 标定工程，用于计算 FAST-LIO 机体系（Mid360 内置 IMU）相对 PX4 飞控 IMU 的固定外参。程序读取 `/Odometry` 和 `/mavros/imu/data`，不发布任何话题，结果写入 YAML/TXT。

## 坐标系和目标

FAST-LIO 的 `header.frame_id`（通常是 `camera_init`）是启动时建立的局部世界系，不自动等于 ENU。`child_frame_id`（通常是 `body`）是 FAST-LIO 机体系，必须确认它对应 Mid360 内置 IMU。PX4 使用 `base_link` 和 ROS FLU 轴：X 前、Y 左、Z 上。

固定目标是 FAST-LIO body `M` 到 PX4 IMU `B` 的变换：

```text
T_B_M = [ R_B_M  t_B_M ]
        [   0      1   ]
```

`t_B_M` 表示 Mid360/FAST-LIO body 原点相对 PX4 IMU 原点的位置，表达在 PX4 FLU 坐标系。这是实际应使用的固定安装偏移。程序输出的 ENU 向量只是将 `t_B_M` 按 `enu_reference_stamp` 时刻的 MAVROS 姿态旋转后的结果，不是固定世界系外参。

## 标定思路

1. **静止初始化。** 程序采集一段静止数据，估计 PX4 陀螺仪零偏；同时使用 PX4 的 ENU 姿态和重力向量 `[0, 0, -9.80665]` m/s² 估计静止加速度偏置。此阶段必须保持整体静止。
2. **三轴激励。** 按 roll、pitch、yaw 顺序采集。每个轴都要有正反方向的明显转动，并达到目标角度。进度条同时要求 PX4 陀螺仪有运动、FAST-LIO odom 四元数也发生对应变化。请缓慢转动并有明显加速、减速；只匀速转动无法充分估计杆臂。
3. **四元数求角运动。** 程序不使用通常为 0 的 `Odometry.twist`。对相邻 FAST-LIO 姿态计算：`delta_R = R_i^T R_(i+1)`，再用 `omega_M = Log(delta_R) / dt` 得到 body 系角速度；对角速度差分得到角加速度。因此旋转运动直接来自 FAST-LIO pose 四元数。
4. **旋转外参。** 在时间重叠的采样点，将 PX4 角速度和 FAST-LIO 角速度进行 SVD 正交 Procrustes 匹配，求得 `R_B_M`。输出的 gyro RMSE 用于检查两个 IMU 的旋转一致性。
5. **平移杆臂。** 对 FAST-LIO pose 位置使用较宽时间窗口差分，得到 FAST-LIO body 系线加速度，并批量求解刚体关系：

   `a_B - R_B_M a_M = R_B_M (alpha_M x t_M_B + omega_M x (omega_M x t_M_B)) + bias`

   其中 `t_M_B` 是程序内部使用的反向平移约定，最后转换为本文定义的 `t_B_M` 输出。只有平移方程条件良好、角加速度充分时才接受结果。
6. **最终验证。** yaw 到 100% 后，程序先给出放置缓冲时间，再采集最终静止窗口。样本不足、静止验证失败、残差过大或平移方程秩退化时，程序会拒绝结果。

这是基于运动激励的批量估计，单次运行不能自动保证绝对精度。建议重复标定，并将固定 FLU 结果与尺量安装尺寸对照后再用于飞控或飞行软件。

## 话题和参数

```yaml
slam_odom_topic: "/Odometry"
body_imu_topic: "/mavros/imu/data"
```

odom 位姿必须是米制，并且与 IMU 话题有重叠时间。只有驱动单位不同时才修改 scale 参数。

## 编译运行

```bash
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs libeigen3-dev
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd .. && catkin_make
source devel/setup.bash
roslaunch mid360_px4_calibration online_calibration.launch
```

启动时保持静止，按提示完成 roll/pitch/yaw；三轴完成后利用放置缓冲时间放下设备，再保持静止完成最终验证。结果写入 `config/online_calibration_result.yaml` 和 `.txt`。

## 许可证

MIT，详见 [LICENSE](LICENSE)。
