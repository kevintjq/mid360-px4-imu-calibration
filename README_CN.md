# Mid360 / PX4 IMU 外参标定

[English](README.md) | 中文

ROS1 / Ubuntu 20.04 C++ 工具。读取 FAST-LIO `/Odometry` 和 MAVROS `/mavros/imu/data`，通过引导式运动采集，批量估计固定旋转、杆臂和加速度组合偏置。不会向飞控发布数据。

## 编译运行

```bash
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs libeigen3-dev
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch mid360_px4_calibration online_calibration.launch
```

先启动 MAVROS 和 FAST-LIO，确认里程计持续跟踪、位姿单位为米。IMU 必须提供有效的 ENU 姿态、FLU 机体系角速度（rad/s）和含重力的加速度计比力（m/s²）。IMU 和里程计应有共同的时间基准；无效姿态、重复/乱序时间戳会被丢弃，坐标系变化会中止采集。

## 怎样操作

设备应保持刚性连接，并在 FAST-LIO 能稳定跟踪的环境中操作。终端显示当前步骤、进度、时间、正反向累计转角和样本量。交互式终端原地刷新；重定向日志时每两秒输出一次，不包含颜色控制码。设置 `NO_COLOR=1` 可关闭颜色。

| 步骤 | 操作 | 默认要求 |
| --- | --- | --- |
| 静止初始化 | 放在稳定平面上，完全静止 | 连续稳定窗口 8 秒，至少 100 帧 IMU |
| 每轴准备 | 回到舒适的中间姿态，短暂停稳 | 稳定 2 秒 |
| 横滚 Roll | 绕机体前后轴左右倾斜约 ±20–35°，再缓慢返回 | 至少 20 秒 |
| 俯仰 Pitch | 绕机体左右轴前后俯仰约 ±20–35°，再缓慢返回 | 至少 20 秒 |
| 偏航 Yaw | 尽量保持水平，左右转向约 ±30–45°，再缓慢返回 | 至少 20 秒 |
| 最终静止 | 轻放在稳定平面上，等振动停止后保持静止 | 连续稳定窗口 5 秒，至少 100 帧 IMU |

每次往复约 3–5 秒，自然加速、减速，通常每轴做 3–5 次；进度未满时继续平滑往复。不要猛甩、连续转圈或大幅平移。角度是操作建议，程序检查的是**两个方向各累计至少 1.5 rad（约 86°）**，不要求一次倾斜 86°。

每轴还必须满足：有效运动至少 8 秒、至少 300 帧 IMU 和 80 帧 odom，以及对应的里程计整体旋转量。时间、转角、样本量全部满足才进入下一步。安装旋转未知时，不要求两个传感器的同名轴同时变化。默认三轴运动至少 60 秒，完整流程通常需要约 1.5–3 分钟，未达标会继续等待；每轴最多等待 180 秒。

静止检查同时使用逐样本角速度门限、去偏角速度 **RMS**、角速度/加速度波动、数据连续性和两路姿态变化。IMU 或里程计相对静止参考姿态的变化超过默认 **1°** 时，会拒绝该候选窗口，防止缓慢匀速转动被当作陀螺仪零偏。正反转动也不会相互抵消。

检测到角速度超限、加速度突变、IMU/里程计缺帧或姿态变化时，程序会**清空候选窗口**。从后续第一帧合格 IMU 数据开始，必须重新采满完整的 8/2/5 秒，验收和进度条使用同一窗口。设备放下后的晃动会自动延长等待；断流、无有效数据或超时会给出明确原因。

## 坐标系与结果

输出定义为：

```text
p_B = R_B_M * p_M + t_B_M
```

`M` 是 FAST-LIO body，必须确认其原点和轴对应 Mid360 IMU；`B` 是 MAVROS 提供的 PX4 FLU 机体系。`t_B_M` 是 Mid360 原点相对 PX4 参考原点的位置，表达在 PX4 FLU 中。用于硬件安装偏移前，应确认数据所用参考原点与目标配置一致。

FAST-LIO 世界系（通常为 `camera_init`）不必是 ENU。程序只利用世界系位置求导后转回 body，不把该世界系直接当作 ENU。输出的 ENU 平移只是固定 FLU 杆臂在 `enu_reference_stamp` 时刻的旋转表达，不是固定世界系外参。

默认结果：

- `~/.ros/mid360_px4_calibration/result.yaml`
- `~/.ros/mid360_px4_calibration/result.txt`

父目录自动创建。支持 `~/`、绝对路径、相对包目录的路径。先写临时文件、检查写入成功，再逐个重命名；两个文件不构成跨文件原子事务，部分保存失败也会报错并返回非零。未通过标定验收时不会覆盖已有结果。

YAML/TXT 包含旋转矩阵、四元数 `[x,y,z,w]`、固定 FLU 平移、参考时刻的 ENU 平移、陀螺仪及加速度组合偏置、时间补偿约定、质量指标、阈值和采集时间段。新版旋转输出使用矩阵和四元数，替代含义容易混淆的 `rpy_mid360_to_body_XYZ_deg` 字段；保留 `translation_body_FLU_m` 作为平移别名。

## 求解与验收

1. 初始静止窗口直接记录采样起止时刻，估计陀螺仪偏置与加速度初始偏置，避免启动延迟把后续运动混进初始化。
2. 在三段运动窗口内，对位置和局部旋转对数做局部三次多项式拟合，求线加速度、角速度、角加速度。默认半窗口 0.3 秒，不使用经常为零的 `Odometry.twist`；不跨阶段边界或长缺帧区间求导，静止样本不用于稀释运动残差。
3. 只在 IMU 有效范围内插值，拒绝越界和过长间隔。利用角速度 SVD 配准估计旋转，检查三轴激励、角速度匹配 RMSE、相对误差和两路运动一致性。
4. 根据刚体加速度关系联合估计杆臂与常量加速度组合偏置。先消去偏置，再检查杆臂系统的条件数及最弱方向绝对激励，避免只检查原始前三列导致误接受。
5. 检查旋转误差、加速度残差、角加速度和物理尺寸上限；通过后才保存。具体失败项会在终端列出。

主要参数见 [config/online_calibration.yaml](config/online_calibration.yaml)：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `imu_time_offset_s` | 0 | 对齐后的 IMU 时间 = 原始 IMU 时间 + 此值 |
| `axis_min_duration_s` / `axis_min_active_s` | 20 / 8 | 每轴最短采集时间 / 最短有效运动时间（秒） |
| `axis_direction_target_rad` | 1.5 | 正、反每个方向的累计目标转角 |
| `static_max_rotation_deg` | 1 | 静止候选窗口内，两路姿态相对参考姿态的最大允许变化（度） |
| `max_gyro_rmse_rad_s` | 0.12 | 配对运动角速度向量 RMSE 上限 |
| `max_gyro_relative_rmse` | 0.25 | 角速度 RMSE / 两路信号对称 RMS 的上限 |
| `min_gyro_correlation` | 0.95 | 旋转对齐后两路角速度向量的归一化一致性下限 |
| `max_accel_rmse_m_s2` | 0.75 | 加速度向量 RMSE 上限 |
| `min_rotation_excitation_ratio` | 0.02 | 角速度激励矩阵最小/最大奇异值比下限 |
| `max_translation_condition` | 1000 | 消去偏置后的杆臂系统条件数上限 |
| `min_translation_excitation_rad_s2` | 0.03 | 最小奇异值除以样本数平方根的下限 |
| `max_lever_arm_m` | 2 | 安装杆臂物理长度上限，可按设备收紧 |

相对误差的分母为 `sqrt((IMU角速度RMS² + odom角速度RMS²) / 2)`；两路一致性为配对向量点积总和除以两路能量乘积的平方根，未减均值，不是 Pearson 相关系数。绝对误差、相对误差、一致性三项都必须通过，避免低幅度的不相关运动或单位比例错误被放行。终端和结果文件都保存这些指标及阈值。

**不自动估计时间偏移。** 先保证驱动时钟同步；仅在已知偏移时设置 `imu_time_offset_s`。角速度误差大时优先检查时钟、单位和刚性连接，不应直接放宽验收阈值。这些默认阈值是拒绝明显不良数据的起点，不代表某种精度保证。建议重复采集，将固定 FLU 杆臂与尺量尺寸对照。

## 测试

核心数值测试不依赖 ROS：

```bash
cd ~/catkin_ws/src/mid360-px4-imu-calibration
c++ -std=c++14 -O1 -Iinclude -I/usr/include/eigen3 \
  tests/calibration_math_test.cpp -o /tmp/calibration_math_test
/tmp/calibration_math_test
```

也可在 `catkin_make` 后进入 `~/catkin_ws/build`，运行 `ctest --output-on-failure -R calibration_math_test`。测试覆盖已知旋转/杆臂/偏置恢复、90° 安装方向、偏置耦合退化、往复运动静止误判、时间不重叠、插值缺帧和不均匀采样求导，还覆盖缓慢匀速转动、短暂运动后的完整重计时、返回原姿态、低幅度失配和比例错误，并包含 10/20/50/200 Hz 带噪声的三轴轨迹。合成测试不能替代真实 ROS/rosbag 和实机验证。

## 许可证

MIT，详见 [LICENSE](LICENSE)。
