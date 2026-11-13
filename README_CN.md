<h1 align="center">Mid360 / PX4 IMU 外参标定</h1>

<p align="center">终端引导采集 · 三轴往复运动 · 旋转与杆臂联合求解</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS-Noetic-22314E?style=flat-square" alt="ROS Noetic">
  <img src="https://img.shields.io/badge/C%2B%2B-14-00599C?style=flat-square" alt="C++14">
  <img src="https://img.shields.io/badge/Ubuntu-20.04-E95420?style=flat-square" alt="Ubuntu 20.04">
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square" alt="MIT License">
</p>

<p align="center"><a href="README.md">English</a> · 简体中文</p>

<p align="center">
  <a href="#快速开始">快速开始</a> ·
  <a href="#输入要求">输入要求</a> ·
  <a href="#八步标定流程">标定流程</a> ·
  <a href="#结果与坐标系">结果与坐标系</a> ·
  <a href="#参数参考">参数参考</a> ·
  <a href="#常见问题">常见问题</a>
</p>

---

读取 FAST-LIO 里程计和 MAVROS IMU，跟随终端完成静止、拿起、三轴手持往复及放下，批量估计 **FAST-LIO body 到 PX4 FLU 参考机体系的旋转、杆臂和加速度组合偏置**。程序只读取数据，不向飞控发布控制消息。

| 能力 | 说明 |
| --- | --- |
| 引导采集 | 八个阶段自动切换，无需按键，显示当前动作和剩余条件 |
| 数据检查 | 静止窗口、双向运动、时间连续性及有效样本数量检查 |
| 联合求解 | 同窗口积分匹配，估计旋转、杆臂和加速度组合偏置 |
| 结果验收 | 质量门限全部通过后才保存；拒绝结果不覆盖旧文件 |

## 快速开始

推荐环境：**Ubuntu 20.04、ROS Noetic、catkin、Eigen3**。开始前将 Mid360 和飞控刚性安装，并先运行 FAST-LIO 与 MAVROS。

### 1. 安装依赖

**ROS 消息与节点库：**

```bash
sudo apt install \
  ros-noetic-roscpp \
  ros-noetic-roslib \
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs
```

**Eigen：**

```bash
sudo apt install libeigen3-dev
```

### 2. 获取代码

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
```

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 4. 启动

加载工作空间环境：

```bash
source ~/catkin_ws/devel/setup.bash
```

将设备放稳，在独立终端启动标定：

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

> 飞机只在初始化后拿起一次，三轴结束后再放下。**轴与轴之间保持手持，不需要反复放回桌面。** 默认完整流程约 1.5–3 分钟，三轴运动至少 60 秒。

## 输入要求

| 输入话题 | 类型 | 要求 |
| --- | --- | --- |
| `/Odometry` | `nav_msgs/Odometry` | 有效跟踪的 FAST-LIO body 位姿，位置单位为米 |
| `/mavros/imu/data` | `sensor_msgs/Imu` | 有效 ENU 参考姿态、FLU 角速度和未去重力的加速度计比力 |

- 角速度单位为 `rad/s`，加速度单位为 `m/s²`；保持两设备刚性连接。
- 两路时间戳应同步、递增；无效/重复/乱序帧会丢弃，坐标系变化、断流和超时会停止采集。
- 支持 **200 Hz 里程计**，不要求与 IMU 同频。高频不缩短最低采集时长，重复发布旧位姿不增加有效信息。
- FAST-LIO 世界系可以是局部初始化坐标系，不要求为 ENU；其 body 原点和轴必须与结果所需参考一致。

**程序不自动估计时间偏移。** 先同步时钟，只在偏移已知时设置 `imu_time_offset_s`：对齐后的 IMU 时间 = 原始时间 + 补偿值。

## 八步标定流程

按终端当前步骤操作，达到全部条件后自动前进。

| 阶段 | 操作 | 默认条件 |
| --- | --- | --- |
| 01 连接 | 运行数据源，设备先放稳 | 30 秒内收到有效数据 |
| 02 静止初始化 | 桌面或地面放稳，松手不动 | 连续静止 8 秒，至少 100 帧 IMU |
| 03 拿起 | 离开初始位置约 10 cm 以上，手持停稳 | 准备至少 5 秒；相对位移 ≥8 cm 持续 0.5 秒 |
| 04 横滚 Roll | 居中停稳，绕机体前后轴左右倾斜约 ±20–35° | 静止准备 2 秒，运动采集至少 20 秒 |
| 05 俯仰 Pitch | 保持手持，居中停稳后前后俯仰约 ±20–35° | 静止准备 2 秒，运动采集至少 20 秒 |
| 06 偏航 Yaw | 居中停稳，尽量水平地左右转向约 ±30–45° | 静止准备 2 秒，运动采集至少 20 秒 |
| 07 放下 | 轻放到稳定平面，松手等待晃动停止 | 放置缓冲 5 秒，再连续静止 5 秒 |
| 08 求解 | 等待计算与质量报告 | 所有质量项通过后保存结果 |

### 怎样完成一轴运动

每次往复约 **3–5 秒**，自然加速、减速，通常做 3–5 次；看终端剩余条件，未达标就继续平滑往复。避免猛甩、连续旋转和大幅平移。

| 条件 | 每轴默认目标 |
| --- | --- |
| 采集 / 有效运动时长 | 至少 20 / 8 秒 |
| 正向 / 反向累计转角 | 每个方向至少 1.5 rad，约 86° |
| IMU / 里程计样本 | 至少 300 / 80 帧 |
| 里程计总转动量 | 至少 1.8 rad，约 103° |

> 转角是多次往复的累加，**不需要一次倾斜 86°**。未知安装方向不要求两路数据的同名轴一起变化。

<details>
<summary>拿起判定、静止重计时与终端显示</summary>

- 拿起阶段检查实际相对位移，不只等待倒计时；最长等待 120 秒。不判断桌面接触或绝对高度。
- 拿起、放下和各轴准备数据不进入三轴求解窗口；每轴最长等待 180 秒。
- 静止检查包含逐样本角速度、RMS、加速度波动、数据连续性及两路姿态变化。姿态偏离静止参考超过默认 1°或出现运动/缺帧时，候选窗口清零，重新采满时长。
- 终端卡片显示步骤、动作、进度、实测值和目标；`✓` 表示达标，`○` 表示等待，`✗` 表示验收失败。底部列出尚需完成的条件。
- 小终端或重定向输出使用周期日志；`NO_COLOR=1` 可关闭颜色。建议单独使用一个终端，避免其他节点日志穿插。

</details>

## 结果与坐标系

默认输出两个文件：

| 文件 | 用途 |
| --- | --- |
| `~/.ros/mid360_px4_calibration/result.yaml` | 结构化结果，供后续配置使用 |
| `~/.ros/mid360_px4_calibration/result.txt` | 人工查看结果与质量指标 |

变换定义为：

```text
p_B = R_B_M × p_M + t_B_M
```

| 符号 | 定义 |
| --- | --- |
| `M` | FAST-LIO 所描述的 Mid360 IMU body，需确认实际原点和轴 |
| `B` | MAVROS 提供的 PX4 FLU 参考机体系 |
| `R_B_M` | 将 M 系向量旋转到 B 系 |
| `t_B_M` | M 系原点相对 B 系原点的位置，以 FLU 表达 |

使用结果时，核对消费该外参的配置方向与参考原点。固定杆臂应使用 FLU 表达；输出中的 ENU 平移只是参考时刻的表达，不是固定世界系外参。

<details>
<summary>结果字段、保存策略与退出码</summary>

结果包含旋转矩阵 `rotation_matrix_B_M`、四元数 `rotation_quaternion_xyzw`（顺序 `[x,y,z,w]`）、FLU 平移、参考时刻 ENU 平移、偏置、时间补偿约定、质量指标和采集区间。

`solver_model: matched_integral_v1` 标识求解模型；`translation_body_FLU_m` 保留为平移字段别名。

输出路径支持绝对路径、`~/` 和相对包目录，父目录自动创建。质量拒绝不覆盖旧结果；文件逐个检查并重命名，不构成跨文件原子事务。

| 退出码 | 含义 |
| --- | --- |
| `0` | 验收通过且保存成功 |
| `2` | 配置、采集或保存失败 |
| `3` | 求解失败或质量拒绝 |

</details>

## 参数参考

完整参数见 [online_calibration.yaml](config/online_calibration.yaml)。常用项如下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `slam_odom_topic` | `/Odometry` | FAST-LIO 位姿 |
| `body_imu_topic` | `/mavros/imu/data` | PX4/MAVROS IMU |
| `imu_time_offset_s` | `0.0` | 已知 IMU 时间偏移，秒 |
| `static_duration_s` | `8.0` | 初始连续静止，秒 |
| `neutral_duration_s` | `2.0` | 各轴开始前静止准备，秒 |
| `axis_min_duration_s` | `20.0` | 每轴最短采集，秒 |
| `axis_min_active_s` | `8.0` | 每轴最短有效运动，秒 |
| `axis_direction_target_rad` | `1.5` | 每个方向的累计转角 |
| `pickup_min_displacement_m` | `0.08` | 拿起确认位移，米 |
| `output_yaml` / `output_txt` | 见上方默认路径 | 输出文件 |

如需单独维护一份参数，可传入现有 launch 的 `config` 参数：

```bash
roslaunch mid360_px4_calibration online_calibration.launch \
  config:=/absolute/path/custom.yaml
```

<details>
<summary>高级质量门限与求解模型</summary>

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `static_max_rotation_deg` | `1.0` | 静止窗口姿态变化上限 |
| `derivative_half_window_s` | `0.3` | 匹配窗口半宽，秒 |
| `max_gyro_rmse_rad_s` | `0.12` | 角速度向量 RMSE 上限 |
| `max_gyro_relative_rmse` | `0.25` | 相对角速度误差上限 |
| `min_gyro_correlation` | `0.95` | 对齐向量一致性下限 |
| `max_accel_rmse_m_s2` | `0.75` | 加速度向量 RMSE 上限 |
| `min_rotation_excitation_ratio` | `0.02` | 三轴旋转激励比下限 |
| `max_translation_condition` | `1000.0` | 消去偏置后的杆臂系统条件数上限 |
| `min_translation_excitation_rad_s2` | `0.03` | 最弱方向激励下限 |
| `min_angular_accel_rms_rad_s2` | `0.1` | 角加速度 RMS 下限 |
| `max_lever_arm_m` | `2.0` | 杆臂物理长度上限 |

求解使用同窗口积分匹配：将里程计端点相对旋转与陀螺仪旋转积分对齐；杆臂方程对位置和旋转矩阵使用相同二阶算子，并以配套积分核处理重力补偿后的加速度。联合估计常量机体系加速度组合偏置，消去偏置后再检查可观测性。

积分不能跨阶段或长缺帧，FAST-LIO 世界系无需 ENU，不使用可能为零的 odom twist。相对角速度误差为 `RMSE / sqrt((IMU_RMS² + odom_RMS²) / 2)`；一致性是配对向量归一化点积，不是减均值后的 Pearson 系数。

质量项需要同时通过。先检查时间同步、单位、跟踪、刚性安装与运动激励，再考虑调整门限。验收通过不等于某种绝对精度，建议重复采集并与尺量安装尺寸对照。

</details>

## 常见问题

| 现象 | 处理建议 |
| --- | --- |
| 等待数据超时 | 检查话题是否更新、IMU 姿态是否有效、时间戳是否同步 |
| 拿起阶段不前进 | 实际离开初始位置，并确认 odom 位置正常变化 |
| 准备阶段反复重计时 | 手持居中停稳，检查振动与姿态漂移 |
| 运动进度停住 | 查看“尚需达标”，时间、双方向、样本和转动量都要满足 |
| 质量验收失败 | 按失败项排查，不要只放宽阈值 |
| 结果无法保存 | 检查输出目录权限和剩余空间 |

## 许可证

本项目采用 [MIT License](LICENSE)。
