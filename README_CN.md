# Mid360/PX4 离线双 IMU 外参标定

[English](README.md) | 中文

这是一个 ROS1 catkin 工程，只负责从 rosbag 读取 Mid360 IMU 和 PX4/MAVROS IMU 话题，离线计算外参参数，并把结果写成 `yaml` 和 `txt`。本工程不发布 MAVROS 话题。

## 输入

离线标定节点读取 ROS1 bag 里的两个 `sensor_msgs/Imu` 话题：

- Mid360 IMU，例如 `/livox/imu`
- PX4/MAVROS IMU，例如 `/mavros/imu/data`

角速度单位应为 `rad/s`，加速度单位应为 `m/s^2`。

## 输出

默认输出在本工程 `config/` 目录：

- `config/calibration_result.yaml`：结构化结果，适合后续程序读取；
- `config/calibration_result.txt`：人可读结果，适合现场查看和记录。

结果约定：

```text
T_M_B maps PX4 body B to Mid360 frame M
T_W_B = T_W_M * T_M_B
t_B = t_M + dt_B_wrt_M
```

其中 `translation_M_B_m` 是 PX4 body/IMU 原点在 Mid360 `M` 系中的坐标。

为了方便直接查看 Mid360 相对飞控 body 的偏移，结果里还会额外输出：

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

这部分是 `T_B_M`，表示 Mid360 相对 PX4 body 的 XYZ 和 RPY，按 ROS/MAVROS 侧 ENU 语义输出，不做 NED/FRD 转换。

## 编译

把工程放到 `catkin_ws/src` 下：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

依赖：

```bash
sudo apt install python3-numpy python3-scipy python3-yaml python3-rospkg ros-noetic-rosbag
```

## 配置

修改工程内的 `config/offline_calibration.yaml`：

```yaml
bag_path: "/path/to/calibration.bag"
mid_imu_topic: "/livox/imu"
body_imu_topic: "/mavros/imu/data"
stamp_source: "header"

output_yaml: "config/calibration_result.yaml"
output_txt: "config/calibration_result.txt"
```

相对路径会按本 ROS 包目录解析；绝对路径会原样使用。

`mid_frame` 和 `body_frame` 不需要手动填写，节点会从 rosbag 中两个 IMU 话题的 `header.frame_id` 读取并写入结果文件。如果某个话题的 `frame_id` 为空，程序才会使用内部默认名称兜底。

## 启动

```bash
roslaunch mid360_px4_calibration offline_calibration.launch
```

也可以指定另一份 YAML：

```bash
roslaunch mid360_px4_calibration offline_calibration.launch config:=/path/to/offline_calibration.yaml
```

## 当前计算内容

- 归一化互相关估计 `dt_B_wrt_M`
- Wahba/SVD 旋转初值
- 非线性细化旋转、时间偏移和陀螺组合偏置
- 鲁棒 IRLS 求解杆臂和平移加速度组合偏置
- 输出残差统计、可观性特征值和验收状态

## 测试

```bash
python3 -m unittest discover -s tests -v
```
