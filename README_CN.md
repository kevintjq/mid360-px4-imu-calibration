# Mid360/PX4 在线双 IMU 外参标定

[English](README.md) | 中文

这是一个 ROS1 catkin 工程，用于计算 Livox Mid360 IMU 与 PX4/MAVROS 飞控 body IMU 之间的刚性外参。推荐使用**在线标定**：节点直接订阅两个 `sensor_msgs/Imu` 话题，在终端提示操作者按阶段运动，计算完成后写出 YAML/TXT 结果。节点不会发布任何 ROS 话题。

原有 rosbag 离线流程仍然保留，方便重复实验和复现。

## 标定内容

- 时间偏移 `dt_B_wrt_M`；
- 两个 IMU 坐标系之间的旋转和平移；
- 合并后的陀螺仪和加速度计偏置；
- 残差、可观测性特征值、条件数和是否通过质量检查。

结果包含你要求的 ENU 侧表示：

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

这里的 `T_B_M` 表示把 Mid360 坐标系 `M` 映射到 PX4 body 坐标系 `B`。输出采用 ROS/MAVROS 侧 ENU 语义，不会偷偷再做一次 NED/FRD 转换。

## 为什么在线流程能改善误差

XYZ 杆臂通常比旋转更难标定。杆臂只通过角加速度和向心加速度产生的差异可观测；静止数据或单轴匀速转动不能可靠确定 XYZ。因此在线流程固定包含：

1. 静止阶段，用于观察偏置和噪声；
2. 分别绕 X、Y、Z 轴正反转；
3. 三轴组合运动，并改变转动速度；
4. 最后静止；
5. 可选的前后半段一致性检查。

不要因为程序运行结束就直接接受 XYZ。必须查看 `accepted`、加速度 RMSE、translation condition number 以及前后半段差异。如果结果不稳定，请用更慢、更大幅度的三轴运动重复采集，并确保两个 IMU 刚性固定。

## 读取的话题和单位

节点只读取以下两个话题：

- Mid360 IMU：默认 `/livox/imu`；
- PX4/MAVROS IMU：默认 `/mavros/imu/data`。

两个话题都必须是 `sensor_msgs/Imu`。角速度单位必须是 `rad/s`，线加速度单位必须是 `m/s^2`。节点使用 `header.stamp`，并自动读取非空的 `header.frame_id` 写入结果。

## Ubuntu 20.04 / ROS Noetic 编译

```bash
sudo apt update
sudo apt install python3-numpy python3-scipy python3-yaml python3-rospkg ros-noetic-rosbag ros-noetic-sensor-msgs
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd ..
catkin_make
source devel/setup.bash
```

## 推荐：在线标定

编辑 [config/online_calibration.yaml](config/online_calibration.yaml)：

```yaml
mid_imu_topic: "/livox/imu"
body_imu_topic: "/mavros/imu/data"
output_yaml: "config/online_calibration_result.yaml"
output_txt: "config/online_calibration_result.txt"
interactive_prompts: true
phase_duration_s: 15.0
static_duration_s: 5.0
validate_split: true
```

在两个 IMU 话题都已经发布时启动：

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

按照终端提示操作。整个 Mid360、飞控和安装结构必须保持刚性，不要单独平移某个传感器。X/Y/Z 阶段都要正反转；组合阶段要改变角速度并尽量激励三轴。

节点默认写出：

- `config/online_calibration_result.yaml`；
- `config/online_calibration_result.txt`。

如果工程目录不可写，请把输出改成绝对路径。

## rosbag 离线模式

编辑 [config/offline_calibration.yaml](config/offline_calibration.yaml)，填写 bag 路径和同样的两个 IMU 话题，然后运行：

```bash
roslaunch mid360_px4_calibration offline_calibration.launch
```

bag 必须包含两个带有效时间戳且时间范围有重叠的 `sensor_msgs/Imu` 话题。不需要 odom，程序也不会发布输出话题。

## 测试

数值测试不依赖 ROS：

```bash
python3 -m unittest discover -s tests -v
```

ROS 级别必须在 Ubuntu 20.04/ROS Noetic 上用 `catkin_make` 和 `roslaunch` 验证；Windows 开发环境不能替代这一步。

## 许可证

MIT，见 [LICENSE](LICENSE)。
