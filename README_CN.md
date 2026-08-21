# Mid360/PX4 点云辅助在线标定

[English](README.md) | 中文

这是一个 ROS1 catkin 工程，用于在线标定 Mid360 坐标系与 PX4 飞控 body IMU 的外参。推荐流程同时读取两个 IMU 话题和一个点云话题：先用稳健 ICP 估计点云运动，再让两个 IMU 分别相对于同一个 LiDAR 运动参考进行标定，最后链式计算 Mid360 到 body 的外参。节点只订阅，不发布任何话题。

流程参考 LI-Init 的分阶段思想：静止阶段检查重力和单位，运动阶段检查可观测性，再进行时间对齐、旋转/偏置初始化以及平移/加速度优化。LI-Init 本身使用 LIO 产生的 LiDAR 运动状态；本工程提供轻量的 PointCloud2/Livox 点云 ICP 前端。

## 输入

`config/online_calibration.yaml` 的默认配置为：

```yaml
mid_imu_topic: "/livox/imu"
body_imu_topic: "/mavros/imu/data_raw"
lidar_topic: "/livox/lidar"
```

两个 IMU 话题必须是 `sensor_msgs/Imu`。点云订阅器支持 `sensor_msgs/PointCloud2`，也支持包含 `points[].x/y/z` 的 Livox 风格消息。角速度必须是 `rad/s`。加速度会按 YAML 中的比例换算；Livox 内置 IMU 常见情况需要设置 `mid_accel_scale: 9.80665`，PX4 raw IMU 通常已经是 `m/s^2`。运行前请用 `rostopic echo` 检查静止时的加速度模长。

## 运动流程

终端会依次提示：

1. 静止；
2. roll：绕 X 轴缓慢正反转；
3. pitch：绕 Y 轴缓慢正反转；
4. yaw：绕 Z 轴缓慢正反转；
5. roll/pitch/yaw 组合运动，并改变角速度；
6. 最后静止验证。

默认采集时间约 25 秒，不包括操作者在提示之间按回车的时间。

Mid360、两个 IMU 和安装结构必须刚性固定。点云环境应静止且有足够几何结构；空白墙面、移动物体或严重扫描畸变都会导致 ICP 失效。节点会输出点云数量、平均 ICP 误差、残差、可观测性和前后半段一致性。优化收敛不代表结果一定可靠。

## Ubuntu 20.04 / ROS Noetic 编译

```bash
sudo apt update
sudo apt install python3-numpy python3-scipy python3-yaml python3-rospkg ros-noetic-roslib ros-noetic-sensor-msgs
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd ..
catkin_make
source devel/setup.bash
```

## 启动

编辑 [config/online_calibration.yaml](config/online_calibration.yaml)，然后运行：

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

结果写入：

- `config/online_calibration_result.yaml`；
- `config/online_calibration_result.txt`。

最终 YAML 包含：

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

`T_B_M` 表示 Mid360 坐标系 `M` 到 PX4 body 坐标系 `B` 的变换，采用 ROS/MAVROS 侧 ENU 语义，不额外执行 NED/FRD 转换。

## 质量检查

重点查看 `accepted`、加速度 RMSE、translation 特征值/条件数、平均 ICP 误差以及前后半段差异。如果有效点云扫描少于 `min_lidar_scans`，或 ICP 无法保持足够匹配点，程序会直接停止，不返回一个容易误用的 XYZ。

## 测试

```bash
python3 -m unittest discover -s tests -v
```

## 许可证

MIT，见 [LICENSE](LICENSE)。
