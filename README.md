# Mid360/PX4 IMU Calibration

English | [中文](README_CN.md)

ROS1 / Ubuntu 20.04 C++ online extrinsic calibration for a Livox Mid360 and a PX4 body IMU. The node subscribes to two IMU topics and one LiDAR point-cloud topic, publishes nothing, and writes the Mid360 pose relative to the PX4 `base_link` body frame. MAVROS uses ROS ENU-side frame semantics; the body frame itself follows ROS FLU (forward-left-up) convention.

The workflow follows the staged idea of [HKU-MaRS LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init):

1. keep the rigid assembly still for initialization;
2. collect roll, pitch, and yaw excitation sequentially; each progress bar must reach 100% before the next axis starts;
3. estimate LiDAR scan motion with ICP;
4. search the IMU time offset, solve rotation, then solve the lever arm from acceleration;
5. validate residuals and print LI-Init-style progress bars and final results.

## Topics

Defaults in `config/online_calibration.yaml`:

```yaml
mid_imu_topic: "/livox/imu"
body_imu_topic: "/mavros/imu/data_raw"
lidar_topic: "/livox/lidar"
```

Both IMU topics must be `sensor_msgs/Imu`. The LiDAR subscriber accepts `sensor_msgs/PointCloud2` and Livox `CustomMsg` (`livox_ros_driver` / `livox_ros_driver2`). Angular velocity must be in rad/s. Check the stationary acceleration norm with `rostopic echo`; Livox built-in IMU commonly needs `mid_accel_scale: 9.80665`, while PX4 raw IMU is normally already in m/s².

## Build

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

## Run

Edit `config/online_calibration.yaml`, then run:

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

The terminal displays three excitation bars. After the static stage, it asks for roll (X), then pitch (Y), then yaw (Z). The next step is not entered until the current bar reaches 100%. The default collection is intentionally longer: 8 seconds static, 2.5 rad accumulated excitation per axis, at least 1500 IMU samples, and 30 valid LiDAR scans. With `motion_timeout_s: 0.0`, the node waits indefinitely if the device is not moved; it never proceeds with insufficient motion data. Use Ctrl+C to abort. A static, geometrically rich scene is required for ICP; avoid blank walls and moving objects.

Results are written under the package directory by default:

- `config/online_calibration_result.yaml`
- `config/online_calibration_result.txt`

The requested result is:

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

`T_B_M` maps a vector from the Mid360 sensor frame `M` into the PX4 `base_link` body frame `B`. The numerical axes are the message frame axes; the surrounding ROS world convention is ENU and the body convention is FLU. MAVROS already performs the FCU NED/FRD to ROS-side conversion, so this node does not apply another NED/FRD conversion.

## Data requirements and limitations

The node needs synchronized, overlapping IMU data, at least `min_samples` samples per IMU, and at least `min_lidar_scans` valid scans. Calibration is rejected when ICP or residual quality is poor. This is a compact C++ initialization frontend inspired by LI-Init, not a replacement for the full FAST-LIO2 map/refinement pipeline; accurate mounting and strong 3-axis excitation remain important.

## License

MIT. See [LICENSE](LICENSE).
