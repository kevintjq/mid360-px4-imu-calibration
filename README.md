# Mid360/PX4 IMU Extrinsic Calibration

English | [中文](README_CN.md)

ROS1 / Ubuntu 20.04 C++ online calibration for a Livox Mid360 (or another SLAM sensor) rigidly mounted to a PX4 IMU. The node subscribes only to a generic `nav_msgs/Odometry` topic, `/mavros/imu/data_raw`, and `/mavros/imu/data`. It publishes nothing and writes YAML/TXT results.

The workflow is inspired by [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init): keep still, excite roll → pitch → yaw sequentially, estimate rotation from angular motion, estimate the lever arm from acceleration, and validate residuals. The odometry source is generic and is not tied to FAST-LIO.

The fixed result `t_B_M` is the SLAM/Mid360 sensor origin relative to the PX4 IMU origin, expressed in PX4 body FLU axes. The reported ENU vector is this offset rotated by the MAVROS attitude at `enu_reference_stamp`, so ENU components change with vehicle attitude.

## Configuration

```yaml
slam_odom_topic: "/Odometry"
body_imu_topic: "/mavros/imu/data_raw"
body_pose_topic: "/mavros/imu/data"
```

The odometry message must contain valid metric pose data and overlapping timestamps. Adjust scale parameters only if the driver units require it.

## Build and run (ROS Noetic)

```bash
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs libeigen3-dev
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd .. && catkin_make
source devel/setup.bash
roslaunch mid360_px4_calibration online_calibration.launch
```

Keep the assembly still during initialization and validation, then follow the terminal's roll/pitch/yaw instructions. `motion_timeout_s: 0.0` waits indefinitely for sufficient motion.

Outputs: `config/online_calibration_result.yaml` and `config/online_calibration_result.txt`.

## License

MIT. See [LICENSE](LICENSE).
