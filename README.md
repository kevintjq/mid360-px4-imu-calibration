# Mid360/PX4 Online Dual-IMU Extrinsic Calibration

English | [中文](README_CN.md)

This ROS1 catkin package estimates the rigid extrinsic calibration between a Livox Mid360 IMU and a PX4/MAVROS body IMU. The recommended workflow is **online calibration**: the node subscribes directly to two `sensor_msgs/Imu` topics, guides the operator through a sequence of motions, computes the result, and writes YAML/TXT files. It does not publish any ROS topic.

The original rosbag workflow is still available for repeatable offline experiments.

## What is estimated

- time offset `dt_B_wrt_M`;
- rotation and translation between the two IMU frames;
- combined gyro and accelerometer biases;
- residuals, observability eigenvalues, condition number, and acceptance status.

The result includes the requested ENU-side representation:

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

Here `T_B_M` maps Mid360 frame `M` to PX4 body frame `B`. The output uses the ROS/MAVROS-side ENU convention; it does not silently apply an extra NED/FRD conversion.

## Important accuracy limitation

Rotation is usually better constrained than the XYZ lever arm. The lever arm is only observable through angular acceleration and centripetal acceleration, so static data or one-axis constant-rate motion cannot determine it reliably. The online procedure therefore includes:

1. a static period for bias/noise characterization;
2. separate positive/negative rotations around X, Y, and Z;
3. a combined multi-axis motion with changing angular speed;
4. a final static period;
5. an optional split-half consistency check.

Do not accept XYZ only because the process finished. Check `accepted`, acceleration RMSE, translation condition number, and the split-half disagreement. If the result is unstable, repeat with slower, larger, multi-axis rotations and a rigid sensor mount.

## Topics and units

The node reads only:

- Mid360 IMU: default `/livox/imu`;
- PX4/MAVROS IMU: default `/mavros/imu/data`.

Both must be `sensor_msgs/Imu`. Angular velocity must be `rad/s` and linear acceleration must be `m/s^2`. The node uses `header.stamp` and automatically copies non-empty `header.frame_id` values into the result.

## Ubuntu 20.04 / ROS Noetic build

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

## Recommended online calibration

Edit [config/online_calibration.yaml](config/online_calibration.yaml):

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

Start it while both IMU topics are being published:

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

Follow the terminal prompts. Keep the complete rigid assembly in your hand or on the vehicle; do not translate it independently. During the X/Y/Z phases, rotate in both directions. During the combined phase, vary the angular speed and excite all three axes.

The node writes:

- `config/online_calibration_result.yaml`;
- `config/online_calibration_result.txt`.

Set absolute output paths if the package directory is not writable.

## Offline rosbag mode

Configure [config/offline_calibration.yaml](config/offline_calibration.yaml) with a bag path and the same two IMU topics, then run:

```bash
roslaunch mid360_px4_calibration offline_calibration.launch
```

The bag must contain both `sensor_msgs/Imu` topics with valid timestamps and overlapping time ranges. Odometry is not required and no output topic is published.

## Tests

The numerical tests do not require ROS:

```bash
python3 -m unittest discover -s tests -v
```

The project must be built and run on Ubuntu 20.04/ROS Noetic for ROS-level validation; the Windows development environment cannot run `catkin_make` or `roslaunch`.

## License

MIT. See [LICENSE](LICENSE).
