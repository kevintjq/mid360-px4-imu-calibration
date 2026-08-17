# Mid360/PX4 Offline Dual-IMU Extrinsic Calibration

English | [中文](README_CN.md)

This is a ROS1 catkin package that only reads Mid360 IMU and PX4/MAVROS IMU topics from a rosbag, computes offline calibration parameters, and writes `yaml` and `txt` result files. It does not publish MAVROS topics.

## Input

The offline calibration node reads two `sensor_msgs/Imu` topics from a ROS1 bag:

- Mid360 IMU, for example `/livox/imu`
- PX4/MAVROS IMU, for example `/mavros/imu/data`

Angular velocity should be in `rad/s`; acceleration should be in `m/s^2`.

## Output

Default outputs are written under this package's `config/` directory:

- `config/calibration_result.yaml`: structured result for later programmatic use;
- `config/calibration_result.txt`: human-readable result for inspection and logs.

Result convention:

```text
T_M_B maps PX4 body B to Mid360 frame M
T_W_B = T_W_M * T_M_B
t_B = t_M + dt_B_wrt_M
```

`translation_M_B_m` is the PX4 body/IMU origin expressed in the Mid360 `M` frame.

For direct inspection of the Mid360 offset relative to the flight-controller body, the result also includes:

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

This is `T_B_M`, the Mid360 frame relative to the PX4 body frame, expressed with ROS/MAVROS ENU-side semantics and no NED/FRD conversion.

## Build

Place this package under `catkin_ws/src`:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Dependencies:

```bash
sudo apt install python3-numpy python3-scipy python3-yaml python3-rospkg ros-noetic-rosbag
```

## Configure

Edit `config/offline_calibration.yaml`:

```yaml
bag_path: "/path/to/calibration.bag"
mid_imu_topic: "/livox/imu"
body_imu_topic: "/mavros/imu/data"
stamp_source: "header"

output_yaml: "config/calibration_result.yaml"
output_txt: "config/calibration_result.txt"
```

Relative paths are resolved under this ROS package; absolute paths are used as-is.

`mid_frame` and `body_frame` do not need to be configured manually. The node reads them from each IMU topic's `header.frame_id` in the rosbag and writes them to the result files. If a topic has an empty `frame_id`, the internal default name is used as a fallback.

## Launch

```bash
roslaunch mid360_px4_calibration offline_calibration.launch
```

You can also pass another YAML file:

```bash
roslaunch mid360_px4_calibration offline_calibration.launch config:=/path/to/offline_calibration.yaml
```

## What It Computes

- normalized correlation estimate of `dt_B_wrt_M`
- Wahba/SVD rotation initialization
- nonlinear refinement of rotation, time offset, and combined gyro bias
- robust IRLS solve for lever arm and combined accelerometer bias
- residual statistics, observability eigenvalues, and acceptance status

## Tests

```bash
python3 -m unittest discover -s tests -v
```
