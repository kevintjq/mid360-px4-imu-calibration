# Mid360/PX4 Point-Cloud-Aided Online Calibration

English | [中文](README_CN.md)

ROS1 catkin package for online calibration of a Mid360 frame and a PX4 body IMU. The recommended path uses two IMU topics plus a point-cloud topic. Point-cloud scan motion is estimated with robust ICP, then both IMUs are calibrated against the same LiDAR motion reference and chained into the requested Mid360-to-body transform. The node subscribes only; it publishes no topic.

This design follows the staged structure used by LI-Init: stationary data for gravity/scale checks, motion sufficiency checks, temporal alignment, rotation/bias initialization, and translation/accelerometer refinement. LI-Init itself uses LiDAR motion states from LIO; this package provides a lightweight PointCloud2/Livox-custom-message ICP frontend for that role.

## Inputs

Defaults in `config/online_calibration.yaml`:

```yaml
mid_imu_topic: "/livox/imu"
body_imu_topic: "/mavros/imu/data_raw"
lidar_topic: "/livox/lidar"
```

The IMU topics must be `sensor_msgs/Imu`. The point-cloud subscriber accepts `sensor_msgs/PointCloud2` and Livox-style messages containing `points[].x/y/z`. Angular velocity must be `rad/s`. Acceleration is converted with the configured scale factors; Livox built-in IMU data commonly needs `mid_accel_scale: 9.80665`, while PX4 raw IMU is normally already in `m/s^2`. Verify the stationary norm with `rostopic echo` before running.

## Motion procedure

The terminal guides these phases:

1. static;
2. roll: slow forward/back rotation around X;
3. pitch: slow forward/back rotation around Y;
4. yaw: slow forward/back rotation around Z;
5. combined roll/pitch/yaw with changing angular speed;
6. static validation.

The default schedule is about 25 seconds, excluding operator time spent between prompts.

Keep the LiDAR and both IMUs rigidly fixed together. The point-cloud scene must be mostly static and contain geometric structure; blank walls, moving objects, or severe scan distortion can make ICP invalid. The node reports scan count, mean ICP error, residuals, observability, and split-half consistency. A result is not trustworthy merely because optimization converged.

## Build on Ubuntu 20.04 / ROS Noetic

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

## Run

Edit [config/online_calibration.yaml](config/online_calibration.yaml), then run:

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

Results are written to:

- `config/online_calibration_result.yaml`;
- `config/online_calibration_result.txt`.

The final YAML contains:

```yaml
mid360_relative_to_body_enu:
  translation_B_M_m: [x, y, z]
  rpy_B_M_deg: [roll, pitch, yaw]
```

`T_B_M` maps Mid360 frame `M` to PX4 body frame `B`, using ROS/MAVROS ENU-side semantics without an extra NED/FRD conversion.

## Quality checks

Inspect `accepted`, acceleration RMSE, translation eigenvalues/condition number, mean ICP error, and split-half disagreement. If the point cloud has fewer than `min_lidar_scans` valid scans or ICP cannot maintain enough correspondences, calibration stops instead of returning a misleading XYZ.

## Tests

```bash
python3 -m unittest discover -s tests -v
```

## License

MIT. See [LICENSE](LICENSE).
