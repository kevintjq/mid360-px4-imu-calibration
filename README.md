# Mid360/PX4 IMU Extrinsic Calibration

English | [中文](README_CN.md)

ROS1 / Ubuntu 20.04 C++ calibration for the FAST-LIO body (Mid360 built-in IMU) relative to a PX4 IMU. The node reads `/Odometry` and `/mavros/imu/data`; it publishes no topics and writes YAML/TXT results.

## Coordinate frames and target

FAST-LIO `header.frame_id` (normally `camera_init`) is a local world frame created at startup. It is not automatically ENU. `child_frame_id` (normally `body`) is the FAST-LIO body frame and must correspond to the Mid360 IMU frame. PX4 data uses `base_link` with ROS FLU axes: X forward, Y left, Z up.

The fixed target is the transform from FAST-LIO body `M` to PX4 IMU `B`:

```text
T_B_M = [ R_B_M  t_B_M ]
        [   0      1   ]
```

`t_B_M` is the Mid360/FAST-LIO body origin relative to the PX4 IMU origin, expressed in PX4 FLU. This is the fixed installation offset to use. The reported ENU vector is only `t_B_M` rotated by the MAVROS attitude at `enu_reference_stamp`; it is not a fixed world-frame extrinsic.

## Calibration method

1. **Static initialization.** The node collects a static window and estimates the PX4 gyro bias. It also uses the PX4 ENU attitude and gravity (`[0, 0, -9.80665]` m/s²) to estimate the static accelerometer bias. Keep the rigid assembly still during this step.
2. **Excitation collection.** Roll, pitch, and yaw are collected sequentially. Each axis must reach the target angle in both directions. Progress requires both PX4 gyro motion and a corresponding change in FAST-LIO odometry quaternions. Move slowly, with clear acceleration and deceleration; a constant-rate rotation is insufficient for the lever arm.
3. **Quaternion angular motion.** The node ignores the commonly-zero `Odometry.twist` fields. For adjacent FAST-LIO orientations it computes
   `delta_R = R_i^T R_(i+1)`, then `omega_M = Log(delta_R) / dt`. Angular acceleration is obtained by differentiating these body-frame angular velocities. The FAST-LIO pose quaternion is therefore the direct source of rotational motion.
4. **Rotation alignment.** At overlapping timestamps, PX4 angular velocity and FAST-LIO angular velocity are matched by an SVD orthogonal Procrustes solve to estimate `R_B_M`. The gyro RMSE is reported as an independent rotation consistency check.
5. **Lever-arm translation.** FAST-LIO pose positions are differentiated with a wider time window to obtain linear acceleration in the FAST-LIO body frame. The rigid-body relation is solved in batch:

   `a_B - R_B_M a_M = R_B_M (alpha_M x t_M_B + omega_M x (omega_M x t_M_B)) + bias`

   where `t_M_B` is the inverse-convention translation used internally. The resulting value is converted to `t_B_M`, the output convention described above. Translation is accepted only when the system is well-conditioned and has sufficient angular acceleration.
6. **Validation.** After yaw reaches 100%, the node provides a settling period, then requires a final static window. It rejects results with insufficient samples, failed static validation, large residuals, or a rank-deficient translation system.

This is an excitation-based batch estimate, not a guarantee of accuracy from a single run. Repeat the calibration and compare the fixed FLU result with a physical measurement before using it in flight software.

## Topics and configuration

```yaml
slam_odom_topic: "/Odometry"
body_imu_topic: "/mavros/imu/data"
```

The odometry pose must be metric and time-overlapped with the IMU streams. Adjust scale parameters only when the driver units require it.

## Build and run

```bash
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs libeigen3-dev
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd .. && catkin_make
source devel/setup.bash
roslaunch mid360_px4_calibration online_calibration.launch
```

Keep still during initialization, follow roll/pitch/yaw prompts, then place the assembly down during the settling period and keep it still for final validation. Outputs are `config/online_calibration_result.yaml` and `.txt`.

## License

MIT. See [LICENSE](LICENSE).
