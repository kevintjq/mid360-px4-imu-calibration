# Mid360 / PX4 IMU Extrinsic Calibration

English | [中文操作说明](README_CN.md)

ROS1 / Ubuntu 20.04 C++ calibration of the FAST-LIO body relative to a PX4 FLU body reference. Reads `/Odometry` and `/mavros/imu/data`, guides motion collection, and batch-estimates rotation, lever arm, and a combined accelerometer bias. Publishes no flight-control messages.

## Build and run

```bash
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs libeigen3-dev
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch mid360_px4_calibration online_calibration.launch
```

Start MAVROS and FAST-LIO first. Odometry must remain tracked and metric. IMU input must contain valid ENU-referenced attitude, FLU body angular velocity in rad/s, and accelerometer specific force including gravity in m/s². The streams must share a time base. Invalid data and duplicate/out-of-order stamps are discarded; changing frame IDs aborts collection.

## Motion procedure

Keep the assembly rigid. The terminal displays Chinese operation prompts, progress, elapsed/active time, positive/negative rotation totals, and sample counts. A TTY refreshes in place; redirected logs produce plain periodic updates. Set `NO_COLOR=1` to disable colors.

| Stage | Operation | Default requirement |
| --- | --- | --- |
| Initialization | Rest on a stable surface | 8 s of stable data, at least 100 IMU samples |
| Before each axis | Return to a comfortable neutral pose and pause | 2 s of stable data |
| Roll | Gently rock left/right around the body's front-back axis, about ±20–35° | At least 20 s |
| Pitch | Gently tilt forward/back around the body's left-right axis, about ±20–35° | At least 20 s |
| Yaw | Stay approximately level and turn left/right, about ±30–45° | At least 20 s |
| Final validation | Put down gently and let vibration stop | 5 s of stable data, at least 100 IMU samples |

Use natural acceleration/deceleration, about 3–5 s per rocking cycle and typically 3–5 cycles per axis. Continue smoothly until all requirements pass. Avoid abrupt shaking, continuous spinning, or large translation. Each direction must accumulate **1.5 rad (~86°)**; this is cumulative, not a requested single tilt.

Each axis also requires at least 8 s of active motion, 300 IMU samples, 80 odometry samples, and corresponding odometry rotation. All requirements must pass together. Odometry rotation is measured independently of mounting orientation, so a 90° mounting rotation does not block collection by requiring matching axis names. Motion alone takes at least 60 s; a complete run normally takes roughly 1.5–3 minutes. Each axis times out after 180 s rather than waiting forever.

Static checks use bias-corrected gyro RMS, gyro/acceleration variation, sample count, window duration, and continuity. Opposite rotations cannot cancel to pass the check. Movement after putting the assembly down extends the wait automatically. Missing/stale data and timeouts produce explicit errors.

## Frames and outputs

The transform is defined by:

```text
p_B = R_B_M * p_M + t_B_M
```

`M` must be the Mid360 IMU body represented by FAST-LIO, and `B` is the MAVROS PX4 FLU body reference. Verify that their reference origins match the configuration consuming the result. `t_B_M` is the Mid360 origin relative to the PX4 reference origin, expressed in FLU.

FAST-LIO world (usually `camera_init`) need not be ENU: position derivatives are transformed back to the FAST-LIO body. The ENU translation is only the fixed FLU lever arm rotated at `enu_reference_stamp`, not a fixed world-frame extrinsic.

Default files are `~/.ros/mid360_px4_calibration/result.yaml` and `result.txt`. Parent directories are created. Absolute, `~/`, and package-relative paths are supported. Files are written and checked before individual renames; the two renames are not a cross-file atomic transaction, and partial save failures return an error. Rejected calibrations do not overwrite existing results.

Files include rotation matrix, quaternion `[x,y,z,w]`, fixed FLU translation, reference ENU translation, gyro/combined accelerometer bias, timestamp offset convention, quality metrics, thresholds, and motion windows. The matrix/quaternion replace the ambiguous legacy `rpy_mid360_to_body_XYZ_deg` field; `translation_body_FLU_m` remains a translation alias.

## Estimation and quality gates

- Record the actual initialization sample boundaries, avoiding startup delays contaminating bias estimates with later movement.
- Fit local cubic polynomials to position and local rotation logarithms (default half-window 0.3 s) to estimate acceleration, angular velocity, and angular acceleration. Ignore usually-zero odometry twist, stage boundaries, long gaps, and static samples that could dilute motion residuals.
- Interpolate strictly inside the IMU time range and reject long gaps. Estimate rotation with SVD and check three-axis excitation plus gyro vector RMSE.
- Solve the rigid-body acceleration relation for lever arm and constant combined bias. Eliminate the bias before testing the lever-arm condition number and weakest-direction excitation, accounting for their joint observability.
- Accept only with adequate rotation consistency, acceleration residual, angular acceleration, and physical lever-arm length. Report every failed quality gate and save only accepted results.

See [config/online_calibration.yaml](config/online_calibration.yaml) for all parameters. Defaults include gyro vector RMSE ≤0.12 rad/s, acceleration vector RMSE ≤0.75 m/s², rotation excitation ratio ≥0.02, bias-eliminated translation condition ≤1000, weakest translation excitation ≥0.03 rad/s², and lever-arm length ≤2 m.

**Time offset is not estimated automatically.** `aligned_imu_stamp = raw_imu_stamp + imu_time_offset_s`, with default zero. Synchronize clocks first and set a nonzero value only for a known offset. Investigate timing, units, tracking, and rigid mounting before relaxing thresholds. Defaults reject clearly poor data; they do not guarantee accuracy. Repeat runs and compare the fixed FLU offset against physical measurements.

## Tests

The numerical tests need Eigen but no ROS:

```bash
cd ~/catkin_ws/src/mid360-px4-imu-calibration
c++ -std=c++14 -O1 -Iinclude -I/usr/include/eigen3 \
  tests/calibration_math_test.cpp -o /tmp/calibration_math_test
/tmp/calibration_math_test
```

Alternatively, after `catkin_make`, enter `~/catkin_ws/build` and run `ctest --output-on-failure -R calibration_math_test`. Coverage includes known extrinsics and bias recovery, 90° mounting, bias-confounded degeneracy, oscillatory static rejection, strict time overlap/gap handling, irregular-sampling derivatives, and noisy three-axis trajectories at 10/20/50 Hz. Synthetic tests do not replace real ROS/rosbag and hardware validation.

## License

MIT. See [LICENSE](LICENSE).
