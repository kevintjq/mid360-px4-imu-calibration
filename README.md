# Mid360 / PX4 IMU Extrinsic Calibration

English · [中文操作说明](README_CN.md)

ROS1 / Ubuntu 20.04 C++ tool that reads FAST-LIO `/Odometry` and MAVROS `/mavros/imu/data`, guides an eight-step collection, and batch-estimates rotation, lever arm, and combined acceleration bias. It publishes no flight-control messages.

## Quick start

```bash
sudo apt install ros-noetic-roscpp ros-noetic-roslib ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs libeigen3-dev
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch mid360_px4_calibration online_calibration.launch
```

For an existing checkout, run `git pull --ff-only` inside the repository and rebuild the workspace. Start MAVROS and FAST-LIO first. Use a dedicated terminal for the calibrator to avoid interleaved logs.

Requirements: rigid assembly; tracked metric odometry; valid ENU-referenced IMU attitude; FLU angular velocity in rad/s; accelerometer specific force including gravity in m/s²; synchronized, increasing timestamps. Invalid/duplicate/out-of-order frames are discarded; frame changes, stale data, and timeouts stop collection. **200 Hz odometry is supported**, independently of IMU rate. Higher rates do not shorten minimum collection times; republishing old poses adds no information.

## Eight-step procedure

No keypress is required. Start on a stable surface, pick up once after initialization, stay handheld between axes, and put down after all three axes.

| Step | Operation | Default requirement |
| --- | --- | --- |
| 01 / 08 Connect | Start both sources; keep the assembly resting | Valid data within 30 s |
| 02 / 08 Initialize | Rest on a table or ground, hands off | Full stable 8 s; at least 100 IMU samples |
| 03 / 08 Pick up | Lift roughly 10 cm or more away from the initial position | At least 5 s of preparation; odometry displacement ≥8 cm sustained for 0.5 s |
| 04 / 08 Roll | Hold neutral briefly, then rock left/right about ±20–35° around body X | Stable 2 s, then at least 20 s of collection |
| 05 / 08 Pitch | Stay handheld, pause neutral, tilt forward/back about ±20–35° around body Y | Stable 2 s, then at least 20 s of collection |
| 06 / 08 Yaw | Stay handheld, pause neutral, stay approximately level and turn left/right about ±30–45° | Stable 2 s, then at least 20 s of collection |
| 07 / 08 Put down | Place gently on a stable surface, release and let vibration stop | 5 s handling buffer followed by a full stable 5 s |
| 08 / 08 Solve | Wait for quality gates and result files | Save only after all quality gates pass |

Pickup does not advance on a countdown alone: it waits for actual displacement, timing out after 120 s. This is relative distance, independent of world-axis convention; it does not infer altitude or physical table contact. Pickup, putdown, and neutral preparation data are excluded from the three motion windows.

Each rocking cycle should take roughly 3–5 s with natural acceleration/deceleration; typically do 3–5 cycles per axis and continue until all conditions pass. Avoid abrupt shaking, continuous spinning, and large translation.

| Per-axis condition | Default target |
| --- | --- |
| Collection / active movement | At least 20 / 8 s |
| Positive / negative angle | At least 1.5 rad (~86°) each, cumulative |
| IMU / odometry samples | At least 300 / 80 |
| Total odometry rotation | At least 1.8 rad (~103°), confirming odometry actually rotates |

Cumulative angles are not requested single tilts. Mounting orientation is not assumed: matching axis names are not required between streams. Motion alone takes at least 60 s; a full run normally takes about 1.5–3 minutes. Each axis times out after 180 s.

Static checks include per-sample angular velocity, RMS, gyro/acceleration variation, continuity, and attitude changes in both streams. Motion, acceleration steps, gaps, or attitude changes above 1° clear the candidate window and restart the full duration from subsequent clean data. Slow rotation, opposite rotations, and brief movement cannot bypass checks through averaging.

## Terminal display

The Chinese UTF-8 interface uses a consistent card with the current step, action, overall progress, measured values/targets, and remaining requirements. Green `✓` means passed; yellow `○` means pending. Final failures use red `✗` and explicit reasons. Quality arrows `↑ / ↓` show threshold direction.

Columns align by character display width. Cards refresh in place when the terminal is large enough. Small terminals, `TERM=dumb`, and redirected output use periodic logs; stage changes print immediately. Ordinary redirected output contains no terminal escape sequences. `NO_COLOR=1` disables color while retaining supported in-place refresh. The dashboard is not a raw-data recording.

## Frames and result files

Default files:

- `~/.ros/mid360_px4_calibration/result.yaml`
- `~/.ros/mid360_px4_calibration/result.txt`

The transform is `p_B = R_B_M * p_M + t_B_M`. `M` must be the Mid360 IMU body represented by FAST-LIO; `B` is the MAVROS PX4 FLU reference body. Verify reference origins against the configuration consuming the result. `t_B_M` is the Mid360 origin relative to the PX4 reference origin, expressed in FLU.

FAST-LIO world (usually `camera_init`) need not be ENU. Position derivatives are transformed back into body coordinates. ENU translation is only the fixed FLU offset rotated at `enu_reference_stamp`, not a fixed world-frame extrinsic.

Files include rotation matrix, quaternion `[x,y,z,w]`, fixed FLU translation, reference ENU translation, biases, timing convention, metrics/thresholds, and collection intervals. `solver_model: matched_integral_v1` identifies the matched-window estimator. Matrix/quaternion output replaces legacy `rpy_mid360_to_body_XYZ_deg`; `translation_body_FLU_m` remains an alias.

Absolute, `~/`, and package-relative output paths are supported; parents are created. Rejected calibrations preserve previous results. Files are checked before individual renames, which are not a cross-file atomic transaction; partial save failures report errors. Exit codes: `0` accepted and saved, `2` configuration/collection/save failure, `3` solve/quality rejection.

## Configuration and quality gates

See [online_calibration.yaml](config/online_calibration.yaml). Supply a custom file with:

```bash
roslaunch mid360_px4_calibration online_calibration.launch config:=/absolute/path/custom.yaml
```

| Parameter | Default | Meaning |
| --- | --- | --- |
| `pickup_min_duration_s` | 5 | Minimum pickup preparation time |
| `pickup_min_displacement_m` | 0.08 | Displacement from the initial resting position |
| `pickup_timeout_s` / `putdown_buffer_s` | 120 / 5 | Pickup timeout / putdown buffer |
| `axis_min_duration_s` / `axis_min_active_s` | 20 / 8 | Per-axis collection / active time |
| `axis_direction_target_rad` | 1.5 | Cumulative angle per direction |
| `static_max_rotation_deg` | 1 | Static-reference attitude-change limit |
| `imu_time_offset_s` | 0 | Aligned IMU stamp = raw stamp + offset |
| `max_gyro_rmse_rad_s` | 0.12 | Gyro vector RMSE ceiling |
| `max_gyro_relative_rmse` / `min_gyro_correlation` | 0.25 / 0.95 | Relative RMSE ceiling / agreement floor |
| `max_accel_rmse_m_s2` | 0.75 | Acceleration vector RMSE ceiling |
| `min_rotation_excitation_ratio` | 0.02 | Three-axis excitation floor |
| `max_translation_condition` | 1000 | Bias-eliminated lever-arm condition ceiling |
| `min_translation_excitation_rad_s2` | 0.03 | Weakest-direction absolute excitation floor |
| `min_angular_accel_rms_rad_s2` | 0.1 | Angular acceleration RMS floor |
| `max_lever_arm_m` | 2 | Physical lever-arm length ceiling |

The estimator uses **matched-window integral equations**, retaining the default 0.3 s half-window:

1. Rotation aligns odometry endpoint-relative rotations against bias-corrected gyro rotation integrals over the same interval. Gyro RMSE, relative RMSE, and agreement compare rotation vectors divided by interval duration, i.e. matched interval-average motion.
2. Translation applies the same discrete second-derivative stencil to position and rotation matrices, integrating gravity-corrected IMU acceleration with its corresponding Green kernel. Matching temporal responses removes the bias from comparing smoothed derivatives against instantaneous acceleration. Rotation-log derivatives are used only for motion selection and angular-acceleration diagnostics.
3. Lever arm and constant body acceleration bias are jointly estimated. The rotating integrated bias basis is projected out before observability checks.
4. Integration stays inside valid IMU ranges and cannot cross stage boundaries or long gaps. Kernel normalization is checked. The odometry world need not be ENU; usually-zero odometry twist is ignored.

The solve performs per-window integration; wait for the terminal quality report after collection.
Relative gyro RMSE is `RMSE / sqrt((IMU_RMS² + odom_RMS²) / 2)`. Agreement is normalized aligned-vector dot-product sum without mean subtraction, not a Pearson coefficient. All absolute/relative residual, agreement, excitation, and physical-size gates must pass.

**No automatic time-offset estimation.** Synchronize clocks first, applying a nonzero offset only when known. Investigate timing, units, tracking, and rigid mounting before relaxing gates. Acceptance is not an absolute accuracy guarantee; repeat collection and compare the fixed FLU offset against physical measurements.

## Validation and troubleshooting

Eigen numerical tests need no ROS:

```bash
cd ~/catkin_ws/src/mid360-px4-imu-calibration
c++ -std=c++14 -O1 -Iinclude -I/usr/include/eigen3 \
  tests/calibration_math_test.cpp -o /tmp/calibration_math_test
/tmp/calibration_math_test
```

With testing enabled in the catkin build, enter `~/catkin_ws/build` and run `ctest --output-on-failure -R calibration_math_test`. Tests cover known extrinsics, bias degeneracy, static restarts, attitude changes, relative gyro errors, pickup confirmation, noiseless and multiple noise seeds at 10/20/50/200 Hz, coupled rotations, irregular sampling, epoch timestamps, a non-ENU world, IMU gaps, and non-overlapping streams. The trajectory accuracy gates are now 0.1 mm without noise and 2 mm for fixed noisy cases, replacing the former 2 cm tolerance.

The same 200 Hz odometry / 100 Hz IMU synthetic trajectory gives:

| Data | Former lever error | Matched estimator error |
| --- | ---: | ---: |
| Noiseless | 12.44 mm | About 0.016 mm |
| Fixed noise, seed 42 | 12.72 mm | About 0.29 mm |

Independent Gaussian noise scales are 0.5 mm per position axis, 0.2 mrad attitude about X, 0.003 rad/s per gyro axis, and 0.02 m/s² per acceleration axis. These are particular synthetic results, not a hardware accuracy guarantee. Synchronization errors, correlated odometry drift, attitude errors, and installation rigidity still affect real measurements.

| Symptom | First checks |
| --- | --- |
| Source timeout | Topic updates, valid attitude, synchronized stamps |
| Waiting at pickup | Actually move away from the resting position; check odometry position |
| Handheld preparation restarts | Pause neutral; check vibration and attitude drift |
| Motion progress stalls | Read pending conditions; time, both directions, samples, and odometry rotation all matter |
| Quality rejection | Investigate the failed gates; retain the old result rather than only relaxing thresholds |
| Save failure | Output directory permissions and disk space |

Numerical tests, simulated flows, and terminal checks do not replace a real ROS Noetic/catkin build and hardware validation. Those two validation steps remain outstanding.

## License

MIT. See [LICENSE](LICENSE).
