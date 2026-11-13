<h1 align="center">Mid360 / PX4 IMU Calibration</h1>

<p align="center">Guided collection · Three-axis motion · Rotation and lever-arm estimation</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS-Noetic-22314E?style=flat-square" alt="ROS Noetic">
  <img src="https://img.shields.io/badge/C%2B%2B-14-00599C?style=flat-square" alt="C++14">
  <img src="https://img.shields.io/badge/Ubuntu-20.04-E95420?style=flat-square" alt="Ubuntu 20.04">
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square" alt="MIT License">
</p>

<p align="center">English · <a href="README_CN.md">简体中文</a></p>

<p align="center">
  <a href="#quick-start">Quick start</a> ·
  <a href="#inputs">Inputs</a> ·
  <a href="#collection-guide">Collection guide</a> ·
  <a href="#results-and-frames">Results</a> ·
  <a href="#configuration">Configuration</a> ·
  <a href="#troubleshooting">Troubleshooting</a>
</p>

---

A ROS1 tool that reads FAST-LIO odometry and MAVROS IMU data, guides a handheld collection, and batch-estimates **rotation, lever arm, and combined acceleration bias from FAST-LIO body to the PX4 FLU reference body**. It reads data only and publishes no flight-control messages.

| Feature | Description |
| --- | --- |
| Guided collection | Eight automatic stages with actions, progress and remaining requirements |
| Data checks | Stable windows, motion in both directions, timing continuity and sample counts |
| Joint estimation | Matched-window integration for rotation, lever arm and acceleration bias |
| Quality gates | Save only after acceptance; rejected results preserve previous files |

## Quick start

Recommended environment: **Ubuntu 20.04, ROS Noetic, catkin and Eigen3**. Rigidly mount the Mid360 and flight controller, then start FAST-LIO and MAVROS before running the calibrator.

### 1. Install dependencies

**ROS libraries and messages:**

```bash
sudo apt install \
  ros-noetic-roscpp \
  ros-noetic-roslib \
  ros-noetic-sensor-msgs \
  ros-noetic-nav-msgs
```

**Eigen:**

```bash
sudo apt install libeigen3-dev
```

### 2. Get the source

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/mid360-px4-imu-calibration.git
```

### 3. Build

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 4. Run

Load the workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

Rest the assembly on a stable surface and start in a dedicated terminal:

```bash
roslaunch mid360_px4_calibration online_calibration.launch
```

> Pick up once after initialization and put down after all three axes. **Stay handheld between axes.** A complete run usually takes 1.5–3 minutes; the three motion stages alone require at least 60 seconds.

## Inputs

| Topic | Type | Requirement |
| --- | --- | --- |
| `/Odometry` | `nav_msgs/Odometry` | Tracked FAST-LIO body pose, positions in metres |
| `/mavros/imu/data` | `sensor_msgs/Imu` | Valid ENU-referenced attitude, FLU angular velocity and accelerometer specific force without gravity removal |

- Use angular velocity in `rad/s` and specific force in `m/s²`; maintain a rigid assembly.
- Synchronize clocks and use increasing timestamps. Invalid, duplicate and out-of-order frames are discarded; frame changes, stale data and timeouts stop collection.
- **200 Hz odometry is supported**, independently of IMU rate. Higher rates do not shorten collection times; repeating old poses adds no information.
- The FAST-LIO world may be a local initialization frame, not ENU. Its body origin and axes must match the intended result reference.

**Time offset is not estimated automatically.** Synchronize clocks first. Set `imu_time_offset_s` only for a known offset: aligned IMU stamp = raw stamp + offset.

## Collection guide

Follow the current terminal stage. The tool advances automatically when all requirements pass.

| Stage | Action | Default requirement |
| --- | --- | --- |
| 01 Connect | Run both data sources; keep the assembly resting | Valid input within 30 s |
| 02 Initialize | Rest on a table or ground, hands off | Continuous stable 8 s, at least 100 IMU samples |
| 03 Pick up | Move roughly 10 cm or more from the initial position; hold steady | At least 5 s preparation; displacement ≥8 cm sustained for 0.5 s |
| 04 Roll | Pause neutral, then rock left/right about body X by roughly ±20–35° | Stable 2 s, then at least 20 s collection |
| 05 Pitch | Stay handheld; pause neutral, then tilt forward/back by roughly ±20–35° | Stable 2 s, then at least 20 s collection |
| 06 Yaw | Pause neutral; stay approximately level and turn left/right by roughly ±30–45° | Stable 2 s, then at least 20 s collection |
| 07 Put down | Place gently, release and let vibration settle | 5 s handling buffer, then continuous stable 5 s |
| 08 Solve | Wait for computation and the quality report | Save after every quality gate passes |

### Completing each axis

Take roughly **3–5 seconds per rocking cycle**, with natural acceleration and deceleration. Usually perform 3–5 cycles, then continue smoothly until the terminal requirements pass. Avoid abrupt shaking, continuous spinning and large translation.

| Condition | Default per-axis target |
| --- | --- |
| Collection / active motion | At least 20 / 8 s |
| Positive / negative cumulative angle | At least 1.5 rad (~86°) in each direction |
| IMU / odometry samples | At least 300 / 80 |
| Total odometry rotation | At least 1.8 rad (~103°) |

> Angles accumulate over multiple cycles: **a single 86° tilt is not required**. Unknown mounting orientation does not require matching axis names between streams.

<details>
<summary>Pickup confirmation, stable-window resets and terminal display</summary>

- Pickup requires actual relative displacement, not just a countdown, and times out after 120 s. It does not infer table contact or absolute altitude.
- Pickup, putdown and neutral-preparation data are excluded from the solve. Each motion stage times out after 180 s.
- Static checks include individual angular rates, RMS, acceleration variation, continuity and both attitudes. Motion, gaps or an attitude change beyond the default 1° clear the candidate window and restart its full duration.
- The terminal shows the current step, action, progress and measured/target values. `✓` means passed, `○` pending and `✗` rejected; remaining conditions appear at the bottom.
- Small terminals and redirected output use periodic logs. `NO_COLOR=1` disables colour. Use a dedicated terminal to avoid interleaved node logs.

</details>

## Results and frames

Default output files:

| File | Purpose |
| --- | --- |
| `~/.ros/mid360_px4_calibration/result.yaml` | Structured result for downstream configuration |
| `~/.ros/mid360_px4_calibration/result.txt` | Readable result and quality metrics |

The transform is:

```text
p_B = R_B_M × p_M + t_B_M
```

| Symbol | Definition |
| --- | --- |
| `M` | Mid360 IMU body represented by FAST-LIO; verify its actual origin and axes |
| `B` | MAVROS PX4 FLU reference body |
| `R_B_M` | Rotation mapping M-frame vectors into B |
| `t_B_M` | Position of the M origin relative to the B origin, expressed in FLU |

Check the consuming configuration's transform direction and reference origin. Use the fixed FLU lever arm for installation offsets; the ENU translation is its expression at a reference instant, not a fixed world-frame extrinsic.

<details>
<summary>Result fields, saving behaviour and exit codes</summary>

Outputs include `rotation_matrix_B_M`, `rotation_quaternion_xyzw` in `[x,y,z,w]` order, FLU translation, reference ENU translation, biases, timing convention, quality metrics and collection intervals.

`solver_model: matched_integral_v1` identifies the estimator. `translation_body_FLU_m` is retained as a translation-field alias.

Paths support absolute locations, `~/` and package-relative locations; parent directories are created. Quality rejection preserves old results. Files are checked and renamed individually, not as one cross-file atomic transaction.

| Exit code | Meaning |
| --- | --- |
| `0` | Accepted and saved |
| `2` | Configuration, collection or saving failure |
| `3` | Solve failure or quality rejection |

</details>

## Configuration

See [online_calibration.yaml](config/online_calibration.yaml) for all options.

| Parameter | Default | Description |
| --- | --- | --- |
| `slam_odom_topic` | `/Odometry` | FAST-LIO pose |
| `body_imu_topic` | `/mavros/imu/data` | PX4/MAVROS IMU |
| `imu_time_offset_s` | `0.0` | Known IMU time offset, s |
| `static_duration_s` | `8.0` | Initial continuous stable window, s |
| `neutral_duration_s` | `2.0` | Neutral preparation before each axis, s |
| `axis_min_duration_s` | `20.0` | Minimum collection per axis, s |
| `axis_min_active_s` | `8.0` | Minimum active motion per axis, s |
| `axis_direction_target_rad` | `1.5` | Cumulative angle in each direction |
| `pickup_min_displacement_m` | `0.08` | Pickup-confirmation displacement, m |
| `output_yaml` / `output_txt` | Paths above | Result files |

To keep separate parameter files, use the existing launch argument:

```bash
roslaunch mid360_px4_calibration online_calibration.launch \
  config:=/absolute/path/custom.yaml
```

<details>
<summary>Advanced quality gates and estimation model</summary>

| Parameter | Default | Description |
| --- | --- | --- |
| `static_max_rotation_deg` | `1.0` | Stable-window attitude-change limit |
| `derivative_half_window_s` | `0.3` | Matched-window half-width, s |
| `max_gyro_rmse_rad_s` | `0.12` | Gyro vector RMSE ceiling |
| `max_gyro_relative_rmse` | `0.25` | Relative gyro error ceiling |
| `min_gyro_correlation` | `0.95` | Aligned-vector agreement floor |
| `max_accel_rmse_m_s2` | `0.75` | Acceleration vector RMSE ceiling |
| `min_rotation_excitation_ratio` | `0.02` | Three-axis rotation-excitation floor |
| `max_translation_condition` | `1000.0` | Bias-eliminated lever-arm condition ceiling |
| `min_translation_excitation_rad_s2` | `0.03` | Weakest-direction excitation floor |
| `min_angular_accel_rms_rad_s2` | `0.1` | Angular acceleration RMS floor |
| `max_lever_arm_m` | `2.0` | Physical lever-arm length ceiling |

The estimator matches odometry endpoint-relative rotations with gyro integrals over the same windows. Lever-arm equations use the same second-derivative operator for position and rotation matrices, with the corresponding integration kernel for gravity-corrected acceleration. Constant body acceleration bias is jointly estimated and eliminated before observability checks.

Integration cannot cross stage boundaries or long gaps. The odometry world need not be ENU, and potentially zero odometry twist is ignored. Relative gyro RMSE is `RMSE / sqrt((IMU_RMS² + odom_RMS²) / 2)`; agreement is a normalized paired-vector dot-product sum, not a mean-centred Pearson coefficient.

All quality gates must pass. Check timing, units, tracking, rigidity and motion excitation before relaxing thresholds. Acceptance is not an absolute accuracy guarantee; repeat collection and compare against physical installation measurements.

</details>

## Troubleshooting

| Symptom | First checks |
| --- | --- |
| Waiting for sources | Topic updates, valid IMU attitude and synchronized timestamps |
| Pickup does not advance | Actual displacement from rest and changing odometry position |
| Preparation keeps restarting | Pause neutral; check vibration and attitude drift |
| Motion progress stalls | Remaining conditions: time, both directions, samples and rotation |
| Quality rejection | Investigate failed gates rather than only relaxing thresholds |
| Saving fails | Directory permissions and free disk space |

## License

Released under the [MIT License](LICENSE).
