import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from mid360_px4_calibration import CalibrationConfig, ImuSamples, calibrate
from mid360_px4_calibration.filtering import angular_acceleration
from mid360_px4_calibration.math_utils import hat
from mid360_px4_calibration.rotation import wahba_rotation


def synthetic_pair(dt=0.037, noise=0.0):
    rng = np.random.default_rng(7)
    t_mid = np.arange(0.0, 30.0, 0.01)
    R_M_B = Rotation.from_euler("xyz", [15.0, -8.0, 30.0], degrees=True).as_matrix()
    R_B_M = R_M_B.T
    lever = np.array([0.12, -0.05, 0.08])
    gyro_bias = np.array([0.02, -0.01, 0.015])
    accel_bias = np.array([0.08, -0.04, 0.03])

    def omega(t):
        return np.column_stack(
            [
                0.8 * np.sin(1.1 * t) + 0.3 * np.sin(2.3 * t),
                0.7 * np.cos(0.8 * t + 0.2),
                0.6 * np.sin(1.7 * t + 0.5),
            ]
        )

    def accel_mid(t):
        return np.column_stack(
            [
                0.3 * np.sin(0.7 * t),
                0.2 * np.cos(0.9 * t),
                9.81 + 0.15 * np.sin(0.5 * t),
            ]
        )

    gyro_m = omega(t_mid)
    mid = ImuSamples(t_mid, gyro_m, accel_mid(t_mid), frame_id="mid")
    alpha_m = angular_acceleration(mid, 0.07, 3)

    t_body = np.arange(0.1, 29.9, 0.01)
    phys = t_body - dt
    gyro_phys = omega(phys)
    alpha_phys = np.empty_like(gyro_phys)
    for axis in range(3):
        alpha_phys[:, axis] = np.interp(phys, t_mid, alpha_m[:, axis])
    accel_phys = accel_mid(phys)
    accel_b = np.zeros_like(accel_phys)
    for i, (om, al) in enumerate(zip(gyro_phys, alpha_phys)):
        J = hat(al) + hat(om) @ hat(om)
        accel_b[i] = R_B_M @ (accel_phys[i] + J @ lever + accel_bias)
    gyro_b = (R_B_M @ gyro_phys.T).T + gyro_bias

    if noise > 0.0:
        gyro_m = gyro_m + rng.normal(0.0, noise, gyro_m.shape)
        gyro_b = gyro_b + rng.normal(0.0, noise, gyro_b.shape)
        accel_b = accel_b + rng.normal(0.0, noise * 5.0, accel_b.shape)

    return ImuSamples(t_mid, gyro_m, accel_mid(t_mid), "mid"), ImuSamples(t_body, gyro_b, accel_b, "body"), R_M_B, lever, dt


class CalibrationTests(unittest.TestCase):
    def test_wahba_rotation_direction(self):
        mid, body, R_M_B, _, dt = synthetic_pair()
        body_at_mid = np.column_stack([np.interp(mid.timestamps + dt, body.timestamps, body.gyro[:, i]) for i in range(3)])
        mask = (mid.timestamps + dt >= body.start) & (mid.timestamps + dt <= body.end)
        R_B_M = wahba_rotation(mid.gyro[mask], body_at_mid[mask])
        err = Rotation.from_matrix(R_B_M @ R_M_B).magnitude()
        self.assertLess(err, 2.0e-6)

    def test_synthetic_calibration_recovers_parameters(self):
        mid, body, R_M_B, lever, dt = synthetic_pair()
        cfg = CalibrationConfig(
            time_search_min_s=-0.1,
            time_search_max_s=0.1,
            max_samples=2000,
            trim_s=0.2,
            gyro_sigma_rad_s=0.02,
            accel_sigma_m_s2=0.1,
            accepted_accel_rmse_m_s2=0.5,
        )
        result = calibrate(mid, body, cfg)
        rot_err = Rotation.from_matrix(result.R_M_B.T @ R_M_B).magnitude()
        self.assertLess(rot_err, 2.0e-3)
        self.assertLess(abs(result.dt_B_wrt_M - dt), 2.0e-3)
        np.testing.assert_allclose(result.translation_M_B_m, lever, atol=2.0e-2)


if __name__ == "__main__":
    unittest.main()
