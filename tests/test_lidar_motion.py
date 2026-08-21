import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from mid360_px4_calibration.lidar_motion import PointCloudScan, estimate_lidar_motion


class LidarMotionTests(unittest.TestCase):
    def test_icp_recovers_scan_motion(self):
        rng = np.random.default_rng(11)
        world_points = rng.uniform([-4.0, -3.0, -1.0], [4.0, 3.0, 2.0], size=(1200, 3))
        scans = []
        for i in range(8):
            R_w_l = Rotation.from_euler("xyz", [0.01 * i, -0.015 * i, 0.025 * i]).as_matrix()
            p_w_l = np.array([0.04 * i, 0.01 * i * i, 0.0])
            points = (R_w_l.T @ (world_points - p_w_l).T).T
            scans.append(PointCloudScan(float(i) * 0.1, points, "lidar"))
        motion = estimate_lidar_motion(
            scans,
            voxel_size=0.05,
            max_correspondence_m=0.5,
            max_iterations=30,
            min_correspondences=100,
            max_points=1500,
        )
        self.assertGreaterEqual(len(motion.timestamps), 7)
        self.assertLess(np.mean(motion.fitness), 0.03)
        self.assertGreater(np.std(motion.gyro[:, 2]), 0.0)


if __name__ == "__main__":
    unittest.main()
