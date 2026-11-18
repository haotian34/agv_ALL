import unittest

import numpy as np

from handeye_validate import camera_point_to_base, summarize_base_points


class HandeyeValidateTests(unittest.TestCase):
    def test_camera_point_to_base_applies_camera_offset_and_units(self):
        handeye = np.eye(4)
        handeye[:3, 3] = [10.0, 20.0, 30.0]
        base_point = camera_point_to_base(
            [100.0, 200.0, 300.0, 0.0, 0.0, 0.0],
            [0.001, 0.002, 0.003],
            handeye,
        )
        np.testing.assert_allclose(base_point, [111.0, 222.0, 333.0], atol=1e-9)

    def test_summary_reports_repeatability_about_current_mean(self):
        points = np.array([[10.0, 20.0, 30.0], [14.0, 24.0, 38.0]])
        summary = summarize_base_points(points)
        np.testing.assert_allclose(summary["mean_mm"], [12.0, 22.0, 34.0])
        np.testing.assert_allclose(summary["axis_range_mm"], [4.0, 4.0, 8.0])
        self.assertAlmostEqual(summary["rms_mm"], np.sqrt(24.0))
        self.assertAlmostEqual(summary["max_mm"], np.sqrt(24.0))
        self.assertEqual(summary["count"], 2)


if __name__ == "__main__":
    unittest.main()
