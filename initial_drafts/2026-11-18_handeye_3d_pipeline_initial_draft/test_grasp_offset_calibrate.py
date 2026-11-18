import unittest

from grasp_offset_calibrate import compute_grasp_offset


class GraspOffsetTests(unittest.TestCase):
    def test_offset_is_actual_manual_tcp_minus_visual_target(self):
        result = compute_grasp_offset(
            [454.0, 32.0, 94.0],
            [460.0, 30.0, 110.0, -180.0, 0.0, -45.0],
        )
        self.assertEqual(result["offset_base_mm"], [6.0, -2.0, 16.0])
        self.assertEqual(result["grasp_orientation_rpy_deg"], [-180.0, 0.0, -45.0])


if __name__ == "__main__":
    unittest.main()
