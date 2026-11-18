import unittest

from pregrasp_dry_run import compute_dry_run_poses


class PregraspDryRunTests(unittest.TestCase):
    def setUp(self):
        self.record = {
            "offset_base_mm": [-2.5, -2.0, -0.5],
            "grasp_orientation_rpy_deg": [-179.0, -0.5, -31.0],
            "manual_grasp_tcp_pose": [451.5, 30.5, 90.0, -179.0, -0.5, -31.0],
        }

    def test_pregrasp_is_above_compensated_grasp_with_fixed_orientation(self):
        result = compute_dry_run_poses([454.0, 32.5, 90.5], self.record, height_mm=80.0)
        self.assertEqual(result["grasp_pose"], [451.5, 30.5, 90.0, -179.0, -0.5, -31.0])
        self.assertEqual(result["pregrasp_pose"], [451.5, 30.5, 170.0, -179.0, -0.5, -31.0])

    def test_motion_is_rejected_when_vision_disagrees_with_manual_reference(self):
        bad_record = dict(self.record)
        bad_record["manual_grasp_tcp_pose"] = [470.0, 30.5, 90.0, -179.0, -0.5, -31.0]
        with self.assertRaises(ValueError):
            compute_dry_run_poses([454.0, 32.5, 90.5], bad_record, height_mm=80.0)

    def test_pregrasp_supports_final_10mm_check(self):
        result = compute_dry_run_poses([454.0, 32.5, 90.5], self.record, height_mm=10.0)
        self.assertEqual(result["pregrasp_pose"], [451.5, 30.5, 100.0, -179.0, -0.5, -31.0])

    def test_height_below_10mm_is_rejected(self):
        with self.assertRaises(ValueError):
            compute_dry_run_poses([454.0, 32.5, 90.5], self.record, height_mm=5.0)

    def test_collision_case_is_rejected_despite_total_error_under_10mm(self):
        collision_record = {
            "offset_base_mm": [-8.074638826627051, -1.0369947211982655, -10.291987267161403],
            "grasp_orientation_rpy_deg": [177.5594482421875, 1.8260236978530884, -30.701887130737305],
            "manual_grasp_tcp_pose": [446.6468200683594, 30.822099685668945, 84.38601684570312,
                                      177.5594482421875, 1.8260236978530884, -30.701887130737305],
        }
        with self.assertRaisesRegex(ValueError, "below manual reference"):
            compute_dry_run_poses([453.827, 31.618, 91.637], collision_record, height_mm=10.0)

    def test_grasp_more_than_1mm_below_manual_z_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "below manual reference"):
            compute_dry_run_poses([454.0, 32.5, 89.0], self.record, height_mm=10.0)


if __name__ == "__main__":
    unittest.main()
