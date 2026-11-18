import unittest

import grasp_lift_test
from grasp_lift_test import build_grasp_lift_poses, starting_pose_is_expected


class FakeGripper:
    def __init__(self, positions):
        self._positions = list(positions)

    def GetGripperCurPosition(self):
        position = self._positions.pop(0) if self._positions else 58
        return (0, 0, position)


class GraspLiftTests(unittest.TestCase):
    def setUp(self):
        self.log = {
            "grasp_pose": [452.0, 29.6, 94.0, -179.2, -0.6, -31.1],
            "pregrasp_pose": [452.0, 29.6, 104.0, -179.2, -0.6, -31.1],
        }
        self.offset_record = {
            "manual_grasp_tcp_pose": [446.6468200683594, 30.822099685668945, 84.38601684570312,
                                      177.5594482421875, 1.8260236978530884, -30.701887130737305]
        }

    def require_function(self, name):
        function = getattr(grasp_lift_test, name, None)
        if function is None:
            self.fail(f"missing required safety function: {name}")
        return function

    def test_lift_is_only_20mm_above_accepted_grasp(self):
        result = build_grasp_lift_poses(self.log, lift_mm=20.0)
        self.assertEqual(result["grasp_pose"], self.log["grasp_pose"])
        self.assertEqual(result["lift_pose"], [452.0, 29.6, 114.0, -179.2, -0.6, -31.1])

    def test_start_must_match_the_accepted_10mm_pose(self):
        self.assertTrue(starting_pose_is_expected(self.log["pregrasp_pose"], self.log["pregrasp_pose"]))
        moved = list(self.log["pregrasp_pose"])
        moved[2] += 6.0
        self.assertFalse(starting_pose_is_expected(moved, self.log["pregrasp_pose"]))

    def test_old_collision_log_is_rejected_by_independent_z_guard(self):
        validate = self.require_function("validate_grasp_reference")
        with self.assertRaisesRegex(ValueError, "below manual reference"):
            validate(self.log["grasp_pose"][:2] + [81.34530938865464] + self.log["grasp_pose"][3:],
                     self.offset_record)

    def test_grasp_at_manual_z_passes_independent_z_guard(self):
        validate = self.require_function("validate_grasp_reference")
        validate(self.offset_record["manual_grasp_tcp_pose"], self.offset_record)

    def test_real_open_position_58_is_not_accepted_as_closed(self):
        confirm = self.require_function("confirm_gripper_holding")
        robot = FakeGripper([58, 58, 58, 58])

        def unexpected_input(_):
            self.fail("must not ask for HOLDING while gripper remains open")

        confirmed, samples = confirm(robot, input_fn=unexpected_input, max_attempts=4,
                                     poll_interval_s=0.0, sleep_fn=lambda _: None)
        self.assertFalse(confirmed)
        self.assertEqual(samples, [58, 58, 58, 58])

    def test_closed_feedback_still_requires_holding_confirmation(self):
        confirm = self.require_function("confirm_gripper_holding")
        robot = FakeGripper([58, 40, 35, 35, 35])
        confirmed, samples = confirm(robot, input_fn=lambda _: "LIFT", max_attempts=5,
                                     poll_interval_s=0.0, sleep_fn=lambda _: None)
        self.assertFalse(confirmed)
        self.assertEqual(samples[-3:], [40, 35, 35])

    def test_stable_closed_feedback_and_holding_allow_lift(self):
        confirm = self.require_function("confirm_gripper_holding")
        robot = FakeGripper([58, 40, 35, 35, 35])
        confirmed, _ = confirm(robot, input_fn=lambda _: "HOLDING", max_attempts=5,
                               poll_interval_s=0.0, sleep_fn=lambda _: None)
        self.assertTrue(confirmed)

    def test_position_feedback_must_have_no_error_or_fault(self):
        read_position = self.require_function("read_gripper_position")

        class FaultedGripper:
            def GetGripperCurPosition(self):
                return (0, 1, 0)

        with self.assertRaises(RuntimeError):
            read_position(FaultedGripper())


if __name__ == "__main__":
    unittest.main()

