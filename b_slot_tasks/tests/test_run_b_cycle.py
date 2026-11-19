import tempfile
import unittest
from pathlib import Path

from run_b_cycle import (
    CycleError,
    GripperFeedback,
    Snapshot,
    TeachingPoint,
    execute_task_2,
    run_task,
)


POINT_DATA = {
    "A_middle": ((0, 0, 300, 180, 0, 0), (0, 10, 20, 30, 40, 50)),
    "B_1high": ((100, 0, 220, 180, 0, 0), (10, 20, 30, 40, 50, 60)),
    "B_2": ((100, 0, 180, 180, 0, 30), (12, 22, 32, 42, 52, 62)),
    "B_3": ((100, 0, 140, 180, 0, 60), (14, 24, 34, 44, 54, 64)),
    "B_4low": ((100, 0, 100, 180, 0, 90), (16, 26, 36, 46, 56, 66)),
}
POINTS = {
    name: TeachingPoint(name, tcp, joints, 2, 0, 50, 0, (0, 0, 0, 0))
    for name, (tcp, joints) in POINT_DATA.items()
}


class FakeSession:
    def __init__(
        self,
        gripper="open",
        gripper_fault=0,
        gripper_active=2,
        feedback_status_override=None,
        closed_position=40,
        slip_after_extract=False,
        collision_after_open=False,
    ):
        self.current = "A_middle"
        self.gripper = gripper
        self.gripper_fault = gripper_fault
        self.gripper_active = gripper_active
        self.feedback_status_override = feedback_status_override
        self.closed_position = closed_position
        self.slip_after_extract = slip_after_extract
        self.collision_after_open = collision_after_open
        self.collision_state = 0
        self.moves = []
        self.speeds = []
        self.accelerations = []
        self.global_speeds = []
        self.grip_forces = []
        self.commands = []
        self.stops = 0

    def set_global_speed(self, speed):
        self.global_speeds.append(speed)
        return 0

    def communication_ok(self):
        return True

    def read_points(self, names):
        return {name: POINTS[name] for name in names}

    def snapshot(self):
        point = POINTS[self.current]
        open_mode = self.gripper == "open"
        return Snapshot(
            tcp=point.tcp,
            joints=point.joints,
            tool_id=2,
            wobj_id=0,
            robot_state=1,
            robot_mode=0,
            enabled=True,
            main_code=0,
            sub_code=0,
            emergency_stop=0,
            safety_stop0=0,
            safety_stop1=0,
            motion_done=1,
            queue_len=0,
            gripper_fault=self.gripper_fault,
            gripper_active=self.gripper_active,
            gripper_status=1 if open_mode else 2,
            gripper_position=60 if open_mode else self.closed_position,
            gripper_current=20,
            tcp_speed=(0, 0, 0, 0, 0, 0),
            joint_speed=(0, 0, 0, 0, 0, 0),
            collision_state=self.collision_state,
        )

    def move_linear(self, point, speed, acceleration=100.0):
        self.moves.append(point.name)
        self.speeds.append(speed)
        self.accelerations.append(acceleration)
        previous = self.current
        self.current = point.name
        if (
            self.slip_after_extract
            and previous == "B_4low"
            and point.name == "B_3"
            and self.gripper == "close"
        ):
            self.gripper = "open"
        return 0

    def gripper_feedback(self):
        open_mode = self.gripper == "open"
        status = self.feedback_status_override
        if status is None:
            status = 1 if open_mode else 2
        return GripperFeedback(
            self.gripper_fault,
            self.gripper_active,
            status,
            60 if open_mode else self.closed_position,
            20,
        )

    def open_gripper(self):
        self.commands.append("open")
        self.gripper = "open"
        if self.collision_after_open:
            self.collision_state = 1
        return 0

    def close_gripper(self, force=60.0):
        self.commands.append("close")
        self.grip_forces.append(force)
        self.gripper = "close"
        return 0

    def stop_motion(self):
        self.stops += 1
        return 0


class TaskTests(unittest.TestCase):
    def run_fake_task(self, session, task_number, **kwargs):
        now = [0.0]
        output = []
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = run_task(
            session,
            task_number,
            output_fn=output.append,
            sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
            clock_fn=lambda: now[0],
            log_dir=Path(temporary.name),
            **kwargs,
        )
        return path, output

    def test_task_2_pick_is_independent_and_stops_at_a(self):
        session = FakeSession(gripper="open", closed_position=0)
        path, _ = self.run_fake_task(session, 2, extract_acceleration=20.0)
        expected = ["B_1high", "B_3", "B_4low", "B_3", "B_2", "B_1high", "A_middle"]
        self.assertEqual(session.moves, expected)
        self.assertEqual(session.speeds, [100.0] * len(expected))
        self.assertEqual(session.accelerations, [100.0, 100.0, 100.0, 20.0, 100.0, 100.0, 100.0])
        self.assertEqual(session.global_speeds, [30.0])
        self.assertEqual(session.commands, ["close"])
        self.assertEqual(session.grip_forces, [70.0])
        self.assertEqual(session.current, "A_middle")
        self.assertEqual(session.gripper, "close")
        self.assertIn("b_task2_", Path(path).name)

    def test_task_1_place_is_independent_and_stops_at_a(self):
        session = FakeSession(gripper="close")
        path, output = self.run_fake_task(session, 1)
        expected = ["B_1high", "B_3", "B_4low", "B_3", "B_2", "B_1high", "A_middle"]
        self.assertEqual(session.moves, expected)
        self.assertEqual(session.speeds, [100.0] * len(expected))
        self.assertEqual(session.accelerations, [100.0] * len(expected))
        self.assertEqual(session.global_speeds, [30.0])
        self.assertEqual(session.commands, ["open"])
        self.assertEqual(session.grip_forces, [])
        self.assertEqual(session.current, "A_middle")
        self.assertEqual(session.gripper, "open")
        self.assertIn("b_task1_", Path(path).name)
        self.assertTrue(any("任务1" in line and "生件放入生件槽" in line for line in output))
        self.assertTrue(any("起点" in line and "终点" in line and "预期夹爪" in line for line in output))

    def test_task_2_rejects_closed_start_immediately_without_motion(self):
        session = FakeSession(gripper="close")
        now = [0.0]
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "任务2.*需要.*张开"):
                run_task(
                    session,
                    2,
                            output_fn=lambda text: None,
                    sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
                    clock_fn=lambda: now[0],
                    log_dir=Path(temporary),
                )
        self.assertLess(now[0], 3.0)
        self.assertEqual(session.moves, [])
        self.assertEqual(session.commands, [])

    def test_task_1_rejects_open_start_immediately_without_motion(self):
        session = FakeSession(gripper="open")
        now = [0.0]
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "任务1.*需要.*闭合"):
                run_task(
                    session,
                    1,
                            output_fn=lambda text: None,
                    sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
                    clock_fn=lambda: now[0],
                    log_dir=Path(temporary),
                )
        self.assertLess(now[0], 3.0)
        self.assertEqual(session.moves, [])
        self.assertEqual(session.commands, [])

    def test_task_1_accepts_closed_position_zero_from_real_gripper(self):
        session = FakeSession(gripper="close", closed_position=0)
        self.run_fake_task(session, 1)
        self.assertEqual(session.current, "A_middle")
        self.assertEqual(session.gripper, "open")
        self.assertEqual(session.commands, ["open"])

    def test_task_2_completes_without_operator_input(self):
        session = FakeSession(gripper="open")
        self.run_fake_task(session, 2)
        self.assertEqual(session.current, "A_middle")
        self.assertEqual(session.gripper, "close")

    def test_task_1_completes_without_operator_input(self):
        session = FakeSession(gripper="close")
        self.run_fake_task(session, 1)
        self.assertEqual(session.current, "A_middle")
        self.assertEqual(session.gripper, "open")
        self.assertEqual(session.commands, ["open"])

    def test_task_2_aborts_if_part_slips_after_first_lift(self):
        session = FakeSession(gripper="open", slip_after_extract=True)
        now = [0.0]
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "no longer closed|dropped|夹持"):
                run_task(
                    session,
                    2,
                    output_fn=lambda text: None,
                    sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
                    clock_fn=lambda: now[0],
                    log_dir=Path(temporary),
                )
        self.assertLess(now[0], 3.0)
        self.assertNotIn("A_middle", session.moves)
        self.assertEqual(session.current, "B_3")

    def test_task_1_collision_after_open_blocks_return(self):
        session = FakeSession(gripper="close", collision_after_open=True)
        now = [0.0]
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "collision state is active"):
                run_task(
                    session,
                    1,
                    output_fn=lambda text: None,
                    sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
                    clock_fn=lambda: now[0],
                    log_dir=Path(temporary),
                )
        self.assertEqual(session.current, "B_4low")
        self.assertEqual(session.commands, ["open"])
        self.assertNotIn("A_middle", session.moves)

    def test_sdk_stale_gripper_fault_and_inactive_mask_do_not_block(self):
        session = FakeSession(
            gripper="open",
            gripper_fault=1,
            gripper_active=0,
            feedback_status_override=0,
        )
        self.run_fake_task(session, 2)
        self.assertEqual(session.commands, ["close"])
        self.assertEqual(session.current, "A_middle")
        self.assertEqual(session.gripper, "close")

    def test_gripper_status_dropped_still_aborts_before_motion(self):
        session = FakeSession(gripper="open", feedback_status_override=3)
        now = [0.0]
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "掉落状态3"):
                run_task(
                    session,
                    2,
                            output_fn=lambda text: None,
                    sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
                    clock_fn=lambda: now[0],
                    log_dir=Path(temporary),
                )
        self.assertEqual(session.moves, [])
        self.assertEqual(session.commands, [])

    def test_extract_speed_out_of_range_is_rejected(self):
        session = FakeSession(gripper="open")
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "extract speed"):
                run_task(
                    session,
                    2,
                    extract_speed=0.0,
                            output_fn=lambda text: None,
                    sleep_fn=lambda seconds: None,
                    clock_fn=lambda: 0.0,
                    log_dir=Path(temporary),
                )
        self.assertEqual(session.moves, [])

    def test_grip_force_out_of_range_is_rejected(self):
        session = FakeSession(gripper="open")
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "grip force"):
                run_task(
                    session,
                    2,
                    grip_force=0.0,
                            output_fn=lambda text: None,
                    sleep_fn=lambda seconds: None,
                    clock_fn=lambda: 0.0,
                    log_dir=Path(temporary),
                )
        self.assertEqual(session.moves, [])

    def test_signal_entrypoint_stops_motion_on_wrong_task_state(self):
        session = FakeSession(gripper="close")
        now = [0.0]
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "任务2.*需要.*张开"):
                execute_task_2(
                    session,
                            output_fn=lambda text: None,
                    sleep_fn=lambda seconds: now.__setitem__(0, now[0] + seconds),
                    clock_fn=lambda: now[0],
                    log_dir=Path(temporary),
                )
        self.assertEqual(session.stops, 1)
        self.assertEqual(session.moves, [])

    def test_invalid_task_number_is_rejected(self):
        session = FakeSession()
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(CycleError, "task number"):
                run_task(
                    session,
                    3,
                            output_fn=lambda text: None,
                    sleep_fn=lambda seconds: None,
                    clock_fn=lambda: 0.0,
                    log_dir=Path(temporary),
                )
        self.assertEqual(session.moves, [])


if __name__ == "__main__":
    unittest.main()
