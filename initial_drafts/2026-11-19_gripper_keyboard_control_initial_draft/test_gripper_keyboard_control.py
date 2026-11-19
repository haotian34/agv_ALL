import unittest

import gripper_keyboard_control


class FakeRobot:
    def __init__(self):
        self.moves = []
        self.reads = 0

    def MoveGripper(self, *args):
        self.moves.append(args)
        return 0

    def GetGripperCurPosition(self):
        self.reads += 1
        return (0, 0, 58 - self.reads)


class RealtimeGripperTests(unittest.TestCase):
    def require_controller_class(self):
        controller_class = getattr(gripper_keyboard_control, "LiveGripperController", None)
        if controller_class is None:
            self.fail("missing non-blocking LiveGripperController")
        return controller_class

    def test_close_command_returns_without_a_blocking_position_poll(self):
        controller_class = self.require_controller_class()
        robot = FakeRobot()
        controller = controller_class(robot, output=lambda _: None, clock=lambda: 0.0)

        controller.handle_key("G")

        self.assertEqual(len(robot.moves), 1)
        self.assertEqual(robot.moves[0][1], gripper_keyboard_control.CLOSE_POSITION)
        self.assertEqual(robot.reads, 0)

    def test_position_feedback_is_polled_on_cadence_without_blocking_commands(self):
        controller_class = self.require_controller_class()
        robot = FakeRobot()
        lines = []
        controller = controller_class(robot, output=lines.append, clock=lambda: 0.0)

        controller.poll(now=0.0)
        controller.poll(now=0.1)
        controller.poll(now=0.2)
        controller.handle_key("O")

        self.assertEqual(robot.reads, 2)
        self.assertEqual(len(robot.moves), 1)
        self.assertEqual(robot.moves[0][1], gripper_keyboard_control.OPEN_POSITION)
        self.assertTrue(any("夹爪位置" in line for line in lines))

    def test_q_requests_exit_without_moving_gripper(self):
        controller_class = self.require_controller_class()
        robot = FakeRobot()
        controller = controller_class(robot, output=lambda _: None, clock=lambda: 0.0)

        keep_running = controller.handle_key("Q")

        self.assertFalse(keep_running)
        self.assertEqual(robot.moves, [])


if __name__ == "__main__":
    unittest.main()
