# -*- coding: utf-8 -*-
"""Standalone Fairino gripper real-time keyboard control.

This script does NOT move the robot arm, camera or AGV.
Keys take effect immediately without pressing Enter:
    O = open gripper (60%)
    G = close gripper (0%)
    S = refresh position immediately
    Q = quit

Position feedback refreshes every 0.2 s in the same loop. No fixed multi-second
waiting period is used.
"""

import sys
import time


IP = "192.168.192.160"
OPEN_POSITION = 60
CLOSE_POSITION = 0
VELOCITY = 30
FORCE = 20
POLL_INTERVAL_SECONDS = 0.2
KEY_SCAN_INTERVAL_SECONDS = 0.02

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)


class LiveGripperController:
    """Non-blocking gripper command handling and cadenced position reporting."""

    def __init__(self, robot, output=print, clock=time.monotonic):
        self.robot = robot
        self.output = output
        self.clock = clock
        self.next_poll_at = self.clock()
        self.last_command_at = None
        self.last_command_name = None

    def _movement_command(self, name, target):
        self.output(f"\n发送{name}指令，目标位置={target} ...")
        started = self.clock()
        self.last_command_at = started
        self.last_command_name = name
        error = self.robot.MoveGripper(
            1, target, VELOCITY, FORCE, 30000, 0, 0, 0, 0, 0
        )
        elapsed_ms = (self.clock() - started) * 1000.0
        self.output(
            f"MoveGripper 返回: {error}，调用耗时={elapsed_ms:.1f} ms；"
            "返回0只表示指令被接受。"
        )
        self.next_poll_at = self.clock()

    def handle_key(self, key):
        """Handle one key immediately. Return False only when exit was requested."""
        command = str(key).upper()
        if command == "G":
            self._movement_command("闭合", CLOSE_POSITION)
        elif command == "O":
            self._movement_command("张开", OPEN_POSITION)
        elif command == "S":
            self.poll(force=True)
        elif command == "Q":
            return False
        elif command.strip():
            self.output("无效命令，请输入 O、G、S 或 Q。")
        return True

    def poll(self, now=None, force=False):
        """Read position when due; never sleep or wait for motion completion."""
        current = self.clock() if now is None else float(now)
        if not force and current < self.next_poll_at:
            return False
        self.next_poll_at = current + POLL_INTERVAL_SECONDS
        try:
            raw = self.robot.GetGripperCurPosition()
        except Exception as exc:
            self.output(f"\r夹爪位置读取失败: {exc}")
            return False

        since_command = ""
        if self.last_command_at is not None:
            elapsed_ms = (current - self.last_command_at) * 1000.0
            since_command = (
                f" | 最近指令={self.last_command_name}"
                f" | 指令后={elapsed_ms:.0f} ms"
            )
        # Carriage return keeps a high-rate live display without scrolling.
        self.output(f"\r夹爪位置反馈: {raw!r}{since_command}          ")
        return True


def run_keyboard_loop(controller):
    """Run a Windows real-time keyboard loop without Enter or blocking prompts."""
    try:
        import msvcrt
    except ImportError as exc:
        raise RuntimeError("实时键盘控制需要 Windows msvcrt") from exc

    print("\n实时控制已启动：O=张开 G=闭合 S=刷新位置 Q=退出（无需回车）")
    try:
        while True:
            controller.poll()
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key in ("\x00", "\xe0"):
                    # Ignore the second byte of Windows special-key sequences.
                    if msvcrt.kbhit():
                        msvcrt.getwch()
                    continue
                if not controller.handle_key(key):
                    print("\n已退出；不会自动张爪或闭爪。")
                    break
            time.sleep(KEY_SCAN_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，退出；不会自动张爪或闭爪。")


def main():
    print("安全要求：夹爪两指之间不得有手指、线缆或杂物；一人守实体急停。")
    print("本脚本只控制夹爪，不发送任何机械臂或底盘运动指令。")

    try:
        from fairino import Robot
    except ImportError:
        print("导入 fairino 失败。请先执行：")
        print(r"cd /d C:\Users\Admin\Desktop\现场调试脚本")
        print(r"D:\anaconda3\envs\agv\python.exe gripper_keyboard_control.py")
        return 1

    robot = None
    try:
        print(f"连接机械臂控制器 {IP}（仅夹爪控制）...")
        robot = Robot.RPC(IP)
        print("夹爪配置（只读）:", robot.GetGripperConfig())
        controller = LiveGripperController(robot)
        controller.poll(force=True)
        run_keyboard_loop(controller)
    finally:
        if robot is not None:
            try:
                robot.CloseRPC()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
