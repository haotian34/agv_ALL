# -*- coding: utf-8 -*-
"""RealSense + gripper keyboard test. No robot-arm or AGV motion.

Start camera first. In the image window:
    G = close gripper (0%)
    O = open gripper (60%)
    Q / ESC / close window = quit without another gripper command

Position feedback is sampled every 0.5 s and logged to JSONL.
Run from the vehicle script directory so the local fairino SDK is importable.
"""

import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs


IP = "192.168.192.160"
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_PATH = SCRIPT_DIR / ("camera_gripper_test_" + time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
WINDOW = "Camera + Gripper: G=close O=open Q=quit"

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)


def write_log(record):
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + "\n")


def main():
    print("安全要求：夹爪两指之间无手指、线缆和杂物；一人守实体急停。")
    print("本脚本不发送任何机械臂或底盘运动指令，也不会自动开合夹爪。")
    print("[1/3] 先启动 RealSense 彩色+深度...")

    pipeline = None
    robot = None
    last_command = None
    last_sample_at = 0.0
    last_printed_raw = None

    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)

        # Wait for one actual frame before allowing a gripper command.
        frames = pipeline.wait_for_frames(15000)
        if not frames.get_color_frame() or not frames.get_depth_frame():
            raise RuntimeError("相机未同时提供彩色和深度帧，禁止测试夹爪")
        print("      相机取流正常。")

        print("[2/3] 连接机械臂控制器，仅用于夹爪和状态读取...")
        from fairino import Robot
        robot = Robot.RPC(IP)
        print("      夹爪配置（只读）:", robot.GetGripperConfig())
        print("      初始夹爪位置（只读）:", robot.GetGripperCurPosition())

        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        print("[3/3] 请点击图像窗口：G=闭合，O=张开，Q/ESC=退出。")
        print("      按键后才动作；位置反馈每0.5秒写入日志。")

        while True:
            frames = pipeline.wait_for_frames(15000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data())
            cv2.putText(image, "G=close  O=open  Q=quit", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.imshow(WINDOW, image)

            now = time.monotonic()
            if now - last_sample_at >= 0.5:
                last_sample_at = now
                raw = robot.GetGripperCurPosition()
                elapsed_ms = None
                if last_command is not None:
                    elapsed_ms = round((now - last_command["monotonic"]) * 1000, 1)
                record = {
                    "type": "position",
                    "wall_time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "elapsed_since_command_ms": elapsed_ms,
                    "position_raw": repr(raw),
                }
                write_log(record)
                # Print every sample for 12 s after a command, otherwise only changes.
                if raw != last_printed_raw or (elapsed_ms is not None and elapsed_ms <= 12000):
                    print("夹爪位置:", raw, "；距上次指令(ms):", elapsed_ms)
                    last_printed_raw = raw

            key = cv2.waitKey(1) & 0xFF
            action = None
            position = None
            if key == ord("g"):
                action = "close"
                position = 0
            elif key == ord("o"):
                action = "open"
                position = 60
            elif key in (ord("q"), 27):
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break

            if action is not None:
                command_mono = time.monotonic()
                last_command = {"action": action, "monotonic": command_mono}
                write_log({
                    "type": "command_before",
                    "action": action,
                    "target_position": position,
                    "wall_time": time.strftime("%Y-%m-%d %H:%M:%S"),
                })
                print(f"发送{'闭合' if action == 'close' else '张开'}指令，目标位置={position} ...")
                error = robot.MoveGripper(1, position, 30, 20, 30000, 0, 0, 0, 0, 0)
                write_log({
                    "type": "command_after",
                    "action": action,
                    "target_position": position,
                    "return_error": error,
                    "elapsed_ms": round((time.monotonic() - command_mono) * 1000, 1),
                })
                print("MoveGripper 返回:", error, "；返回0不代表已经动作完成，请观察实物和位置反馈。")

    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()
        if robot is not None:
            try:
                robot.CloseRPC()
            except Exception:
                pass
        print("已退出；退出时未发送额外夹爪或机械臂动作。日志:", LOG_PATH)


if __name__ == "__main__":
    main()
