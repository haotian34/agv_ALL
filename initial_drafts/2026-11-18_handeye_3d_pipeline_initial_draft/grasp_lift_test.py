"""Guarded real-grasp test using an accepted 10 mm dry-run log.

Safety gates:
- The target and actual descent TCP must not be more than 1 mm below the
  manually demonstrated grasp Z.
- Gripper command success, MotionDone, and a fixed delay are never treated as
  proof of closure. Actual position feedback must be stable in the closed
  range, followed by a separate human HOLDING confirmation before any lift.
- No release or AGV motion is commanded. Any failed gate forbids lifting.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np


MAX_TOTAL_REFERENCE_ERROR_MM = 10.0
MAX_BELOW_MANUAL_Z_MM = 1.0
MIN_OPEN_POSITION = 55
MAX_CLOSED_POSITION = 45
STABLE_CLOSED_READINGS = 3


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)


def build_grasp_lift_poses(dry_run_log, lift_mm=20.0):
    grasp = list(dry_run_log["grasp_pose"])
    pregrasp = list(dry_run_log["pregrasp_pose"])
    if abs((pregrasp[2] - grasp[2]) - 10.0) > 0.5:
        raise ValueError("input log is not a 10 mm pre-grasp dry run")
    lift = list(grasp)
    lift[2] += float(lift_mm)
    return {"grasp_pose": grasp, "lift_pose": lift}


def validate_grasp_reference(grasp_pose, offset_record):
    """Reject a low grasp even when its total 3D error is below 10 mm."""
    grasp = np.asarray(grasp_pose, dtype=float)
    manual = np.asarray(offset_record["manual_grasp_tcp_pose"], dtype=float)
    difference = grasp[:3] - manual[:3]
    difference_norm = float(np.linalg.norm(difference))
    below_manual_z = -float(difference[2])
    if below_manual_z > MAX_BELOW_MANUAL_Z_MM:
        raise ValueError(
            f"grasp Z below manual reference by {below_manual_z:.3f} mm > "
            f"{MAX_BELOW_MANUAL_Z_MM:g} mm; lifting is forbidden"
        )
    if difference_norm > MAX_TOTAL_REFERENCE_ERROR_MM:
        raise ValueError(
            f"vision/manual reference mismatch {difference_norm:.3f} mm > "
            f"{MAX_TOTAL_REFERENCE_ERROR_MM:g} mm"
        )
    return {
        "manual_difference_mm": difference.tolist(),
        "manual_difference_norm_mm": difference_norm,
        "below_manual_z_mm": below_manual_z,
    }


def starting_pose_is_expected(current_pose, expected_pregrasp):
    current = np.asarray(current_pose, dtype=float)
    expected = np.asarray(expected_pregrasp, dtype=float)
    return bool(
        np.linalg.norm(current[:3] - expected[:3]) <= 5.0
        and np.max(np.abs(current[3:] - expected[3:])) <= 1.0
    )


def read_gripper_position(robot):
    """Read (error, fault, position); any error or fault is a hard stop."""
    raw = robot.GetGripperCurPosition()
    try:
        error, fault, position = raw
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"无法解析夹爪位置反馈: {raw!r}") from exc
    if error != 0 or fault != 0:
        raise RuntimeError(f"夹爪位置反馈异常: {raw!r}")
    return int(position)


def confirm_gripper_holding(
    robot,
    input_fn=input,
    max_attempts=20,
    poll_interval_s=0.5,
    sleep_fn=time.sleep,
):
    """Return (human_and_feedback_confirmed, position_samples); never move the arm."""
    samples = []
    stable_closed = 0
    for attempt in range(max_attempts):
        position = read_gripper_position(robot)
        samples.append(position)
        print(f"夹爪实际位置反馈[{attempt + 1}/{max_attempts}]: {position}")
        if position <= MAX_CLOSED_POSITION:
            stable_closed += 1
        else:
            stable_closed = 0
        if stable_closed >= STABLE_CLOSED_READINGS:
            print("位置反馈已连续稳定进入闭合范围，但这仍不等于已经夹住工件。")
            answer = input_fn(
                "肉眼确认工件确实在两指之间、已经夹稳且灯态正常，输入 HOLDING 才提升；其他输入取消："
            ).strip()
            return answer == "HOLDING", samples
        if attempt + 1 < max_attempts:
            sleep_fn(poll_interval_s)
    print(
        f"夹爪在 {max_attempts} 次读取后仍未稳定进入闭合范围"
        f"（要求位置 <= {MAX_CLOSED_POSITION}），禁止提升。"
    )
    return False, samples


def main():
    parser = argparse.ArgumentParser(description="Guarded grasp and short lift")
    parser.add_argument("--speed", type=float, choices=[10.0, 20.0], default=10.0)
    parser.add_argument("--lift", type=float, choices=[10.0, 20.0, 30.0], default=20.0)
    args = parser.parse_args()
    script_dir = Path(__file__).resolve().parent
    candidates = sorted(script_dir.glob("pregrasp_dry_run_10mm_*.json"))
    if not candidates:
        raise FileNotFoundError("no pregrasp_dry_run_10mm_*.json found")
    source_path = candidates[-1]
    dry_run_log = json.loads(source_path.read_text(encoding="utf-8"))
    offset_record = json.loads((script_dir / "grasp_offset.json").read_text(encoding="utf-8"))
    poses = build_grasp_lift_poses(dry_run_log, lift_mm=args.lift)
    try:
        reference_check = validate_grasp_reference(poses["grasp_pose"], offset_record)
    except ValueError as exc:
        print(f"安全核对失败：{exc}")
        print("该空跑日志禁止用于实抓；不会连接机械臂。请重新进行视觉取点和空跑。")
        return

    print("受保护实抓：独立高度核对 -> 下降 -> 实际闭合反馈 -> 人工确认夹稳 -> 小幅提升。")
    print("使用 10 mm 空跑日志:", source_path)
    print("抓取位姿:", [round(v, 3) for v in poses["grasp_pose"]])
    print("提升位姿:", [round(v, 3) for v in poses["lift_pose"]])
    print(
        f"高度保护：不得低于手动演示 Z 超过 {MAX_BELOW_MANUAL_Z_MM:g} mm；"
        f"本次 dZ={reference_check['manual_difference_mm'][2]:.3f} mm。"
    )
    print("要求：toolcoord2、夹爪实际张开、工件可安全提升、清场、一人守实体急停。")
    print("手动或自动模式均可，但必须已使能、无报警；不要运行 WebApp 示教程序。")
    input("确认当前仍在刚才的 10 mm 位置，按回车连接并核对起点...")

    from fairino import Robot
    robot = Robot.RPC("192.168.192.160")
    log_path = script_dir / ("grasp_lift_test_" + time.strftime("%Y%m%d_%H%M%S") + ".json")
    log = {
        "source_dry_run": str(source_path),
        **poses,
        "reference_check": reference_check,
        "safety_limits": {
            "max_total_reference_error_mm": MAX_TOTAL_REFERENCE_ERROR_MM,
            "max_below_manual_z_mm": MAX_BELOW_MANUAL_Z_MM,
            "min_open_position": MIN_OPEN_POSITION,
            "max_closed_position": MAX_CLOSED_POSITION,
            "stable_closed_readings": STABLE_CLOSED_READINGS,
        },
        "speed_percent": args.speed,
        "lift_mm": args.lift,
        "stages": {},
    }

    def save_log():
        log_path.write_text(json.dumps(log, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    def read_tcp(stage):
        err, pose = robot.GetActualTCPPose(0)
        print(stage, "TCP err=", err, "值=", pose)
        log["stages"][stage] = {"tcp_error": err, "tcp_pose": list(pose) if err == 0 else None}
        save_log()
        return err, pose

    def move_and_wait(stage, target):
        error = robot.MoveL(target, 2, 0, vel=args.speed, blendR=-1.0)
        print(stage, "MoveL 原始返回:", error)
        log["stages"][stage] = {"target": target, "move_error": error, "motion_done": False}
        save_log()
        if error != 0:
            return False
        deadline = time.time() + 120.0
        done = False
        while time.time() < deadline:
            done_error, state = robot.GetRobotMotionDone()
            if done_error == 0 and state == 1:
                done = True
                break
            time.sleep(0.2)
        log["stages"][stage]["motion_done"] = done
        save_log()
        if not done:
            print(stage, "到位超时，调用 StopMotion:", robot.StopMotion())
            return False
        read_tcp(stage + "_actual")
        return True

    try:
        err, current = robot.GetActualTCPPose(0)
        if err != 0 or not starting_pose_is_expected(current, dry_run_log["pregrasp_pose"]):
            print("起点与已确认的 10 mm 位姿不一致，禁止下降。当前:", current)
            return
        print("起点核对通过。")
        print("夹爪配置（只读）:", robot.GetGripperConfig())
        open_position = read_gripper_position(robot)
        log["stages"]["open_check"] = {"position": open_position}
        save_log()
        print("下降前夹爪实际位置:", open_position)
        if open_position < MIN_OPEN_POSITION:
            print(f"夹爪未实际张开（要求位置 >= {MIN_OPEN_POSITION}），禁止下降。")
            return

        answer = input("输入 DOWN：以设定速度下降 10 mm；其他输入取消：").strip()
        if answer != "DOWN":
            print("已取消，未运动。")
            return
        speed_error = robot.SetSpeed(args.speed)
        print("SetSpeed 返回:", speed_error)
        log["stages"]["set_speed"] = {"error": speed_error}
        save_log()
        if speed_error != 0:
            print("速度设置失败，停止后续步骤。")
            return
        if not move_and_wait("descend", poses["grasp_pose"]):
            print("下降失败或未完成，停止后续步骤。")
            return

        err, descended_tcp = robot.GetActualTCPPose(0)
        if err != 0:
            print("无法读取下降后的实际 TCP，禁止闭合和提升。")
            return
        try:
            actual_check = validate_grasp_reference(descended_tcp, offset_record)
        except ValueError as exc:
            print(f"下降后实际高度安全核对失败：{exc}")
            print("禁止闭合和提升；不要继续下压。")
            return
        log["stages"]["descend_reference_check"] = actual_check
        save_log()

        print("\n已核对下降位置。若夹爪碰到大圆柱、位置不合适或现场不安全，请取消。")
        answer = input("输入 GRIP：只闭合夹爪；其他输入取消且不提升：").strip()
        if answer != "GRIP":
            print("已取消，未提升。")
            return
        grip_error = robot.MoveGripper(1, 0, 30, 20, 30000, 0, 0, 0, 0, 0)
        print("MoveGripper 原始返回:", grip_error)
        log["stages"]["grip"] = {"move_gripper_error": grip_error}
        save_log()
        if grip_error != 0:
            print("夹爪指令失败，禁止提升。")
            return

        holding_confirmed, samples = confirm_gripper_holding(robot)
        log["stages"]["grip"]["position_samples"] = samples
        log["stages"]["grip"]["holding_confirmed"] = holding_confirmed
        save_log()
        if not holding_confirmed:
            print("未同时满足实际闭合反馈和人工夹稳确认，禁止提升。不要移动机械臂或底盘。")
            return

        if not move_and_wait("lift", poses["lift_pose"]):
            print("提升失败或未完成，保持现场并检查，不要跑车。")
            return
        print("提升完成，脚本结束。不会自动下降或松开；不要导航。日志:", log_path)
        print("请汇报：工件是否夹稳、灯态、是否有滑移，并发送日志。")
    except KeyboardInterrupt:
        print("Ctrl+C，调用 StopMotion:", robot.StopMotion())
    finally:
        save_log()
        robot.CloseRPC()


if __name__ == "__main__":
    main()
