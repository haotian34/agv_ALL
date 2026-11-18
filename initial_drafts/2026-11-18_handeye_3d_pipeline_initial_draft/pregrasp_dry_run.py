"""One-move pre-grasp dry run with interactive confirmation.

Safety behavior: capture and verify first; refuse if vision/manual reference
mismatch exceeds 10 mm. Move only to 80 mm above the compensated grasp pose.
No descent, gripper action, second motion, or AGV motion is commanded.
"""

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

from handeye_validate import camera_point_to_base


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)


def compute_dry_run_poses(vision_target_base_mm, offset_record, height_mm=80.0):
    if not 10.0 <= float(height_mm) <= 100.0:
        raise ValueError("dry-run height must be between 10 and 100 mm")
    vision = np.asarray(vision_target_base_mm, dtype=float)
    offset = np.asarray(offset_record["offset_base_mm"], dtype=float)
    manual_reference = np.asarray(offset_record["manual_grasp_tcp_pose"], dtype=float)
    grasp_xyz = vision + offset
    difference = grasp_xyz - manual_reference[:3]
    difference_norm = float(np.linalg.norm(difference))
    below_manual_z = -float(difference[2])
    if below_manual_z > 1.0:
        raise ValueError(
            f"grasp Z below manual reference by {below_manual_z:.3f} mm > 1 mm; "
            "refuse motion because the gripper may strike the large cylinder"
        )
    if difference_norm > 10.0:
        raise ValueError(f"vision/manual reference mismatch {difference_norm:.3f} mm > 10 mm")
    orientation = list(offset_record["grasp_orientation_rpy_deg"])
    grasp_pose = [*grasp_xyz.tolist(), *orientation]
    pregrasp_pose = [* (grasp_xyz + np.array([0.0, 0.0, float(height_mm)])).tolist(), *orientation]
    return {
        "grasp_pose": grasp_pose,
        "pregrasp_pose": pregrasp_pose,
        "manual_difference_mm": difference.tolist(),
        "manual_difference_norm_mm": difference_norm,
    }


def main():
    parser = argparse.ArgumentParser(description="One-move pre-grasp dry run")
    parser.add_argument("--height", type=float, choices=[80.0, 50.0, 30.0, 10.0], default=80.0)
    args = parser.parse_args()
    height_mm = args.height

    script_dir = Path(__file__).resolve().parent
    with (script_dir / "handeye_result.json").open("r", encoding="utf-8") as handle:
        handeye = np.asarray(json.load(handle)["transform"], dtype=float)
    with (script_dir / "grasp_offset.json").open("r", encoding="utf-8") as handle:
        offset_record = json.load(handle)

    print(f"预抓取空跑：只运动到目标上方 {height_mm:g} mm，不闭合夹爪、不跑车。")
    print("先低速手动回到能看清顶部工件的拍照姿态；工件保持不动，toolcoord2。")
    print("整个过程中清场、一人守实体急停；WebApp 不运行任何遗留示教程序。")
    input("确认后按回车开始视觉取点...")

    import cv2
    import pyrealsense2 as rs
    from fairino import Robot

    robot = Robot.RPC("192.168.192.160")
    pipeline = None
    captured = None
    latest = {"depth_frame": None}
    window = "pregrasp dry run - click target"
    try:
        err, current_tcp = robot.GetActualTCPPose(0)
        if err != 0:
            raise RuntimeError(f"读取 TCP 失败 err={err}")
        print("当前 TCP:", current_tcp)
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        def record(u, v):
            nonlocal captured
            depth_frame = latest["depth_frame"]
            if depth_frame is None:
                return
            depths = []
            for du in range(-2, 3):
                for dv in range(-2, 3):
                    uu, vv = u + du, v + dv
                    if 0 <= uu < 640 and 0 <= vv < 480:
                        depth = depth_frame.get_distance(uu, vv)
                        if depth > 0:
                            depths.append(depth)
            if not depths:
                print("该点深度无效，未记录")
                return
            z = float(np.median(depths))
            camera_point = [
                (u - intr.ppx) / intr.fx * z,
                (v - intr.ppy) / intr.fy * z,
                z,
            ]
            err, tcp_pose = robot.GetActualTCPPose(0)
            if err != 0:
                print(f"读取 TCP 失败 err={err}")
                return
            base_point = camera_point_to_base(tcp_pose, camera_point, handeye)
            captured = {
                "tcp_pose": list(tcp_pose),
                "pixel": [int(u), int(v)],
                "depth_m": z,
                "cam_point_m": camera_point,
                "base_point_mm": base_point.tolist(),
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            print("视觉靶点 XYZ(mm):", np.round(base_point, 3).tolist())
            print("图像窗口回车确认；u 重选；q/ESC 放弃。")

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                record(x, y)

        cv2.namedWindow(window)
        cv2.setMouseCallback(window, on_mouse)
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            latest["depth_frame"] = depth_frame
            image = np.asanyarray(color_frame.get_data())
            status = "Enter=confirm, u=redo" if captured else "click cylinder-top center"
            cv2.putText(image, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.imshow(window, image)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                captured = None
                break
            if key == ord("u"):
                captured = None
            if key == 13 and captured is not None:
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                captured = None
                break
        pipeline.stop()
        pipeline = None
        cv2.destroyAllWindows()
        if captured is None:
            print("未确认视觉点，已取消，未运动。")
            return

        try:
            computed = compute_dry_run_poses(captured["base_point_mm"], offset_record, height_mm=height_mm)
        except ValueError as exc:
            print(f"安全核对失败：{exc}")
            print("已取消：未运动、未生成可用于实抓的空跑日志。请重新取点或重新手动演示，禁止强行实抓。")
            return
        print("\n视觉与手动标定交叉核对:")
        print("  差值 dXYZ(mm):", np.round(computed["manual_difference_mm"], 3).tolist())
        print(f"  合成差: {computed['manual_difference_norm_mm']:.3f} mm（超过 10 mm 会自动拒绝）")
        print(f"  高度差 dZ: {computed['manual_difference_mm'][2]:.3f} mm；低于手动演示超过 1 mm 会自动拒绝")
        print("计算抓取位姿（本次不下降到这里）:", [round(v, 3) for v in computed["grasp_pose"]])
        print("唯一目标：预抓取位姿:", [round(v, 3) for v in computed["pregrasp_pose"]])
        print("\n运动前最后确认：")
        print("1) WebApp 自动模式、已使能、无报警、速度最低；不要按示教程序开始；")
        print("2) 当前到预抓取点的直线路径无碰撞；夹爪保持张开；")
        print("3) 一人已守实体急停；只执行一次低速 MoveL，工具号 2、工件号 0。")
        answer = input("确认执行唯一一次运动请输入 MOVE；其他输入均取消：").strip()
        if answer != "MOVE":
            print("已取消，未运动。")
            return

        speed_error = robot.SetSpeed(5)
        print("SetSpeed(5) 返回:", speed_error)
        if speed_error != 0:
            print("速度设置失败，未发运动指令。")
            return
        print("开始唯一一次预抓取运动；异常请立即停止或按实体急停。")
        move_error = robot.MoveL(computed["pregrasp_pose"], 2, 0, vel=5.0, blendR=-1.0)
        print("MoveL 原始返回:", move_error)
        motion_done = False
        if move_error == 0:
            deadline = time.time() + 120.0
            while time.time() < deadline:
                done_error, motion_done = robot.GetRobotMotionDone()
                if done_error == 0 and motion_done == 1:
                    break
                time.sleep(0.2)
            if not motion_done:
                print("等待到位超时，调用 StopMotion；请检查机械臂和 WebApp。")
                print("StopMotion 原始返回:", robot.StopMotion())
        err, actual_tcp = robot.GetActualTCPPose(0)
        print("运动后 TCP 读取 err=", err, "实际值=", actual_tcp)
        log = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "vision_target": captured,
            **computed,
            "height_mm": height_mm,
            "speed_percent": 5,
            "tool": 2,
            "user": 0,
            "move_error": move_error,
            "motion_done": motion_done,
            "actual_tcp_read_error": err,
            "actual_tcp_pose": list(actual_tcp) if err == 0 else None,
        }
        log_path = script_dir / (f"pregrasp_dry_run_{height_mm:g}mm_" + time.strftime("%Y%m%d_%H%M%S") + ".json")
        log_path.write_text(json.dumps(log, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("日志:", log_path)
        print("脚本结束：不会下降、不会闭合夹爪、不会自动撤退。请把日志发回。")
    except KeyboardInterrupt:
        print("检测到 Ctrl+C，调用 StopMotion；请同时确认现场停止状态。")
        print("StopMotion 原始返回:", robot.StopMotion())
    finally:
        if pipeline is not None:
            pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        robot.CloseRPC()


if __name__ == "__main__":
    main()



