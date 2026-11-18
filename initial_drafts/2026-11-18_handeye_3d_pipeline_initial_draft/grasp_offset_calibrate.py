"""Interactive manual grasp-offset calibration; commands no equipment motion.

Step 1: click the cylinder-top target in RealSense color/depth frames.
Step 2: the user manually drags/jogs the arm to a perfect open-gripper pose.
Step 3: read the actual TCP and save actual_TCP - visual_target as the offset.
"""

import json
from pathlib import Path
import sys
import time

import numpy as np

from handeye_validate import camera_point_to_base


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)


def compute_grasp_offset(vision_target_base_mm, manual_grasp_tcp_pose):
    if len(manual_grasp_tcp_pose) != 6:
        raise ValueError("manual_grasp_tcp_pose must contain six values")
    vision = np.asarray(vision_target_base_mm, dtype=float)
    actual = np.asarray(manual_grasp_tcp_pose, dtype=float)
    return {
        "offset_base_mm": (actual[:3] - vision).tolist(),
        "grasp_orientation_rpy_deg": actual[3:].tolist(),
    }


def main():
    script_dir = Path(__file__).resolve().parent
    output_path = script_dir / "grasp_offset.json"
    if output_path.exists():
        print(f"已存在 {output_path}，为防止误覆盖，本次退出。请先备份旧文件。")
        return
    with (script_dir / "handeye_result.json").open("r", encoding="utf-8") as handle:
        handeye = np.asarray(json.load(handle)["transform"], dtype=float)

    print("抓取偏移标定：脚本不驱动机械臂、底盘或夹爪。")
    print("仅对小车顶部同一工件操作；toolcoord2；工件固定；底盘停车。")
    print("拖动/点动时清场、低速、一人守实体急停；先不对中提升实抓。")
    input("确认后按回车进入第 1 步视觉取点...")

    import cv2
    import pyrealsense2 as rs
    from fairino import Robot

    robot = Robot.RPC("192.168.192.160")
    pipeline = None
    captured = None
    latest = {"depth_frame": None}
    window = "grasp offset - click target"

    try:
        err, tcp_pose = robot.GetActualTCPPose(0)
        if err != 0:
            raise RuntimeError(f"读取 TCP 失败 err={err}")
        print("当前 TCP:", tcp_pose)
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
                print("尚无深度帧，请稍后重试")
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
            err, current_tcp = robot.GetActualTCPPose(0)
            if err != 0:
                print(f"读取 TCP 失败 err={err}，未记录")
                return
            base_point = camera_point_to_base(current_tcp, camera_point, handeye)
            captured = {
                "tcp_pose": list(current_tcp),
                "pixel": [int(u), int(v)],
                "depth_m": z,
                "cam_point_m": camera_point,
                "base_point_mm": base_point.tolist(),
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            print("视觉靶点基座 XYZ(mm):", np.round(base_point, 3).tolist())
            print("确认此点：图像窗口按回车；重新点选：按 u；放弃：q/ESC。")

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                record(x, y)

        cv2.namedWindow(window)
        cv2.setMouseCallback(window, on_mouse)
        print("第 1 步：在清晰姿态点小圆柱顶面中心。只需一个点。")
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            latest["depth_frame"] = depth_frame
            image = np.asanyarray(color_frame.get_data())
            status = "captured: Enter=confirm, u=redo" if captured else "click cylinder-top center"
            cv2.putText(image, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 2)
            cv2.imshow(window, image)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                captured = None
                break
            if key == ord("u"):
                captured = None
                print("已清除，请重新点选")
            if key == 13 and captured is not None:
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                captured = None
                break
    finally:
        if pipeline is not None:
            pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    try:
        if captured is None:
            print("未确认视觉靶点，本次不生成补偿。")
            return
        print("\n第 2 步：由你手动完成完美对位，脚本不发运动指令。")
        print("1) 夹爪保持安全张开；2) 按住末端拖动按钮慢拖到附近；")
        print("3) WebApp 微调；4) 竖直向下，两指弧口合拢圆心对准小圆柱；")
        print("5) 不移动工件，不提升；完全停稳后再记录。")
        answer = input("完美对位并停稳后按回车读取 TCP；输入 q 放弃：").strip().lower()
        if answer == "q":
            print("已放弃，未生成补偿。")
            return
        err, manual_tcp = robot.GetActualTCPPose(0)
        if err != 0:
            raise RuntimeError(f"读取手动抓取 TCP 失败 err={err}")
        computed = compute_grasp_offset(captured["base_point_mm"], manual_tcp)
        print("\n第 3 步：请核对")
        print("视觉靶点 XYZ(mm):", np.round(captured["base_point_mm"], 3).tolist())
        print("手动完美抓取 TCP:", [round(v, 6) for v in manual_tcp])
        print("固定补偿 dXYZ(mm):", np.round(computed["offset_base_mm"], 3).tolist())
        print("固定抓取 RPY(deg):", np.round(computed["grasp_orientation_rpy_deg"], 6).tolist())
        print("补偿定义：实际手动抓取 TCP 位置 - 视觉靶点位置。")
        answer = input("仅当对位准确时输入 YES 保存；其他输入均放弃：").strip()
        if answer != "YES":
            print("未保存。")
            return
        record = {
            "created_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "tool_coord": "toolcoord2",
            "scope": "same workpiece geometry and fixed vertical grasp orientation",
            "vision_target": captured,
            "manual_grasp_tcp_pose": list(manual_tcp),
            **computed,
            "warning": "Validate with a 50-100 mm pre-grasp dry run before gripping or lifting.",
        }
        output_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("已保存:", output_path)
        print("下一步只做 50~100 mm 预抓取空跑，不要直接实抓。")
    finally:
        robot.CloseRPC()


if __name__ == "__main__":
    main()
