"""Independent, read-only static validation for eye-in-hand calibration.

The script does not command robot or AGV motion. Jog only from WebApp.
Each run creates a new JSONL file, separate from handeye_data.jsonl.
"""

import json
from pathlib import Path
import sys
import time

import numpy as np

from handeye_solve import tcp_pose_to_matrix


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)


def camera_point_to_base(tcp_pose, camera_point_m, handeye_transform):
    """Convert one camera point (m) to base coordinates (mm)."""
    base_from_tcp = tcp_pose_to_matrix(tcp_pose)
    camera_point_mm = np.asarray(camera_point_m, dtype=float) * 1000.0
    camera_point_h = np.append(camera_point_mm, 1.0)
    return (base_from_tcp @ handeye_transform @ camera_point_h)[:3]


def summarize_base_points(points):
    """Measure repeatability around the mean of the current validation run."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) == 0:
        raise ValueError("at least one point is required")
    mean = np.mean(points, axis=0)
    deviations = points - mean
    norms = np.linalg.norm(deviations, axis=1)
    return {
        "count": len(points),
        "mean_mm": mean,
        "axis_range_mm": np.ptp(points, axis=0),
        "rms_mm": float(np.sqrt(np.mean(norms ** 2))),
        "max_mm": float(np.max(norms)),
        "deviations_mm": deviations,
    }


def main():
    script_dir = Path(__file__).resolve().parent
    result_path = script_dir / "handeye_result.json"
    with result_path.open("r", encoding="utf-8") as handle:
        calibration = json.load(handle)
    handeye = np.asarray(calibration["transform"], dtype=float)
    previous_target = np.asarray(calibration["target_base_mm"], dtype=float)

    print("独立静态复测：脚本只读取 TCP 和相机，不驱动机械臂或底盘。")
    print("确认：toolcoord2；同一工件本次不再移动；WebApp 最低速点动；一人守实体急停。")
    input("确认上述条件后按回车启动相机（退出请按 q / ESC / 点X）...")

    import cv2
    import pyrealsense2 as rs
    from fairino import Robot

    robot = None
    pipeline = None
    samples = []
    output_path = script_dir / ("handeye_validation_" + time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
    window = "handeye static validation"
    latest = {"depth_frame": None}

    try:
        print("[1/3] 连接机械臂，只读当前 TCP ...")
        robot = Robot.RPC("192.168.192.160")
        err, tcp_pose = robot.GetActualTCPPose(0)
        if err != 0:
            raise RuntimeError(f"读取 TCP 失败 err={err}")
        print("      当前 TCP:", tcp_pose)

        print("[2/3] 启动 RealSense（深度对齐到彩色）...")
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        def record(u, v):
            depth_frame = latest["depth_frame"]
            if depth_frame is None:
                print("      尚无深度帧，请稍后重试")
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
                print("      该点深度无效，未记录；请调整姿态后重试")
                return
            z = float(np.median(depths))
            camera_point = [
                (u - intr.ppx) / intr.fx * z,
                (v - intr.ppy) / intr.fy * z,
                z,
            ]
            err, tcp_pose = robot.GetActualTCPPose(0)
            if err != 0:
                print(f"      读取 TCP 失败 err={err}，未记录")
                return
            base_point = camera_point_to_base(tcp_pose, camera_point, handeye)
            sample = {
                "tcp_pose": list(tcp_pose),
                "pixel": [int(u), int(v)],
                "depth_m": z,
                "cam_point_m": camera_point,
                "base_point_mm": base_point.tolist(),
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            samples.append(sample)
            with output_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(sample, ensure_ascii=False) + "\n")
            delta_previous = base_point - previous_target
            print(f"      第 {len(samples)} 组基座坐标 XYZ(mm): {np.round(base_point, 3).tolist()}")
            print(f"      与上次标定靶点差值 dXYZ(mm): {np.round(delta_previous, 3).tolist()}，合成 {np.linalg.norm(delta_previous):.3f} mm")
            if len(samples) >= 2:
                summary = summarize_base_points([s["base_point_mm"] for s in samples])
                print(f"      本次重复性 RMS={summary['rms_mm']:.3f} mm，最大={summary['max_mm']:.3f} mm")

        def on_mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                record(x, y)

        cv2.namedWindow(window)
        cv2.setMouseCallback(window, on_mouse)
        print("[3/3] 开始复测：WebApp 停稳后点同一小圆柱顶面中心，建议 5~8 个新姿态。")
        print("      u 撤销本组；q / ESC / 点X 退出。数据不会写入原标定文件。")

        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            latest["depth_frame"] = depth_frame
            image = np.asanyarray(color_frame.get_data())
            cv2.putText(image, f"validation samples: {len(samples)}  (u=undo, q=quit)",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.imshow(window, image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            if key == ord("u") and samples:
                samples.pop()
                with output_path.open("w", encoding="utf-8") as handle:
                    for sample in samples:
                        handle.write(json.dumps(sample, ensure_ascii=False) + "\n")
                print(f"      已撤销，剩 {len(samples)} 组")
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        if pipeline is not None:
            pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if robot is not None:
            robot.CloseRPC()

    print(f"复测结束，共 {len(samples)} 组。")
    if samples:
        summary = summarize_base_points([s["base_point_mm"] for s in samples])
        print("本次均值 XYZ(mm):", np.round(summary["mean_mm"], 3).tolist())
        print("XYZ 极差(mm):", np.round(summary["axis_range_mm"], 3).tolist())
        print(f"重复性 RMS={summary['rms_mm']:.3f} mm，最大={summary['max_mm']:.3f} mm")
        print("数据文件:", output_path)
        print("请把该文件发回。不要根据本结果直接实抓。")
    else:
        print("没有记录数据，未生成有效复测文件。")


if __name__ == "__main__":
    main()
