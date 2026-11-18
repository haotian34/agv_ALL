"""Read-only interactive nine-point acquisition on a static A4 photograph.

The script never commands equipment motion. Operators jog manually through the
WebApp while this script only reads TCP poses and appends session events.
"""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile

import numpy as np

from plane_math import (
    apply_homography,
    base_points_to_photo_local,
    solve_plane_calibration,
)
from plane_report import write_report
from plane_session import (
    SUPPORTED_PROFILE_IDS,
    append_session_event,
    create_session,
    load_session,
)


SCRIPT_DIR = Path(__file__).resolve().parent
IMAGE_SIZE = [640, 480]
ROBOT_IP = "192.168.192.160"
RPY_TOLERANCE_DEG = 0.5
SAFETY_PHRASE = "确认只读手动示教"
WINDOW_NAME = "A4 static calibration - read only"


class AcquisitionAborted(RuntimeError):
    """The operator stopped acquisition; the append-only session remains."""


def _require_safety_confirmation(input_fn=input) -> None:
    """Require explicit confirmation of all field-safety prerequisites."""
    print("开始前请逐项确认：")
    print("1. 当前激活工具坐标为 toolcoord2。")
    print("2. A4 标定目标已固定，采集期间不会移动。")
    print("3. 现场有专人守护物理急停。")
    print("4. 所有移动均由人员通过 WebApp 手动完成；本脚本只读TCP。")
    response = input_fn(f"请输入 {SAFETY_PHRASE} 以继续：").strip()
    if response != SAFETY_PHRASE:
        raise ValueError("safety confirmation failed; 安全确认失败，未开始采集")


def _connect_robot():
    """Lazy Fairino connection used only for TCP reads."""
    from fairino import Robot

    return Robot.RPC(ROBOT_IP)


def _read_tcp(robot) -> list[float]:
    error_code, tcp_pose = robot.GetActualTCPPose(0)
    if error_code != 0:
        raise RuntimeError(f"读取Fairino TCP失败，错误码：{error_code}")
    try:
        values = [float(value) for value in tcp_pose]
    except (TypeError, ValueError):
        raise ValueError("Fairino TCP response must contain six finite numbers") from None
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("Fairino TCP response must contain six finite numbers")
    return values


def _capture_static_image(image_path: Path) -> np.ndarray:
    """Capture one 640x480 BGR color frame and save it as a new PNG."""
    import cv2
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("RealSense未返回彩色图像")
        image = np.asanyarray(color_frame.get_data())
    finally:
        pipeline.stop()

    if image.shape != (480, 640, 3):
        raise RuntimeError(f"彩色图像尺寸错误：{image.shape}，预期(480, 640, 3)")
    success, encoded = cv2.imencode(".png", image)
    if not success:
        raise RuntimeError("PNG编码失败")
    image_path = Path(image_path)
    image_path.parent.mkdir(parents=True, exist_ok=True)
    with image_path.open("xb") as handle:
        handle.write(encoded.tobytes())
        handle.flush()
        os.fsync(handle.fileno())
    return image


def _load_saved_image(image_path: Path) -> np.ndarray:
    """Load the session's immutable reference PNG without opening a camera."""
    import cv2

    encoded = np.fromfile(str(image_path), dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None or image.shape != (480, 640, 3):
        raise ValueError(f"无法读取640x480彩色参考图：{image_path}")
    return image


def _collect_pixels(
    session_path: Path,
    profile_id: str,
    pixels: list[dict],
    image: np.ndarray,
) -> list[dict]:
    """Click ordered pixels on the saved image; persist clicks and undo at once."""
    import cv2

    active = list(pixels)
    if len(active) == 9:
        return active

    def redraw() -> None:
        display = image.copy()
        for item in active:
            index = item["index"]
            u, v = (int(round(value)) for value in item["pixel"])
            cv2.circle(display, (u, v), 5, (0, 0, 255), 1)
            cv2.putText(
                display, str(index), (u + 7, v - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
            )
        cv2.imshow(WINDOW_NAME, display)

    def on_mouse(event, x, y, _flags, _parameter) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or len(active) >= 9:
            return
        index = len(active)
        stored_event = append_session_event(session_path, profile_id, {
            "type": "pixel", "index": index, "pixel": [float(x), float(y)],
        })
        active.append(stored_event)
        print(f"像素点 {index}: ({x}, {y})")
        redraw()

    print("阶段A：在静态图上按0–8顺序点击；u撤销，九点齐全后Enter继续，q中止。")
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    redraw()
    try:
        while True:
            key = cv2.waitKey(50) & 0xFF
            if key in (ord("q"), 27):
                raise AcquisitionAborted("操作员中止像素采集；会话已保留")
            if key == ord("u"):
                if active:
                    append_session_event(session_path, profile_id, {
                        "type": "undo", "target": "pixel",
                    })
                    removed = active.pop()
                    print(f"撤销像素点 {removed['index']}")
                    redraw()
            elif key in (10, 13) and len(active) == 9:
                return active
    finally:
        cv2.destroyWindow(WINDOW_NAME)


def _collect_taught_points(
    session_path: Path,
    profile_id: str,
    taught_points: list[dict],
    image: np.ndarray,
    robot,
    input_fn=input,
) -> list[dict]:
    """Read one TCP per confirmation in order, with durable undo and resume."""
    import cv2

    active = list(taught_points)
    print("阶段B：保持参考图可见，仅通过WebApp手动移动。")
    print("按Enter读取当前编号TCP；u撤销；九点齐全后Enter完成；q中止。")
    try:
        while True:
            index = len(active)
            cv2.imshow(WINDOW_NAME, image)
            cv2.waitKey(1)
            if index == 9:
                prompt = "九点已齐全：Enter完成，u撤销点8，q中止 > "
            else:
                prompt = f"示教点 {index}（0–8顺序）> "
            command = input_fn(prompt).strip().lower()
            if command == "q":
                raise AcquisitionAborted("操作员中止示教采集；会话已保留")
            if command == "u":
                if active:
                    append_session_event(session_path, profile_id, {
                        "type": "undo", "target": "taught_point",
                    })
                    removed = active.pop()
                    print(f"撤销示教点 {removed['index']}")
                continue
            if command:
                print("仅接受Enter、u或q。")
                continue
            if index == 9:
                return active

            tcp_pose = _read_tcp(robot)
            stored_event = append_session_event(session_path, profile_id, {
                "type": "taught_point", "index": index, "tcp_pose": tcp_pose,
            })
            active.append(stored_event)
            print(f"示教点 {index}: {tcp_pose}")
    finally:
        cv2.destroyWindow(WINDOW_NAME)


def _point_residuals(
    homography: list[list[float]],
    pixels: np.ndarray,
    local_points: np.ndarray,
) -> list[dict]:
    predicted = apply_homography(np.asarray(homography, dtype=float), pixels)
    residual = predicted - local_points[:, :2]
    return [
        {
            "index": index,
            "pixel": pixels[index].tolist(),
            "actual_local_xy": local_points[index, :2].tolist(),
            "predicted_local_xy": predicted[index].tolist(),
            "residual_xy_mm": residual[index].tolist(),
            "planar_error_mm": float(np.linalg.norm(residual[index])),
        }
        for index in range(9)
    ]


def _new_result(
    session_path: Path,
    state: dict,
    solution: dict,
    pixels: np.ndarray,
    local_points: np.ndarray,
    center_tolerance_mm: float,
    outer_tolerance_mm: float,
) -> dict:
    created = datetime.now().astimezone()
    return {
        "schema_version": 1,
        "result_id": f"{state['profile_id']}-{created.strftime('%Y%m%dT%H%M%S%f%z')}",
        "created_at": created.isoformat(),
        "profile_id": state["profile_id"],
        "toolcoord": "toolcoord2",
        "image_size": list(IMAGE_SIZE),
        "reference_photo_pose": list(state["reference_photo_pose"]),
        "rpy_tolerance_deg": float(RPY_TOLERANCE_DEG),
        "center_tolerance_mm": float(center_tolerance_mm),
        "outer_tolerance_mm": float(outer_tolerance_mm),
        "homography": solution["homography"],
        "plane_coefficients": solution["plane_coefficients"],
        "outer_fit_metrics": solution["outer_fit_metrics"],
        "fit_metrics": solution["fit_metrics"],
        "center_validation": solution["center_validation"],
        "point_residuals": _point_residuals(
            solution["homography"], pixels, local_points
        ),
        "source_png": state["image_path"],
        "source_session": str(session_path),
        "motion_authorized": False,
    }


def _backup_path(result_path: Path) -> Path:
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%f%z")
    candidate = result_path.with_name(
        f"{result_path.stem}.backup-{stamp}{result_path.suffix}"
    )
    counter = 1
    while candidate.exists():
        candidate = result_path.with_name(
            f"{result_path.stem}.backup-{stamp}-{counter}{result_path.suffix}"
        )
        counter += 1
    return candidate


def write_versioned_result(result: dict, result_path: Path) -> Path:
    """Back up the same profile's current result, then atomically replace it."""
    result_path = Path(result_path)
    expected_name = f"{result['profile_id']}.json"
    if result_path.name != expected_name:
        raise ValueError(f"结果路径必须为{expected_name}，拒绝替换其他配置")
    result_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", newline="\n",
            dir=result_path.parent, prefix=f"{result_path.stem}.",
            suffix=".tmp", delete=False,
        ) as handle:
            temporary_path = Path(handle.name)
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())

        if result_path.exists():
            try:
                with result_path.open("r", encoding="utf-8") as handle:
                    existing = json.load(handle)
            except (OSError, json.JSONDecodeError, UnicodeDecodeError) as error:
                raise ValueError("现有结果不可读，拒绝未备份替换") from error
            if existing.get("profile_id") != result["profile_id"]:
                raise ValueError("现有结果属于其他配置，拒绝替换")
            shutil.copy2(result_path, _backup_path(result_path))
        os.replace(temporary_path, result_path)
        temporary_path = None
        return result_path
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def finalize_session(
    session_path: Path,
    center_tolerance_mm: float,
    outer_tolerance_mm: float,
    output_root: Path = SCRIPT_DIR,
) -> tuple[dict, Path, Path]:
    """Solve a complete session and write result/report only after all checks pass."""
    session_path = Path(session_path)
    probe = json.loads(session_path.read_text(encoding="utf-8").splitlines()[0])
    profile_id = probe["profile_id"]
    state = load_session(session_path, profile_id)
    if len(state["pixels"]) != 9 or len(state["taught_points"]) != 9:
        raise ValueError("session incomplete: 必须包含9个像素点和9个示教点")

    pixels = np.asarray([item["pixel"] for item in state["pixels"]], dtype=float)
    taught_base = np.asarray([
        item["tcp_pose"][:3] for item in state["taught_points"]
    ], dtype=float)
    local_points = base_points_to_photo_local(
        state["reference_photo_pose"], taught_base
    )
    solution = solve_plane_calibration(
        local_points,
        pixels,
        center_tolerance_mm=center_tolerance_mm,
        outer_tolerance_mm=outer_tolerance_mm,
    )
    result = _new_result(
        session_path,
        state,
        solution,
        pixels,
        local_points,
        center_tolerance_mm,
        outer_tolerance_mm,
    )
    result_path = Path(output_root) / "plane_results" / f"{profile_id}.json"
    write_versioned_result(result, result_path)
    report_path = write_report(result, result_path.with_suffix(".txt"))
    return result, result_path, report_path


def _new_session_paths(output_root: Path, profile_id: str) -> tuple[Path, Path]:
    session_directory = Path(output_root) / "plane_sessions"
    session_directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%f%z")
    stem = f"{profile_id}_{stamp}"
    session_path = session_directory / f"{stem}.jsonl"
    image_path = session_directory / f"{stem}.png"
    counter = 1
    while session_path.exists() or image_path.exists():
        session_path = session_directory / f"{stem}-{counter}.jsonl"
        image_path = session_directory / f"{stem}-{counter}.png"
        counter += 1
    return session_path, image_path


def run_acquisition(
    profile_id: str,
    center_tolerance_mm: float,
    outer_tolerance_mm: float,
    resume: Path | None = None,
    output_root: Path = SCRIPT_DIR,
) -> tuple[dict, Path, Path]:
    """Run or resume both phases, closing any Fairino connection in all cases."""
    if not isinstance(profile_id, str) or profile_id not in SUPPORTED_PROFILE_IDS:
        supported = ", ".join(sorted(SUPPORTED_PROFILE_IDS))
        raise ValueError(
            f"unsupported profile_id {profile_id!r}; supported profile IDs: {supported}"
        )
    robot = None
    try:
        if resume is not None:
            session_path = Path(resume)
            state = load_session(session_path, profile_id)
            if len(state["pixels"]) == 9 and len(state["taught_points"]) == 9:
                return finalize_session(
                    session_path,
                    center_tolerance_mm=center_tolerance_mm,
                    outer_tolerance_mm=outer_tolerance_mm,
                    output_root=output_root,
                )
            image = _load_saved_image(Path(state["image_path"]))
        else:
            session_path, image_path = _new_session_paths(output_root, profile_id)
            robot = _connect_robot()
            image = _capture_static_image(image_path)
            reference_pose = _read_tcp(robot)
            create_session(
                session_path, profile_id, reference_pose, image_path
            )
            state = load_session(session_path, profile_id)

        pixels = _collect_pixels(
            session_path, profile_id, state["pixels"], image
        )
        state["pixels"] = pixels
        if len(state["taught_points"]) < 9:
            if robot is None:
                robot = _connect_robot()
            taught_points = _collect_taught_points(
                session_path, profile_id, state["taught_points"], image, robot
            )
            state["taught_points"] = taught_points
        return finalize_session(
            session_path,
            center_tolerance_mm=center_tolerance_mm,
            outer_tolerance_mm=outer_tolerance_mm,
            output_root=output_root,
        )
    finally:
        if robot is not None:
            robot.CloseRPC()


def main(argv=None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(
        description="静态A4九点只读采集；所有移动仅由人员通过WebApp手动执行。"
    )
    parser.add_argument(
        "--profile-id", required=True, choices=sorted(SUPPORTED_PROFILE_IDS),
        help="工位配置ID",
    )
    parser.add_argument("--center-tolerance-mm", type=float, required=True)
    parser.add_argument("--outer-tolerance-mm", type=float, required=True)
    parser.add_argument("--resume", type=Path, help="保留并续采现有JSONL会话")
    arguments = parser.parse_args(argv)
    try:
        _require_safety_confirmation()
        result, result_path, report_path = run_acquisition(
            arguments.profile_id,
            arguments.center_tolerance_mm,
            arguments.outer_tolerance_mm,
            resume=arguments.resume,
        )
    except (AcquisitionAborted, ValueError, RuntimeError) as error:
        print(f"采集未生成新结果：{error}", file=sys.stderr)
        return 2
    print(f"结果：{result_path}")
    print(f"报告：{report_path}")
    print(
        "外圈拟合最大残差："
        f"{result['outer_fit_metrics']['max_planar_error_mm']:.6f} mm"
    )
    print(f"独立中心误差：{result['center_validation']['error_xy_mm']:.6f} mm")
    print("仅用于静态验证，禁止据此直接自动运动。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
