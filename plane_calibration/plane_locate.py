"""Static, read-only localization using a versioned plane calibration profile.

All translations and output coordinates are millimetres. Fairino RPY values are
in degrees. The plane Z value is diagnostic photo-local geometry, never a
motion target.
"""

import argparse
import json
import math
from numbers import Real
from pathlib import Path
import sys
from typing import Sequence

import numpy as np

from plane_math import (
    apply_homography,
    photo_local_to_base,
    rpy_angle_difference_degrees,
)
from plane_session import SUPPORTED_PROFILE_IDS


_IMAGE_SIZE = [640, 480]
_ROBOT_IP = "192.168.192.160"
_STATIC_WARNING = "仅静态验证，诊断Z不得直接用于运动。"
_RPY_TOLERANCE_DEG = 0.5


def _validate_profile_id(profile_id) -> None:
    if not isinstance(profile_id, str) or profile_id not in SUPPORTED_PROFILE_IDS:
        supported = ", ".join(sorted(SUPPORTED_PROFILE_IDS))
        raise ValueError(
            f"配置 profile_id {profile_id!r} 不受支持；支持：{supported}"
        )


def _finite_vector(values, size: int, name: str) -> np.ndarray:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    try:
        items = list(values)
    except TypeError:
        raise ValueError(f"{name} must contain {size} finite numbers") from None
    if len(items) != size or any(
        isinstance(item, bool) or not isinstance(item, Real) or not math.isfinite(item)
        for item in items
    ):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return np.asarray(items, dtype=float)


def _finite_homography(value) -> np.ndarray:
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        raise ValueError("homography H must be a finite 3x3 matrix") from None
    if matrix.shape != (3, 3):
        raise ValueError("homography H must have shape (3, 3)")
    if not np.isfinite(matrix).all():
        raise ValueError("homography H must contain only finite values")
    return matrix


def load_profile(path: Path, expected_profile_id: str) -> dict:
    """Load and validate one calibration result for the expected station profile."""
    _validate_profile_id(expected_profile_id)
    with Path(path).open("r", encoding="utf-8") as handle:
        document = json.load(handle)
    if not isinstance(document, dict):
        raise ValueError("配置 JSON 顶层必须是对象")

    schema_version = document.get("schema_version")
    if type(schema_version) is not int or schema_version != 1:
        raise ValueError(
            f"配置 schema_version 必须为整数1，实际为 {schema_version!r}"
        )

    profile_id = document.get("profile_id")
    _validate_profile_id(profile_id)
    if profile_id != expected_profile_id:
        raise ValueError(
            f"配置不匹配：结果为 {profile_id!r}，请求为 {expected_profile_id!r}"
        )
    if document.get("toolcoord") != "toolcoord2":
        raise ValueError("配置 toolcoord 必须为 toolcoord2")
    if document.get("image_size") != _IMAGE_SIZE:
        raise ValueError("配置 image_size 图像尺寸必须为 [640, 480]")

    reference_pose = _finite_vector(
        document.get("reference_photo_pose"), 6, "reference photo pose 位姿"
    )
    homography = _finite_homography(document.get("homography"))
    plane = _finite_vector(
        document.get("plane_coefficients"), 3, "plane coefficients 平面"
    )
    tolerance = document.get("rpy_tolerance_deg")
    if (
        isinstance(tolerance, bool)
        or not isinstance(tolerance, Real)
        or not math.isfinite(tolerance)
        or float(tolerance) != _RPY_TOLERANCE_DEG
    ):
        raise ValueError(
            "RPY tolerance 姿态容差在 schema_version=1 中必须精确为 0.5 度"
        )
    outer_tolerance = document.get("outer_tolerance_mm")
    if (
        isinstance(outer_tolerance, bool)
        or not isinstance(outer_tolerance, Real)
        or not math.isfinite(outer_tolerance)
        or outer_tolerance <= 0.0
    ):
        raise ValueError("outer tolerance 外圈容差必须为有限正数")

    profile = dict(document)
    profile["image_size"] = list(_IMAGE_SIZE)
    profile["reference_photo_pose"] = reference_pose.tolist()
    profile["homography"] = homography.tolist()
    profile["plane_coefficients"] = plane.tolist()
    profile["rpy_tolerance_deg"] = float(tolerance)
    profile["outer_tolerance_mm"] = float(outer_tolerance)
    return profile


def locate_pixel(
    profile: dict, current_photo_pose: Sequence[float], pixel: Sequence[float]
) -> dict:
    """Return static base-frame coordinates for one zero-based image pixel."""
    pose = _finite_vector(current_photo_pose, 6, "current photo pose 位姿")
    uv = _finite_vector(pixel, 2, "pixel 像素")
    width, height = profile["image_size"]
    if not (0.0 <= uv[0] < width and 0.0 <= uv[1] < height):
        raise ValueError(
            f"pixel 像素超出图像范围：u={uv[0]}, v={uv[1]}，"
            f"有效范围为 0<=u<{width}, 0<=v<{height}"
        )

    angle_difference = rpy_angle_difference_degrees(
        pose[3:], profile["reference_photo_pose"][3:]
    )
    if angle_difference > profile["rpy_tolerance_deg"] + 1e-12:
        raise ValueError(
            f"当前TCP姿态与配置参考姿态的RPY差异为 "
            f"{angle_difference:.6f} deg，超过容差 "
            f"{profile['rpy_tolerance_deg']:.6f} deg"
        )

    q_x, q_y = apply_homography(
        np.asarray(profile["homography"], dtype=float), uv.reshape(1, 2)
    )[0]
    a, b, c = profile["plane_coefficients"]
    q_z = a * q_x + b * q_y + c
    base = photo_local_to_base(pose, np.array([[q_x, q_y, q_z]], dtype=float))[0]

    return {
        "profile_id": profile["profile_id"],
        "image_size": list(profile["image_size"]),
        "schema_version": profile["schema_version"],
        "base_xy_mm": [float(base[0]), float(base[1])],
        "diagnostic_plane_z_mm": float(q_z),
        "base_point_mm": [float(value) for value in base],
        "pixel": [float(uv[0]), float(uv[1])],
        "motion_authorized": False,
    }


def _read_fairino_tcp() -> Sequence[float]:
    """Acquire one TCP pose without sending any motion or configuration command."""
    from fairino import Robot

    robot = Robot.RPC(_ROBOT_IP)
    try:
        error_code, tcp_pose = robot.GetActualTCPPose(0)
        if error_code != 0:
            raise RuntimeError(f"读取Fairino TCP失败，错误码：{error_code}")
        return tcp_pose
    finally:
        robot.CloseRPC()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="使用已标定平面配置执行静态定位；不会授权或执行运动。"
    )
    parser.add_argument("--profile", type=Path, required=True, help="标定结果JSON路径")
    parser.add_argument("--profile-id", required=True, help="预期配置ID")
    parser.add_argument(
        "--pixel", nargs=2, type=float, required=True, metavar=("U", "V"),
        help="零基图像像素坐标",
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--tcp", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="离线TCP位姿：毫米与角度；提供后不连接硬件",
    )
    source.add_argument(
        "--fairino", action="store_true",
        help="从Fairino只读获取当前TCP（省略--tcp时为默认行为）",
    )
    arguments = parser.parse_args(argv)

    profile = load_profile(arguments.profile, arguments.profile_id)
    current_pose = (
        arguments.tcp if arguments.tcp is not None else _read_fairino_tcp()
    )
    result = locate_pixel(profile, current_pose, arguments.pixel)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    print(_STATIC_WARNING)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
