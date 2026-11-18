"""Pure NumPy Fairino pose, homography, and photo-local plane helpers.

Translations are in millimetres and Fairino RPY angles are in degrees, with
R = Rz(rz) @ Ry(ry) @ Rx(rx).
"""

from numbers import Real
from typing import Sequence

import numpy as np


def _finite_vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    if vector.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},)")
    if not np.isfinite(vector).all():
        raise ValueError(f"{name} must contain only finite values")
    return vector


def _finite_points(points: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{name} must have shape (n, 3)")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain only finite values")
    return array


def _rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
    rx, ry, rz = np.radians(rpy)
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rotation_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cx, -sx],
        [0.0, sx, cx],
    ])
    rotation_y = np.array([
        [cy, 0.0, sy],
        [0.0, 1.0, 0.0],
        [-sy, 0.0, cy],
    ])
    rotation_z = np.array([
        [cz, -sz, 0.0],
        [sz, cz, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return rotation_z @ rotation_y @ rotation_x


def tcp_pose_to_matrix(pose: Sequence[float]) -> np.ndarray:
    """Convert [x, y, z, rx, ry, rz] (mm, degrees) to a 4x4 matrix."""
    values = _finite_vector(pose, 6, "pose")
    transform = np.eye(4)
    transform[:3, :3] = _rpy_to_rotation(values[3:])
    transform[:3, 3] = values[:3]
    return transform


def rpy_angle_difference_degrees(
    a: Sequence[float], b: Sequence[float]
) -> float:
    """Return the smallest rotation angle between two Fairino RPY poses."""
    rotation_a = _rpy_to_rotation(_finite_vector(a, 3, "a"))
    rotation_b = _rpy_to_rotation(_finite_vector(b, 3, "b"))
    cosine = (np.trace(rotation_a.T @ rotation_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def base_points_to_photo_local(
    photo_pose: Sequence[float], base_points: np.ndarray
) -> np.ndarray:
    """Convert an (n, 3) array of base-frame points to photo-local points."""
    transform = tcp_pose_to_matrix(photo_pose)
    points = _finite_points(base_points, "base_points")
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return (points - translation) @ rotation


def photo_local_to_base(
    photo_pose: Sequence[float], local_points: np.ndarray
) -> np.ndarray:
    """Convert an (n, 3) array of photo-local points to base-frame points."""
    transform = tcp_pose_to_matrix(photo_pose)
    points = _finite_points(local_points, "local_points")
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return points @ rotation.T + translation


def _finite_xy_points(points: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError(f"{name} must have shape (n, 2)")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain only finite values")
    return array


def _normalization_transform(points: np.ndarray, name: str) -> np.ndarray:
    if np.unique(points, axis=0).shape[0] != points.shape[0]:
        raise ValueError(f"{name} contains duplicate points")

    center = np.mean(points, axis=0)
    centered = points - center
    if np.linalg.matrix_rank(centered) < 2:
        raise ValueError(f"{name} is degenerate or collinear")

    mean_distance = np.mean(np.linalg.norm(centered, axis=1))
    scale = np.sqrt(2.0) / mean_distance
    return np.array([
        [scale, 0.0, -scale * center[0]],
        [0.0, scale, -scale * center[1]],
        [0.0, 0.0, 1.0],
    ])


def fit_homography(pixels: np.ndarray, local_xy: np.ndarray) -> np.ndarray:
    """Fit a normalized-DLT homography from pixel coordinates to local XY."""
    source = _finite_xy_points(pixels, "pixels")
    target = _finite_xy_points(local_xy, "local_xy")
    if source.shape != target.shape:
        raise ValueError("pixels and local_xy must have matching shapes")
    if source.shape[0] < 4:
        raise ValueError("at least four point pairs are required")

    source_transform = _normalization_transform(source, "pixels")
    target_transform = _normalization_transform(target, "local_xy")
    source_h = np.column_stack([source, np.ones(source.shape[0])])
    target_h = np.column_stack([target, np.ones(target.shape[0])])
    source_normalized = (source_transform @ source_h.T).T
    target_normalized = (target_transform @ target_h.T).T

    rows = []
    for (u, v, _), (x, y, _) in zip(source_normalized, target_normalized):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y])
    design = np.asarray(rows)
    if np.linalg.matrix_rank(design) < 8:
        raise ValueError("point configuration has insufficient DLT rank")

    _, _, vh = np.linalg.svd(design, full_matrices=True)
    normalized_h = vh[-1].reshape(3, 3)
    homography = (
        np.linalg.inv(target_transform) @ normalized_h @ source_transform
    )
    if not np.isfinite(homography).all():
        raise ValueError("homography fit produced non-finite values")

    homography = homography / np.max(np.abs(homography))
    if abs(homography[2, 2]) < 1e-12:
        raise ValueError("homography has a degenerate H[2, 2] entry")
    return homography / homography[2, 2]


def apply_homography(homography: np.ndarray, pixels: np.ndarray) -> np.ndarray:
    """Map an (n, 2) pixel array through a 3x3 homography."""
    matrix = np.asarray(homography, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("homography must have shape (3, 3)")
    if not np.isfinite(matrix).all():
        raise ValueError("homography must contain only finite values")
    points = _finite_xy_points(pixels, "pixels")

    homogeneous = np.column_stack([points, np.ones(points.shape[0])])
    projected = homogeneous @ matrix.T
    denominator = projected[:, 2]
    denominator_scale = (
        np.linalg.norm(matrix[2]) * np.linalg.norm(homogeneous, axis=1)
    )
    threshold = (
        16.0 * np.finfo(float).eps * np.maximum(1.0, denominator_scale)
    )
    if np.any(np.abs(denominator) <= threshold):
        raise ValueError("homography projection denominator is near zero")

    result = projected[:, :2] / denominator[:, None]
    if not np.isfinite(result).all():
        raise ValueError("homography projection produced non-finite values")
    return result


def _planar_residual_metrics(
    predicted: np.ndarray,
    actual: np.ndarray,
    point_indices: list[int] | None = None,
) -> dict:
    residuals = predicted - actual
    axis_rms = np.sqrt(np.mean(residuals**2, axis=0))
    planar_errors = np.linalg.norm(residuals, axis=1)
    maximum_index = int(np.argmax(planar_errors))
    metrics = {
        "rms_x_mm": float(axis_rms[0]),
        "rms_y_mm": float(axis_rms[1]),
        "rms_planar_mm": float(np.sqrt(np.mean(planar_errors**2))),
        "max_planar_error_mm": float(planar_errors[maximum_index]),
        "max_planar_error_index": maximum_index,
    }
    if point_indices is not None:
        metrics["fit_point_indices"] = list(point_indices)
        metrics["point_residuals"] = [
            {
                "index": index,
                "residual_xy_mm": residuals[offset].tolist(),
                "planar_error_mm": float(planar_errors[offset]),
            }
            for offset, index in enumerate(point_indices)
        ]
    return metrics


def solve_plane_calibration(
    local_points: np.ndarray,
    pixels: np.ndarray,
    center_tolerance_mm: float,
    outer_tolerance_mm: float,
) -> dict:
    """Fit nine-point local-plane calibration with outer and center gates."""
    local = _finite_points(local_points, "local_points")
    pixel_array = _finite_xy_points(pixels, "pixels")
    if local.shape != (9, 3) or pixel_array.shape != (9, 2):
        raise ValueError("local_points and pixels must have shapes (9, 3) and (9, 2)")
    try:
        tolerance = float(center_tolerance_mm)
    except (TypeError, ValueError):
        raise ValueError("center tolerance must be a finite non-negative number") from None
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("center tolerance must be a finite non-negative number")
    if isinstance(outer_tolerance_mm, bool) or not isinstance(outer_tolerance_mm, Real):
        raise ValueError("outer tolerance must be a finite positive number")
    outer_tolerance = float(outer_tolerance_mm)
    if not np.isfinite(outer_tolerance) or outer_tolerance <= 0.0:
        raise ValueError("outer tolerance must be a finite positive number")

    outer_h = fit_homography(pixel_array[:8], local[:8, :2])
    outer_predicted = apply_homography(outer_h, pixel_array[:8])
    outer_metrics = _planar_residual_metrics(
        outer_predicted, local[:8, :2], list(range(8))
    )
    if (
        outer_metrics["rms_planar_mm"] > outer_tolerance
        or outer_metrics["max_planar_error_mm"] > outer_tolerance
    ):
        raise ValueError(
            "外圈拟合残差超限: "
            f"rms_planar_mm={outer_metrics['rms_planar_mm']:.6f}, "
            f"max_planar_error_mm={outer_metrics['max_planar_error_mm']:.6f} "
            f"(point {outer_metrics['max_planar_error_index']}), "
            f"outer_tolerance_mm={outer_tolerance:.6f}"
        )

    center_xy = apply_homography(outer_h, pixel_array[8:9])[0]
    center_error = center_xy - local[8, :2]
    center_error_mm = float(np.linalg.norm(center_error))
    if center_error_mm > tolerance:
        raise ValueError(f"中心独立检查失败: {center_error_mm:.3f} mm")

    final_h = fit_homography(pixel_array, local[:, :2])
    a, b, c = np.linalg.lstsq(
        np.column_stack([local[:, :2], np.ones(9)]),
        local[:, 2],
        rcond=None,
    )[0]

    final_metrics = _planar_residual_metrics(
        apply_homography(final_h, pixel_array), local[:, :2]
    )
    center_z = a * center_xy[0] + b * center_xy[1] + c

    return {
        "homography": final_h.tolist(),
        "plane_coefficients": [float(a), float(b), float(c)],
        "outer_fit_metrics": outer_metrics,
        "fit_metrics": final_metrics,
        "center_validation": {
            "fit_point_indices": list(range(8)),
            "predicted_local_xyz": [
                float(center_xy[0]), float(center_xy[1]), float(center_z)
            ],
            "actual_local_xyz": local[8].tolist(),
            "error_xy_mm": center_error_mm,
        },
    }
