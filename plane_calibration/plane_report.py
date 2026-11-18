"""UTF-8 reports for read-only, static plane-calibration results."""

import argparse
from pathlib import Path

from plane_locate import load_profile


STATIC_WARNING = "仅用于静态验证，禁止据此直接自动运动。"


def render_report(result: dict) -> str:
    """Render calibration metrics and traceability without authorizing motion."""
    lines = [
        "Single-station plane calibration report",
        "=======================================",
        f"result_id: {result['result_id']}",
        f"created_at: {result['created_at']}",
        f"profile_id: {result['profile_id']}",
        f"schema_version: {result['schema_version']}",
        f"toolcoord: {result['toolcoord']}",
        f"image_size: {result['image_size']}",
        f"reference_photo_pose: {result['reference_photo_pose']}",
        f"rpy_tolerance_deg: {result['rpy_tolerance_deg']}",
        f"center_tolerance_mm: {result['center_tolerance_mm']}",
        f"outer_tolerance_mm: {result['outer_tolerance_mm']}",
        f"source_png: {result['source_png']}",
        f"source_session: {result['source_session']}",
        "",
        "homography (pixel to reference-photo-local XY):",
    ]
    for row in result["homography"]:
        lines.append("  " + "  ".join(f"{value:.12g}" for value in row))
    lines.extend([
        f"plane_coefficients: {result['plane_coefficients']}",
        "",
        "outer fit metrics (points 0-7):",
    ])
    outer = result["outer_fit_metrics"]
    for key in (
        "rms_x_mm", "rms_y_mm", "rms_planar_mm",
        "max_planar_error_mm", "max_planar_error_index",
    ):
        lines.append(f"  {key}: {outer[key]}")
    lines.extend([
        "",
        "per-point outer residuals from the outer-eight homography:",
        "point index | residual_x_mm | residual_y_mm | planar_error_mm",
    ])
    outer_residuals = outer["point_residuals"]
    if len(outer_residuals) != 8:
        raise ValueError("report requires eight outer-point residuals")
    for expected_index, item in enumerate(outer_residuals):
        if item["index"] != expected_index:
            raise ValueError("report outer residuals are not in index order")
        lines.append(
            f"outer point {expected_index}: "
            f"residual_x_mm={item['residual_xy_mm'][0]:.9f}, "
            f"residual_y_mm={item['residual_xy_mm'][1]:.9f}, "
            f"planar_error_mm={item['planar_error_mm']:.9f}"
        )
    lines.extend([
        "",
        "fit metrics:",
    ])
    for key, value in result["fit_metrics"].items():
        lines.append(f"  {key}: {value}")
    lines.extend([
        "",
        "per-point residuals from the final nine-point homography:",
        "point index | residual_x_mm | residual_y_mm | planar_error_mm",
    ])
    residuals = result["point_residuals"]
    if len(residuals) != 9:
        raise ValueError("report requires nine per-point residuals")
    for expected_index, item in enumerate(residuals):
        if item["index"] != expected_index:
            raise ValueError("report point residuals are not in index order")
        lines.append(
            f"point {expected_index}: "
            f"residual_x_mm={item['residual_xy_mm'][0]:.9f}, "
            f"residual_y_mm={item['residual_xy_mm'][1]:.9f}, "
            f"planar_error_mm={item['planar_error_mm']:.9f}"
        )
    center = result["center_validation"]
    lines.extend([
        "",
        "independent center validation (outer points 0-7 only):",
        f"  fit_point_indices: {center['fit_point_indices']}",
        f"  predicted_local_xyz: {center['predicted_local_xyz']}",
        f"  actual_local_xyz: {center['actual_local_xyz']}",
        f"independent_center_error_xy_mm: {center['error_xy_mm']:.9f}",
        "",
        STATIC_WARNING,
    ])
    return "\n".join(lines) + "\n"


def write_report(result: dict, output_path: Path) -> Path:
    """Write a UTF-8 text report after a successful calibration."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(render_report(result), encoding="utf-8", newline="\n")
    return output_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="生成静态平面标定UTF-8报告。")
    parser.add_argument("--profile", type=Path, required=True, help="标定结果JSON")
    parser.add_argument("--profile-id", required=True, help="预期配置ID")
    parser.add_argument("--output", type=Path, help="报告路径；默认与结果同名.txt")
    arguments = parser.parse_args(argv)
    result = load_profile(arguments.profile, arguments.profile_id)
    output = arguments.output or arguments.profile.with_suffix(".txt")
    write_report(result, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
