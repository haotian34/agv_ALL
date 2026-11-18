"""Offline eye-in-hand calibration solver; no robot or camera connection.

Coordinate chain:
    T_base_target = T_base_tcp @ T_tcp_camera @ T_camera_target

TCP translations and solved translations are millimetres. Input camera points
are metres and are converted to millimetres before solving. Fairino TCP RPY is
interpreted as extrinsic XYZ: R = Rz(rz) @ Ry(ry) @ Rx(rx).
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)


def tcp_pose_to_matrix(tcp_pose):
    """Convert [x, y, z, rx, ry, rz] (mm, degrees) to a 4x4 matrix."""
    if len(tcp_pose) != 6:
        raise ValueError("tcp_pose must contain six values")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler(
        "xyz", np.asarray(tcp_pose[3:], dtype=float), degrees=True
    ).as_matrix()
    transform[:3, 3] = np.asarray(tcp_pose[:3], dtype=float)
    return transform


def error_metrics(errors):
    errors = np.asarray(errors, dtype=float).reshape(-1, 3)
    norms = np.linalg.norm(errors, axis=1)
    return {
        "rms_mm": float(np.sqrt(np.mean(norms ** 2))),
        "mean_mm": float(np.mean(norms)),
        "max_mm": float(np.max(norms)),
        "axis_rms_mm": np.sqrt(np.mean(errors ** 2, axis=0)).tolist(),
        "max_sample_index": int(np.argmax(norms) + 1),
    }


def predict_target(result, sample):
    base_from_tcp = tcp_pose_to_matrix(sample["tcp_pose"])
    camera_point_mm = np.asarray(sample["cam_point_m"], dtype=float) * 1000.0
    return (
        base_from_tcp[:3, :3]
        @ (result["rotation"] @ camera_point_mm + result["translation_mm"])
        + base_from_tcp[:3, 3]
    )


def solve_handeye(samples):
    """Jointly solve T_tcp_camera and the fixed target position in base frame."""
    if len(samples) < 6:
        raise ValueError("at least six samples are required")

    rotations = []
    translations = []
    camera_points = []
    for sample in samples:
        base_from_tcp = tcp_pose_to_matrix(sample["tcp_pose"])
        rotations.append(base_from_tcp[:3, :3])
        translations.append(base_from_tcp[:3, 3])
        camera_points.append(np.asarray(sample["cam_point_m"], dtype=float) * 1000.0)

    rotations = np.asarray(rotations)
    translations = np.asarray(translations)
    camera_points = np.asarray(camera_points)
    if not np.isfinite(camera_points).all():
        raise ValueError("camera points contain NaN or infinity")

    # Linear system in row-major R entries, camera offset t, and base target y:
    # R_i @ R @ p_i + R_i @ t + t_i - y = 0.
    rows = []
    rhs = []
    for rotation, translation, point in zip(rotations, translations, camera_points):
        block = np.zeros((3, 15))
        block[:, :9] = np.kron(point[None, :], rotation)
        block[:, 9:12] = rotation
        block[:, 12:15] = -np.eye(3)
        rows.append(block)
        rhs.append(-translation)
    matrix = np.vstack(rows)
    vector = np.concatenate(rhs)
    if np.linalg.matrix_rank(matrix) < 15:
        raise ValueError("samples do not provide enough pose diversity")

    initial = np.linalg.lstsq(matrix, vector, rcond=None)[0]
    u, _, vt = np.linalg.svd(initial[:9].reshape(3, 3))
    projected = u @ vt
    if np.linalg.det(projected) < 0:
        u[:, -1] *= -1
        projected = u @ vt
    initial_parameters = np.concatenate([
        Rotation.from_matrix(projected).as_rotvec(),
        initial[9:12],
        initial[12:15],
    ])

    def predicted_targets(parameters):
        rotation_tcp_camera = Rotation.from_rotvec(parameters[:3]).as_matrix()
        translation_tcp_camera = parameters[3:6]
        return (
            np.einsum("nij,jk,nk->ni", rotations, rotation_tcp_camera, camera_points)
            + np.einsum("nij,j->ni", rotations, translation_tcp_camera)
            + translations
        )

    def residual(parameters):
        return (predicted_targets(parameters) - parameters[6:9]).reshape(-1)

    optimized = least_squares(residual, initial_parameters, method="lm")
    rotation = Rotation.from_rotvec(optimized.x[:3]).as_matrix()
    translation = optimized.x[3:6]
    target = optimized.x[6:9]
    predicted = predicted_targets(optimized.x)
    errors = predicted - target
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    sample_results = []
    for index, (sample, point, error) in enumerate(zip(samples, predicted, errors), 1):
        sample_results.append({
            "index": index,
            "time": sample.get("time", ""),
            "predicted_target_base_mm": point.tolist(),
            "error_vector_mm": error.tolist(),
            "error_mm": float(np.linalg.norm(error)),
        })
    return {
        "transform": transform,
        "rotation": rotation,
        "translation_mm": translation,
        "target_base_mm": target,
        "metrics": error_metrics(errors),
        "samples": sample_results,
    }


def leave_one_out(samples):
    errors = []
    sample_results = []
    for index, sample in enumerate(samples):
        result = solve_handeye(samples[:index] + samples[index + 1:])
        error = predict_target(result, sample) - result["target_base_mm"]
        errors.append(error)
        sample_results.append({"index": index + 1, "error_mm": float(np.linalg.norm(error))})
    metrics = error_metrics(errors)
    metrics["samples"] = sample_results
    return metrics


def split_time_batches(samples):
    batches = []
    current = []
    for sample in samples:
        if current:
            previous_time = datetime.fromisoformat(current[-1]["time"])
            current_time = datetime.fromisoformat(sample["time"])
            if (current_time - previous_time).total_seconds() > 3600:
                batches.append(current)
                current = []
        current.append(sample)
    if current:
        batches.append(current)
    return batches


def analyze_batches(samples):
    batches = split_time_batches(samples)
    if len(batches) < 2:
        return []
    output = []
    start = 1
    fitted = []
    for batch in batches:
        if len(batch) >= 6:
            result = solve_handeye(batch)
            fitted.append((start, batch, result))
            output.append({
                "start_index": start,
                "end_index": start + len(batch) - 1,
                "sample_count": len(batch),
                "translation_mm": result["translation_mm"].tolist(),
                "target_base_mm": result["target_base_mm"].tolist(),
                "metrics": result["metrics"],
            })
        start += len(batch)
    for train_start, train_samples, result in fitted:
        for test_start, test_samples, _ in fitted:
            if train_start == test_start:
                continue
            errors = [predict_target(result, sample) - result["target_base_mm"] for sample in test_samples]
            output.append({
                "train_start_index": train_start,
                "test_start_index": test_start,
                "metrics": error_metrics(errors),
            })
    return output


def serializable_result(result, samples):
    leave_one_out_result = leave_one_out(samples)
    return {
        "coordinate_chain": "T_base_target = T_base_tcp @ T_tcp_camera @ T_camera_target",
        "rpy_convention": "extrinsic XYZ: R = Rz(rz) @ Ry(ry) @ Rx(rx), degrees",
        "translation_unit": "mm",
        "sample_count": len(samples),
        "transform": result["transform"].tolist(),
        "rotation": result["rotation"].tolist(),
        "rotation_rpy_deg": Rotation.from_matrix(result["rotation"]).as_euler("xyz", degrees=True).tolist(),
        "translation_mm": result["translation_mm"].tolist(),
        "target_base_mm": result["target_base_mm"].tolist(),
        "metrics": result["metrics"],
        "leave_one_out": leave_one_out_result,
        "samples": result["samples"],
        "batch_analysis": analyze_batches(samples),
        "usage_warning": "Offline result only. Perform static repeatability validation before any robot motion.",
    }


def write_report(path, data):
    metrics = data["metrics"]
    lines = [
        "手眼标定离线求解报告",
        "=" * 72,
        f"样本数: {data['sample_count']}",
        "坐标链: T_base_target = T_base_tcp @ T_tcp_camera @ T_camera_target",
        "RPY 约定: R = Rz(rz) @ Ry(ry) @ Rx(rx)，角度单位 deg，平移单位 mm",
        "",
        "T_tcp_camera（相机坐标 -> TCP 坐标，平移 mm）:",
    ]
    lines.extend("  " + "  ".join(f"{value:12.6f}" for value in row) for row in data["transform"])
    lines.extend([
        "",
        "相机相对 TCP 的平移 (mm): " + ", ".join(f"{v:.3f}" for v in data["translation_mm"]),
        "相机相对 TCP 的 RPY (deg): " + ", ".join(f"{v:.6f}" for v in data["rotation_rpy_deg"]),
        "固定靶点在基座坐标中的位置 (mm): " + ", ".join(f"{v:.3f}" for v in data["target_base_mm"]),
        "",
        "全体数据拟合误差:",
        f"  RMS = {metrics['rms_mm']:.3f} mm; 平均 = {metrics['mean_mm']:.3f} mm; 最大 = {metrics['max_mm']:.3f} mm",
        f"  最大误差样本 = 第 {metrics['max_sample_index']} 组",
        "  XYZ 各轴 RMS = " + ", ".join(f"{v:.3f}" for v in metrics["axis_rms_mm"]) + " mm",
        "",
        "逐组留一验证（每次用其余样本求解，再预测被留出的样本）:",
        f"  RMS = {data['leave_one_out']['rms_mm']:.3f} mm; 平均 = {data['leave_one_out']['mean_mm']:.3f} mm; 最大 = {data['leave_one_out']['max_mm']:.3f} mm",
        "",
        "每组误差（预测靶点 - 全体数据求得的固定靶点）:",
        "编号  时间                  dX(mm)    dY(mm)    dZ(mm)    合成误差(mm)",
    ])
    for sample in data["samples"]:
        error = sample["error_vector_mm"]
        lines.append(
            f"{sample['index']:>3}   {sample['time']:<19} "
            f"{error[0]:>9.3f}  {error[1]:>9.3f}  {error[2]:>9.3f}  {sample['error_mm']:>12.3f}"
        )
    if data["batch_analysis"]:
        lines.extend(["", "分批与跨批检查:", json.dumps(data["batch_analysis"], ensure_ascii=False, indent=2)])
    lines.extend([
        "",
        "注意：本报告只是离线标定结果，不是实抓许可。",
        "下一步必须在设备静止、低速和人工急停监护条件下做同一靶点多姿态静态复测；",
        "复测通过后仍需进行一次性抓取偏移标定及 50~100 mm 预抓取空跑。",
    ])
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Offline eye-in-hand calibration solver")
    parser.add_argument("--input", type=Path, default=script_dir / "handeye_data.jsonl")
    parser.add_argument("--output", type=Path, default=script_dir / "handeye_result.json")
    parser.add_argument("--report", type=Path, default=script_dir / "handeye_report.txt")
    args = parser.parse_args()
    with args.input.open("r", encoding="utf-8") as handle:
        samples = [json.loads(line) for line in handle if line.strip()]
    result = solve_handeye(samples)
    data = serializable_result(result, samples)
    args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_report(args.report, data)
    print(f"已处理 {len(samples)} 组数据。")
    print(f"拟合 RMS: {data['metrics']['rms_mm']:.3f} mm; 最大: {data['metrics']['max_mm']:.3f} mm")
    print(f"留一验证 RMS: {data['leave_one_out']['rms_mm']:.3f} mm; 最大: {data['leave_one_out']['max_mm']:.3f} mm")
    print(f"结果: {args.output}")
    print(f"报告: {args.report}")
    print("仅用于离线分析；静态复测通过前禁止据此自动运动或实抓。")


if __name__ == "__main__":
    main()
