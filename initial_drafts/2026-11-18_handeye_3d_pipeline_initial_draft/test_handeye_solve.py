import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from handeye_solve import solve_handeye, tcp_pose_to_matrix


def independent_rpy_matrix(rx, ry, rz):
    """R = Rz(rz) @ Ry(ry) @ Rx(rx), angles in degrees."""
    x, y, z = np.radians([rx, ry, rz])
    cx, sx = np.cos(x), np.sin(x)
    cy, sy = np.cos(y), np.sin(y)
    cz, sz = np.cos(z), np.sin(z)
    rx_m = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry_m = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz_m = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz_m @ ry_m @ rx_m


def make_clean_samples():
    rng = np.random.default_rng(20261116)
    rotation = independent_rpy_matrix(12.0, -8.0, 25.0)
    translation = np.array([32.0, -18.0, 75.0])
    target = np.array([410.0, -95.0, 260.0])
    samples = []
    for index in range(15):
        base_rotation = Rotation.random(random_state=rng)
        base_translation = rng.uniform([-250.0, -180.0, 180.0], [550.0, 220.0, 650.0])
        base_from_tcp = base_rotation.as_matrix()
        camera_point_mm = rotation.T @ (
            base_from_tcp.T @ (target - base_translation) - translation
        )
        rpy = base_rotation.as_euler("xyz", degrees=True)
        samples.append({
            "tcp_pose": [*base_translation.tolist(), *rpy.tolist()],
            "pixel": [320, 240],
            "depth_m": float(camera_point_mm[2] / 1000.0),
            "cam_point_m": (camera_point_mm / 1000.0).tolist(),
            "time": f"2026-11-16 10:{index:02d}:00",
        })
    return samples, rotation, translation, target


class HandeyeSolveTests(unittest.TestCase):
    def test_tcp_pose_uses_millimetres_and_fairino_rpy_order(self):
        pose = [10.0, 20.0, 30.0, 15.0, -25.0, 40.0]
        transform = tcp_pose_to_matrix(pose)
        expected_rotation = independent_rpy_matrix(15.0, -25.0, 40.0)
        np.testing.assert_allclose(transform[:3, :3], expected_rotation, atol=1e-12)
        np.testing.assert_allclose(transform[:3, 3], [10.0, 20.0, 30.0])
        np.testing.assert_allclose(transform[3], [0.0, 0.0, 0.0, 1.0])

    def test_solver_recovers_known_transform_from_clean_samples(self):
        samples, expected_rotation, expected_translation, expected_target = make_clean_samples()
        result = solve_handeye(samples)
        np.testing.assert_allclose(result["rotation"], expected_rotation, atol=1e-7)
        np.testing.assert_allclose(result["translation_mm"], expected_translation, atol=1e-5)
        np.testing.assert_allclose(result["target_base_mm"], expected_target, atol=1e-5)
        self.assertLess(result["metrics"]["rms_mm"], 1e-6)
        np.testing.assert_allclose(result["transform"][3], [0.0, 0.0, 0.0, 1.0])

    def test_solver_reports_every_samples_residual(self):
        samples, _, _, _ = make_clean_samples()
        result = solve_handeye(samples)
        self.assertEqual(len(result["samples"]), len(samples))
        self.assertEqual(result["samples"][0]["index"], 1)
        self.assertEqual(result["samples"][-1]["index"], len(samples))
        for sample_result in result["samples"]:
            self.assertLess(sample_result["error_mm"], 1e-5)

    def test_cli_writes_json_and_text_report(self):
        samples, expected_rotation, expected_translation, expected_target = make_clean_samples()
        script = Path(__file__).with_name("handeye_solve.py")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "input.jsonl"
            output_path = root / "result.json"
            report_path = root / "report.txt"
            input_path.write_text(
                "".join(json.dumps(sample) + "\n" for sample in samples), encoding="utf-8"
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--input",
                    str(input_path),
                    "--output",
                    str(output_path),
                    "--report",
                    str(report_path),
                ],
                capture_output=True,
                text=True, encoding="utf-8", timeout=30,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertTrue(output_path.is_file())
            self.assertTrue(report_path.is_file())
            result = json.loads(output_path.read_text(encoding="utf-8"))
            self.assertEqual(result["sample_count"], len(samples))
            np.testing.assert_allclose(result["rotation"], expected_rotation, atol=1e-7)
            np.testing.assert_allclose(result["translation_mm"], expected_translation, atol=1e-5)
            np.testing.assert_allclose(result["target_base_mm"], expected_target, atol=1e-5)
            self.assertLess(result["leave_one_out"]["rms_mm"], 1e-5)
            self.assertIn("RMS", report_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()

