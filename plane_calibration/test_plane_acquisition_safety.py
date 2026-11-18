import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np

import plane_calibrate
from plane_calibrate import (
    _collect_taught_points,
    _require_safety_confirmation,
    finalize_session,
    run_acquisition,
    write_versioned_result,
)
from plane_locate import load_profile, locate_pixel
from plane_math import apply_homography, photo_local_to_base
import plane_report  # noqa: F401  # Import must work without device SDKs.
from plane_session import append_session_event, create_session, load_session


SCRIPT_DIR = Path(__file__).resolve().parent
WARNING = "仅用于静态验证，禁止据此直接自动运动。"


def _json_object_without_duplicate_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


class AcquisitionSafetyTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.reference_pose = [410.0, 20.0, 350.0, 165.0, -12.0, 35.0]
        local_xy = np.array([
            [-40.0, -30.0],
            [0.0, -30.0],
            [40.0, -30.0],
            [-40.0, 0.0],
            [40.0, 0.0],
            [-40.0, 30.0],
            [0.0, 30.0],
            [40.0, 30.0],
            [0.0, 0.0],
        ])
        local_z = 0.02 * local_xy[:, 0] - 0.03 * local_xy[:, 1] - 100.0
        self.local_points = np.column_stack([local_xy, local_z])
        local_to_pixel = np.array([
            [2.4, 0.15, 320.0],
            [-0.08, 2.1, 240.0],
            [0.0004, -0.0003, 1.0],
        ])
        homogeneous = np.column_stack([local_xy, np.ones(9)])
        projected = homogeneous @ local_to_pixel.T
        self.pixels = projected[:, :2] / projected[:, 2:3]
        self.taught_base = photo_local_to_base(
            self.reference_pose, self.local_points
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def _make_session(self, name="session.jsonl", local_points=None, taught_count=9):
        session_path = self.root / name
        image_path = self.root / f"{session_path.stem}.png"
        image_path.write_bytes(b"offline fixture; finalization does not decode the image")
        create_session(
            session_path, "warehouse_raw_1", self.reference_pose, image_path
        )
        local = self.local_points if local_points is None else local_points
        taught = photo_local_to_base(self.reference_pose, local)
        for index in range(9):
            append_session_event(session_path, "warehouse_raw_1", {
                "type": "pixel", "index": index,
                "pixel": self.pixels[index].tolist(),
            })
        for index in range(taught_count):
            append_session_event(session_path, "warehouse_raw_1", {
                "type": "taught_point", "index": index,
                "tcp_pose": [*taught[index].tolist(), 165.0, -12.0, 35.0],
            })
        return session_path

    def test_source_has_no_motion_calls_and_only_read_only_robot_methods(self):
        source = (SCRIPT_DIR / "plane_calibrate.py").read_text(encoding="utf-8")
        for forbidden in ["MoveL(", "MoveJ(", "MoveGripper(", "ActGripper(",
                          "StartMove(", "nav_to"]:
            self.assertNotIn(forbidden, source)
        self.assertIn("GetActualTCPPose", source)
        tree = ast.parse(source)
        robot_calls = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "robot"
        }
        self.assertEqual(robot_calls, {"GetActualTCPPose", "CloseRPC"})

    def test_generated_result_schema_round_trips_and_report_has_all_points(self):
        session_path = self._make_session()
        result, result_path, report_path = finalize_session(
            session_path,
            center_tolerance_mm=0.01,
            outer_tolerance_mm=1.0,
            output_root=self.root,
        )
        loaded = load_profile(result_path, "warehouse_raw_1")
        required = {
            "result_id", "created_at", "profile_id", "schema_version",
            "toolcoord", "image_size", "reference_photo_pose",
            "rpy_tolerance_deg", "homography", "plane_coefficients",
            "fit_metrics", "outer_fit_metrics", "center_validation",
            "center_tolerance_mm", "outer_tolerance_mm",
            "source_png", "source_session", "point_residuals",
            "motion_authorized",
        }
        self.assertTrue(required.issubset(loaded))
        self.assertEqual(loaded, result)
        self.assertIs(type(loaded["schema_version"]), int)
        self.assertEqual(loaded["schema_version"], 1)
        self.assertEqual(loaded["toolcoord"], "toolcoord2")
        self.assertEqual(loaded["image_size"], [640, 480])
        self.assertEqual(loaded["rpy_tolerance_deg"], 0.5)
        self.assertEqual(loaded["outer_tolerance_mm"], 1.0)
        self.assertEqual(
            loaded["outer_fit_metrics"]["fit_point_indices"], list(range(8))
        )
        self.assertEqual(len(loaded["outer_fit_metrics"]["point_residuals"]), 8)
        self.assertEqual(loaded["source_session"], str(session_path))
        self.assertEqual(loaded["source_png"], str(session_path.with_suffix(".png")))
        self.assertIs(loaded["motion_authorized"], False)
        self.assertEqual(len(loaded["point_residuals"]), 9)
        np.testing.assert_allclose(
            apply_homography(np.array(loaded["homography"]), self.pixels),
            self.local_points[:, :2], atol=1e-7,
        )
        self.assertLess(loaded["center_validation"]["error_xy_mm"], 0.01)
        report = report_path.read_text(encoding="utf-8")
        for index in range(9):
            self.assertIn(f"point {index}", report)
        self.assertIn("outer_tolerance_mm: 1.0", report)
        self.assertIn("outer fit metrics (points 0-7)", report)
        self.assertIn("outer point 0", report)
        self.assertIn("independent_center_error_xy_mm", report)
        self.assertTrue(report.rstrip().endswith(WARNING))

    def test_one_full_synthetic_profile_and_three_entry_registry_check(self):
        session_path = self._make_session()
        result, result_path, _ = finalize_session(
            session_path,
            center_tolerance_mm=0.01,
            outer_tolerance_mm=1.0,
            output_root=self.root,
        )
        loaded = load_profile(result_path, "warehouse_raw_1")

        current_pose = list(self.reference_pose)
        current_pose[:3] = np.asarray(current_pose[:3]) + [60.0, 30.0, 0.0]
        expected_xy = self.taught_base[8, :2] + [60.0, 30.0]
        location = locate_pixel(loaded, current_pose, self.pixels[8])

        np.testing.assert_allclose(
            location["base_xy_mm"], expected_xy, atol=1e-7
        )
        np.testing.assert_allclose(
            location["base_point_mm"][:2], expected_xy, atol=1e-7
        )
        self.assertIs(location["motion_authorized"], False)
        self.assertEqual(set(location), {
            "profile_id", "image_size", "schema_version", "base_xy_mm",
            "diagnostic_plane_z_mm", "base_point_mm", "pixel",
            "motion_authorized",
        })
        self.assertEqual(result["profile_id"], "warehouse_raw_1")

        registry_text = (SCRIPT_DIR / "plane_profiles.json").read_text(
            encoding="utf-8"
        )
        registry = json.loads(
            registry_text, object_pairs_hook=_json_object_without_duplicate_keys
        )
        self.assertEqual(set(registry), {
            "warehouse_raw_1", "warehouse_raw_2", "machine_finished_pick",
        })
        result_paths = [entry["result"] for entry in registry.values()]
        self.assertEqual(len(result_paths), len(set(result_paths)))
        expected_descriptions = {
            "warehouse_raw_1": (
                "仓库第1行（warehouse_raw_1）使用仓库标准拍照 RPY，"
                "与 warehouse_raw_2 共享同一套仓库 RPY。"
            ),
            "warehouse_raw_2": (
                "仓库第2行（warehouse_raw_2）使用仓库标准拍照 RPY，"
                "与 warehouse_raw_1 共享同一套仓库 RPY。"
            ),
            "machine_finished_pick": (
                "加工区（machine_finished_pick）使用独立的加工区标准拍照 RPY，"
                "不与仓库 RPY 共用。"
            ),
        }
        for profile_id, entry in registry.items():
            self.assertEqual(entry["result"], f"plane_results/{profile_id}.json")
            self.assertIs(entry["z_teaching_required"], True)
            self.assertEqual(
                entry["standard_photo_description"],
                expected_descriptions[profile_id],
            )
            self.assertIsNone(entry["site_z_parameter"])

        with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
            json.loads(
                '{"profile": {}, "profile": {}}',
                object_pairs_hook=_json_object_without_duplicate_keys,
            )

    def test_center_failure_preserves_session_and_existing_result(self):
        bad_local = self.local_points.copy()
        bad_local[8, :2] += [5.0, -4.0]
        session_path = self._make_session(local_points=bad_local)
        original_session = session_path.read_bytes()
        result_path = self.root / "plane_results" / "warehouse_raw_1.json"
        result_path.parent.mkdir(parents=True)
        result_path.write_bytes(b"existing result must remain untouched")
        with self.assertRaisesRegex(ValueError, "center|中心"):
            finalize_session(
                session_path,
                center_tolerance_mm=0.5,
                outer_tolerance_mm=1.0,
                output_root=self.root,
            )
        self.assertEqual(session_path.read_bytes(), original_session)
        self.assertEqual(
            result_path.read_bytes(), b"existing result must remain untouched"
        )
        self.assertEqual(list(result_path.parent.glob("*.backup-*.json")), [])

    def test_incomplete_session_cannot_create_result(self):
        session_path = self._make_session(taught_count=8)
        with self.assertRaisesRegex(ValueError, "incomplete|不完整|nine|9"):
            finalize_session(
                session_path,
                center_tolerance_mm=0.01,
                outer_tolerance_mm=1.0,
                output_root=self.root,
            )
        self.assertFalse(
            (self.root / "plane_results" / "warehouse_raw_1.json").exists()
        )

    def test_existing_result_is_backed_up_before_atomic_replacement(self):
        session_path = self._make_session()
        original, result_path, _ = finalize_session(
            session_path,
            center_tolerance_mm=0.01,
            outer_tolerance_mm=1.0,
            output_root=self.root,
        )
        replacement = dict(original)
        replacement["result_id"] = "warehouse_raw_1-offline-replacement"
        replacement["created_at"] = "2026-11-18T13:00:00+08:00"
        write_versioned_result(replacement, result_path)
        backups = list(result_path.parent.glob("warehouse_raw_1.backup-*.json"))
        self.assertEqual(len(backups), 1)
        self.assertEqual(json.loads(backups[0].read_text(encoding="utf-8")), original)
        self.assertEqual(
            json.loads(result_path.read_text(encoding="utf-8")), replacement
        )
        self.assertEqual(list(result_path.parent.glob("*.tmp")), [])

    def test_unknown_profile_is_rejected_before_any_output_or_hardware(self):
        output_root = self.root / "invalid-profile-output"
        with mock.patch.object(
            plane_calibrate, "_new_session_paths",
            side_effect=AssertionError("session path creation attempted"),
        ) as new_paths, mock.patch.object(
            plane_calibrate, "_connect_robot",
            side_effect=AssertionError("robot connection attempted"),
        ) as connect, mock.patch.object(
            plane_calibrate, "_capture_static_image",
            side_effect=AssertionError("camera capture attempted"),
        ) as capture:
            with self.assertRaisesRegex(ValueError, "profile|配置"):
                run_acquisition(
                    "warehouse_raw_1;unknown",
                    center_tolerance_mm=0.01,
                    outer_tolerance_mm=1.0,
                    output_root=output_root,
                )
        new_paths.assert_not_called()
        connect.assert_not_called()
        capture.assert_not_called()
        self.assertFalse(output_root.exists())
    def test_complete_resume_finalizes_without_camera_robot_or_gui(self):
        session_path = self._make_session()
        with mock.patch.object(
            plane_calibrate, "_connect_robot",
            side_effect=AssertionError("robot connection attempted"),
        ), mock.patch.object(
            plane_calibrate, "_capture_static_image",
            side_effect=AssertionError("camera capture attempted"),
        ), mock.patch.object(
            plane_calibrate, "_load_saved_image",
            side_effect=AssertionError("GUI image load attempted"),
        ):
            result, _, _ = run_acquisition(
                "warehouse_raw_1",
                center_tolerance_mm=0.01,
                outer_tolerance_mm=1.0,
                resume=session_path,
                output_root=self.root,
            )
        self.assertEqual(result["source_session"], str(session_path))

    def test_finalization_rejects_missing_or_changed_reference_image(self):
        for change in ("missing", "changed"):
            with self.subTest(change=change):
                session_path = self._make_session(name=f"image-{change}.jsonl")
                image_path = session_path.with_suffix(".png")
                if change == "missing":
                    image_path.unlink()
                else:
                    changed = bytearray(image_path.read_bytes())
                    changed[0] ^= 0x01
                    image_path.write_bytes(bytes(changed))
                with self.assertRaisesRegex(
                    ValueError, "image|reference|missing|changed|不存在|变化"
                ):
                    finalize_session(
                        session_path,
                        center_tolerance_mm=0.01,
                        outer_tolerance_mm=1.0,
                        output_root=self.root,
                    )
                self.assertFalse(
                    (self.root / "plane_results" / "warehouse_raw_1.json").exists()
                )

    def test_calibrate_cli_requires_outer_tolerance(self):
        missing_outer = subprocess.run(
            [
                sys.executable, str(SCRIPT_DIR / "plane_calibrate.py"),
                "--profile-id", "warehouse_raw_1",
                "--center-tolerance-mm", "1.0",
            ],
            cwd=SCRIPT_DIR,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=15,
            check=False,
        )
        self.assertEqual(missing_outer.returncode, 2)
        self.assertIn("--outer-tolerance-mm", missing_outer.stderr)

        help_result = subprocess.run(
            [sys.executable, str(SCRIPT_DIR / "plane_calibrate.py"), "--help"],
            cwd=SCRIPT_DIR,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=15,
            check=False,
        )
        self.assertEqual(help_result.returncode, 0, help_result.stderr)
        self.assertIn("--outer-tolerance-mm", help_result.stdout)
        self.assertIn("--center-tolerance-mm", help_result.stdout)

    def test_phase_b_resume_starts_at_first_missing_point_and_persists_undo(self):
        session_path = self._make_session(taught_count=8)
        state = load_session(session_path, "warehouse_raw_1")
        expected_pose = state["reference_photo_pose"]
        replacement = [
            *self.taught_base[8].tolist(), 165.0, -12.0, 35.0
        ]

        class FakeRobot:
            def __init__(self):
                self.poses = iter([[900.0, 900.0, 900.0, 0.0, 0.0, 0.0], replacement])
                self.read_count = 0

            def GetActualTCPPose(self, _):
                self.read_count += 1
                return 0, next(self.poses)

        robot = FakeRobot()
        commands = iter(["", "u", "", ""])
        fake_cv2 = types.SimpleNamespace(
            imshow=lambda *_: None,
            waitKey=lambda *_: 1,
            destroyWindow=lambda *_: None,
        )
        with mock.patch.dict(sys.modules, {"cv2": fake_cv2}):
            active = _collect_taught_points(
                session_path,
                "warehouse_raw_1",
                state["taught_points"],
                np.zeros((480, 640, 3), dtype=np.uint8),
                robot,
                input_fn=lambda _: next(commands),
            )
        self.assertEqual(robot.read_count, 2)
        self.assertEqual(len(active), 9)
        self.assertEqual(active[8]["tcp_pose"], replacement)
        self.assertNotEqual(active[8]["tcp_pose"], expected_pose)
        replayed = load_session(session_path, "warehouse_raw_1")
        self.assertEqual(replayed["taught_points"], active)
        raw_events = [json.loads(line) for line in session_path.read_text(
            encoding="utf-8"
        ).splitlines()]
        self.assertEqual(raw_events[-3]["type"], "taught_point")
        self.assertEqual(raw_events[-2]["type"], "undo")
        self.assertEqual(raw_events[-2]["target"], "taught_point")
        self.assertIn("timestamp", raw_events[-2])
        self.assertEqual(raw_events[-1]["type"], "taught_point")
        self.assertEqual(raw_events[-1]["index"], 8)
        self.assertIn("timestamp", raw_events[-1])

    def test_initial_confirmation_is_explicit(self):
        with self.assertRaisesRegex(ValueError, "confirmation|确认"):
            _require_safety_confirmation(input_fn=lambda _: "yes")
        _require_safety_confirmation(input_fn=lambda _: "确认只读手动示教")

    def test_module_imports_need_no_device_or_gui_packages(self):
        blocker = self.root / "blockers"
        blocker.mkdir()
        for module in ("fairino", "pyrealsense2", "cv2"):
            (blocker / f"{module}.py").write_text(
                f"raise AssertionError({module!r} + ' imported during offline import')\n",
                encoding="utf-8",
            )
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join([str(blocker), str(SCRIPT_DIR)])
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        completed = subprocess.run(
            [sys.executable, "-c", "import plane_calibrate, plane_report"],
            cwd=SCRIPT_DIR,
            env=environment,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=15,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
