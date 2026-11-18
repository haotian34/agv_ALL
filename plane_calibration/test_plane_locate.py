"""Offline tests for versioned profile loading and static localization."""

import ast
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

import plane_session
from plane_locate import load_profile, locate_pixel
import plane_locate


SCRIPT_DIR = Path(__file__).resolve().parent
WARNING = "仅静态验证，诊断Z不得直接用于运动。"


def rotation_from_rpy(rpy):
    """Independent test implementation of Fairino Rz @ Ry @ Rx."""
    rx, ry, rz = np.radians(rpy)
    rotation_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(rx), -np.sin(rx)],
        [0.0, np.sin(rx), np.cos(rx)],
    ])
    rotation_y = np.array([
        [np.cos(ry), 0.0, np.sin(ry)],
        [0.0, 1.0, 0.0],
        [-np.sin(ry), 0.0, np.cos(ry)],
    ])
    rotation_z = np.array([
        [np.cos(rz), -np.sin(rz), 0.0],
        [np.sin(rz), np.cos(rz), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return rotation_z @ rotation_y @ rotation_x


class ProfileTestCase(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.path = Path(self.temporary_directory.name) / "profile.json"
        self.profile = {
            "schema_version": 1,
            "profile_id": "warehouse_raw_1",
            "toolcoord": "toolcoord2",
            "image_size": [640, 480],
            "reference_photo_pose": [400, 0, 350, 165, -12, 35],
            "rpy_tolerance_deg": 0.5,
            "outer_tolerance_mm": 0.01,
            # Center pixel maps to local (0, 0); units are mm per pixel.
            "homography": [
                [0.1, 0.0, -32.0],
                [0.0, 0.1, -24.0],
                [0.0, 0.0, 1.0],
            ],
            "plane_coefficients": [0.02, -0.03, -100.0],
            "fit_metrics": {"rms_planar_mm": 0.0},
        }
        self.write_profile(self.profile)

    def write_profile(self, profile):
        self.path.write_text(
            json.dumps(profile, ensure_ascii=False), encoding="utf-8"
        )


class ProfileLoadingTests(ProfileTestCase):
    def test_load_profile_uses_the_session_whitelist(self):
        self.assertIs(
            plane_locate.SUPPORTED_PROFILE_IDS,
            plane_session.SUPPORTED_PROFILE_IDS,
        )
        loaded = load_profile(self.path, "warehouse_raw_1")
        self.assertEqual(loaded["profile_id"], "warehouse_raw_1")
        self.assertEqual(loaded["schema_version"], 1)
        self.assertEqual(loaded["fit_metrics"], {"rms_planar_mm": 0.0})

    def test_schema_version_must_be_exact_integer_one(self):
        missing = copy.deepcopy(self.profile)
        del missing["schema_version"]
        self.write_profile(missing)
        with self.assertRaisesRegex(ValueError, "schema_version"):
            load_profile(self.path, "warehouse_raw_1")

        invalid_versions = (None, True, False, "1", 1.0, 0, 2, [], {})
        for version in invalid_versions:
            with self.subTest(schema_version=version):
                invalid = copy.deepcopy(self.profile)
                invalid["schema_version"] = version
                self.write_profile(invalid)
                with self.assertRaisesRegex(ValueError, "schema_version"):
                    load_profile(self.path, "warehouse_raw_1")
    def test_wrong_profile_and_rpy_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "配置"):
            load_profile(self.path, "machine_finished_pick")
        wrong = [400, 0, 350, 170, -12, 35]
        with self.assertRaisesRegex(ValueError, "RPY|姿态"):
            locate_pixel(self.profile, wrong, [320, 240])

    def test_unsupported_profile_ids_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "配置|profile"):
            load_profile(self.path, "unknown_profile")
        unsupported = copy.deepcopy(self.profile)
        unsupported["profile_id"] = "unknown_profile"
        self.write_profile(unsupported)
        with self.assertRaisesRegex(ValueError, "配置|profile"):
            load_profile(self.path, "warehouse_raw_1")

    def test_wrong_tool_coordinate_is_rejected(self):
        self.profile["toolcoord"] = "toolcoord1"
        self.write_profile(self.profile)
        with self.assertRaisesRegex(ValueError, "toolcoord"):
            load_profile(self.path, "warehouse_raw_1")

    def test_wrong_image_size_is_rejected(self):
        for image_size in ([640, 481], [639, 480], [640], "640,480"):
            with self.subTest(image_size=image_size):
                self.profile["image_size"] = image_size
                self.write_profile(self.profile)
                with self.assertRaisesRegex(ValueError, "image|图像|尺寸"):
                    load_profile(self.path, "warehouse_raw_1")

    def test_invalid_reference_pose_is_rejected(self):
        bad_poses = [
            [400, 0, 350, 165, -12],
            [400, 0, np.nan, 165, -12, 35],
            [400, 0, 350, 165, np.inf, 35],
        ]
        for pose in bad_poses:
            with self.subTest(pose=pose):
                self.profile["reference_photo_pose"] = pose
                self.write_profile(self.profile)
                with self.assertRaisesRegex(ValueError, "pose|位姿|finite"):
                    load_profile(self.path, "warehouse_raw_1")

    def test_invalid_homography_shape_and_values_are_rejected(self):
        invalid = [
            [[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [1.0] * 9,
            [[1.0, 0.0, 0.0], [0.0, np.nan, 0.0], [0.0, 0.0, 1.0]],
        ]
        for homography in invalid:
            with self.subTest(homography=homography):
                self.profile["homography"] = homography
                self.write_profile(self.profile)
                with self.assertRaisesRegex(ValueError, "homography|H|finite"):
                    load_profile(self.path, "warehouse_raw_1")

    def test_invalid_plane_coefficients_are_rejected(self):
        for coefficients in ([0.02, -0.03], [0.02, -0.03, -100, 1],
                             [0.02, np.inf, -100]):
            with self.subTest(coefficients=coefficients):
                self.profile["plane_coefficients"] = coefficients
                self.write_profile(self.profile)
                with self.assertRaisesRegex(ValueError, "plane|平面|finite"):
                    load_profile(self.path, "warehouse_raw_1")

    def test_rpy_tolerance_must_be_exactly_half_degree(self):
        for tolerance in (0, -0.1, 0.49, 0.51, 1.0, 2.0, np.nan, np.inf,
                          True, "2", None):
            with self.subTest(tolerance=tolerance):
                self.profile["rpy_tolerance_deg"] = tolerance
                self.write_profile(self.profile)
                with self.assertRaisesRegex(ValueError, "RPY|tolerance|容差"):
                    load_profile(self.path, "warehouse_raw_1")

    def test_outer_tolerance_must_be_finite_positive(self):
        missing = copy.deepcopy(self.profile)
        del missing["outer_tolerance_mm"]
        self.write_profile(missing)
        with self.assertRaisesRegex(ValueError, "outer|tolerance|容差"):
            load_profile(self.path, "warehouse_raw_1")

        for tolerance in (0, -0.1, np.nan, np.inf, True, "0.01", None):
            with self.subTest(outer_tolerance_mm=tolerance):
                invalid = copy.deepcopy(self.profile)
                invalid["outer_tolerance_mm"] = tolerance
                self.write_profile(invalid)
                with self.assertRaisesRegex(ValueError, "outer|tolerance|容差"):
                    load_profile(self.path, "warehouse_raw_1")


class LocalizationTests(ProfileTestCase):
    def test_current_photo_translation_is_applied(self):
        result_a = locate_pixel(
            self.profile, [400, 0, 350, 165, -12, 35], [320, 240]
        )
        result_b = locate_pixel(
            self.profile, [460, 30, 350, 165, -12, 35], [320, 240]
        )
        np.testing.assert_allclose(
            np.array(result_b["base_xy_mm"]) - result_a["base_xy_mm"],
            [60, 30],
            atol=1e-8,
        )
        self.assertFalse(result_b["motion_authorized"])

    def test_output_contract_and_units(self):
        result = locate_pixel(
            self.profile, [400, 0, 350, 165, -12, 35], [330, 250]
        )
        self.assertEqual(set(result), {
            "profile_id", "image_size", "schema_version", "base_xy_mm",
            "diagnostic_plane_z_mm",
            "base_point_mm", "pixel", "motion_authorized",
        })
        local_point = np.array([1.0, 1.0, -100.01])
        expected_base = (
            rotation_from_rpy([165, -12, 35]) @ local_point
            + np.array([400.0, 0.0, 350.0])
        )
        self.assertEqual(result["profile_id"], "warehouse_raw_1")
        self.assertEqual(result["image_size"], [640, 480])
        self.assertEqual(result["schema_version"], 1)
        self.assertEqual(result["pixel"], [330.0, 250.0])
        self.assertAlmostEqual(result["diagnostic_plane_z_mm"], -100.01)
        self.assertNotAlmostEqual(
            result["diagnostic_plane_z_mm"], expected_base[2]
        )
        np.testing.assert_allclose(result["base_point_mm"], expected_base, atol=1e-9)
        np.testing.assert_allclose(result["base_xy_mm"], expected_base[:2], atol=1e-9)
        self.assertIs(result["motion_authorized"], False)

    def test_rpy_check_uses_rotation_angle_and_inclusive_tolerance(self):
        equivalent = copy.deepcopy(self.profile)
        result = locate_pixel(
            equivalent, [400, 0, 350, 525, -12, 35], [320, 240]
        )
        self.assertIs(result["motion_authorized"], False)

        boundary = copy.deepcopy(self.profile)
        boundary["reference_photo_pose"] = [400, 0, 350, 0, 0, 0]
        result = locate_pixel(boundary, [400, 0, 350, 0, 0, 0.5], [320, 240])
        self.assertIs(result["motion_authorized"], False)
        with self.assertRaisesRegex(ValueError, "RPY|姿态"):
            locate_pixel(boundary, [400, 0, 350, 0, 0, 0.51], [320, 240])

    def test_out_of_bounds_pixels_are_rejected(self):
        for pixel in ([-0.001, 240], [640, 240], [320, -0.001], [320, 480]):
            with self.subTest(pixel=pixel):
                with self.assertRaisesRegex(ValueError, "pixel|像素|bounds|范围"):
                    locate_pixel(
                        self.profile, self.profile["reference_photo_pose"], pixel
                    )

    def test_invalid_pixel_shapes_and_non_finite_values_are_rejected(self):
        for pixel in ([320], [320, 240, 0], [np.nan, 240], [320, np.inf]):
            with self.subTest(pixel=pixel):
                with self.assertRaisesRegex(ValueError, "pixel|像素|finite"):
                    locate_pixel(
                        self.profile, self.profile["reference_photo_pose"], pixel
                    )

    def test_non_finite_and_wrong_shape_current_pose_are_rejected(self):
        for pose in ([400, 0, 350, 165, -12],
                     [np.nan, 0, 350, 165, -12, 35],
                     [400, 0, 350, 165, np.inf, 35]):
            with self.subTest(pose=pose):
                with self.assertRaisesRegex(ValueError, "pose|位姿|finite"):
                    locate_pixel(self.profile, pose, [320, 240])


class CommandLineAndSafetyTests(ProfileTestCase):
    def test_offline_cli_needs_no_fairino_and_prints_json_then_warning(self):
        blocker = Path(self.temporary_directory.name) / "fairino.py"
        blocker.write_text(
            "raise AssertionError('hardware import attempted in offline mode')\n",
            encoding="utf-8",
        )
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join([
            str(self.temporary_directory.name), str(SCRIPT_DIR)
        ])
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        completed = subprocess.run(
            [
                sys.executable, str(SCRIPT_DIR / "plane_locate.py"),
                "--profile", str(self.path),
                "--profile-id", "warehouse_raw_1",
                "--pixel", "320", "240",
                "--tcp", "400", "0", "350", "165", "-12", "35",
            ],
            cwd=SCRIPT_DIR,
            env=environment,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=15,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        lines = completed.stdout.strip().splitlines()
        self.assertEqual(len(lines), 2, completed.stdout)
        result = json.loads(lines[0])
        self.assertIs(result["motion_authorized"], False)
        self.assertEqual(result["image_size"], [640, 480])
        self.assertEqual(result["schema_version"], 1)
        self.assertEqual(result["diagnostic_plane_z_mm"], -100.0)
        self.assertEqual(lines[-1], WARNING)

    def test_hardware_source_is_read_only_and_scipy_is_absent(self):
        source = (SCRIPT_DIR / "plane_locate.py").read_text(encoding="utf-8")
        self.assertNotIn("scipy", source.lower())
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
        for forbidden in ("MoveL(", "MoveJ(", "MoveGripper(", "ActGripper(",
                          "StartMove(", "nav_to"):
            self.assertNotIn(forbidden, source)

    def test_profile_registry_has_exact_versioned_entries(self):
        registry = json.loads(
            (SCRIPT_DIR / "plane_profiles.json").read_text(encoding="utf-8")
        )
        self.assertEqual(registry, {
            "warehouse_raw_1": {
                "result": "plane_results/warehouse_raw_1.json",
                "z_teaching_required": True,
                "standard_photo_description": (
                    "仓库第1行（warehouse_raw_1）使用仓库标准拍照 RPY，"
                    "与 warehouse_raw_2 共享同一套仓库 RPY。"
                ),
                "site_z_parameter": None,
            },
            "warehouse_raw_2": {
                "result": "plane_results/warehouse_raw_2.json",
                "z_teaching_required": True,
                "standard_photo_description": (
                    "仓库第2行（warehouse_raw_2）使用仓库标准拍照 RPY，"
                    "与 warehouse_raw_1 共享同一套仓库 RPY。"
                ),
                "site_z_parameter": None,
            },
            "machine_finished_pick": {
                "result": "plane_results/machine_finished_pick.json",
                "z_teaching_required": True,
                "standard_photo_description": (
                    "加工区（machine_finished_pick）使用独立的加工区标准拍照 RPY，"
                    "不与仓库 RPY 共用。"
                ),
                "site_z_parameter": None,
            },
        })


if __name__ == "__main__":
    unittest.main()
