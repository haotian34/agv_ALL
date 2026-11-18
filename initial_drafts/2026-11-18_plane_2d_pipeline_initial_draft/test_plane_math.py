import unittest

import numpy as np

import plane_math
from plane_math import (
    base_points_to_photo_local,
    photo_local_to_base,
    rpy_angle_difference_degrees,
    tcp_pose_to_matrix,
)


class PoseTests(unittest.TestCase):
    def test_round_trip_with_nonvertical_pose(self):
        photo = [410, 20, 350, 165, -12, 35]
        local = np.array([[20, -15, -95], [-30, 40, -90]], dtype=float)
        base = photo_local_to_base(photo, local)
        np.testing.assert_allclose(
            base_points_to_photo_local(photo, base), local, atol=1e-9
        )

    def test_angle_difference_handles_wrap(self):
        self.assertLess(
            rpy_angle_difference_degrees([179.8, 0, 0], [-179.8, 0, 0]), 0.41
        )

    def test_invalid_pose_is_rejected(self):
        with self.assertRaises(ValueError):
            tcp_pose_to_matrix([1, 2, 3])


class PlaneCalibrationTests(unittest.TestCase):
    def setUp(self):
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

        # Independently defined local-to-pixel mapping; calibration recovers its inverse.
        local_to_pixel = np.array([
            [2.4, 0.15, 640.0],
            [-0.08, 2.1, 480.0],
            [0.0004, -0.0003, 1.0],
        ])
        homogeneous = np.column_stack([local_xy, np.ones(9)])
        projected = homogeneous @ local_to_pixel.T
        self.pixels = projected[:, :2] / projected[:, 2:3]

        collinear_xy = np.column_stack([
            np.linspace(-40.0, 40.0, 9),
            np.linspace(-10.0, 10.0, 9),
        ])
        collinear_z = 0.02 * collinear_xy[:, 0] - 0.03 * collinear_xy[:, 1] - 100.0
        self.collinear_local = np.column_stack([collinear_xy, collinear_z])
        collinear_homogeneous = np.column_stack([collinear_xy, np.ones(9)])
        collinear_projected = collinear_homogeneous @ local_to_pixel.T
        self.collinear_pixels = (
            collinear_projected[:, :2] / collinear_projected[:, 2:3]
        )

    def test_recovers_mapping_plane_metrics_and_center_holdout(self):
        result = plane_math.solve_plane_calibration(
            self.local_points,
            self.pixels,
            center_tolerance_mm=0.01,
            outer_tolerance_mm=0.01,
        )

        np.testing.assert_allclose(
            plane_math.apply_homography(
                np.array(result["homography"]), self.pixels
            ),
            self.local_points[:, :2],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            result["plane_coefficients"], [0.02, -0.03, -100], atol=1e-9
        )
        center = result["center_validation"]
        self.assertEqual(center["fit_point_indices"], list(range(8)))
        self.assertLess(center["error_xy_mm"], 0.01)
        np.testing.assert_allclose(center["actual_local_xyz"], self.local_points[8])
        np.testing.assert_allclose(
            center["predicted_local_xyz"][:2], self.local_points[8, :2], atol=1e-7
        )
        self.assertLess(result["fit_metrics"]["rms_planar_mm"], 1e-7)
        self.assertLess(result["fit_metrics"]["max_planar_error_mm"], 1e-7)
        self.assertIn(result["fit_metrics"]["max_planar_error_index"], range(9))
        outer = result["outer_fit_metrics"]
        self.assertEqual(outer["fit_point_indices"], list(range(8)))
        self.assertLess(outer["rms_planar_mm"], 1e-7)
        self.assertLess(outer["max_planar_error_mm"], 1e-7)
        self.assertIn(outer["max_planar_error_index"], range(8))
        self.assertEqual(len(outer["point_residuals"]), 8)

    def test_collinear_points_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "rank|degenerate|共线"):
            plane_math.solve_plane_calibration(
                self.collinear_local,
                self.collinear_pixels,
                center_tolerance_mm=1.0,
                outer_tolerance_mm=1.0,
            )

    def test_center_outlier_is_rejected_before_final_fit(self):
        bad = self.local_points.copy()
        bad[8, :2] += [5.0, -4.0]
        with self.assertRaisesRegex(ValueError, "center|中心"):
            plane_math.solve_plane_calibration(
                bad,
                self.pixels,
                center_tolerance_mm=0.5,
                outer_tolerance_mm=1.0,
            )

    def test_outer_ring_residual_is_rejected_before_center_holdout(self):
        distorted_pixels = self.pixels.copy()
        for index, offset in (
            (0, [1.0, 0.5]),
            (2, [-1.0, 0.5]),
            (5, [1.0, -0.5]),
            (7, [-1.0, -0.5]),
        ):
            distorted_pixels[index] += offset

        outer_h = plane_math.fit_homography(
            distorted_pixels[:8], self.local_points[:8, :2]
        )
        center = plane_math.apply_homography(outer_h, distorted_pixels[8:9])[0]
        center_error = np.linalg.norm(center - self.local_points[8, :2])
        self.assertLess(center_error, 0.01)

        for outer_tolerance in (0.25, 0.20):
            with self.subTest(outer_tolerance_mm=outer_tolerance):
                with self.assertRaisesRegex(
                    ValueError, "outer|外圈|tolerance|残差"
                ):
                    plane_math.solve_plane_calibration(
                        self.local_points,
                        distorted_pixels,
                        center_tolerance_mm=0.01,
                        outer_tolerance_mm=outer_tolerance,
                    )

    def test_duplicate_points_are_rejected(self):
        pixels = self.pixels.copy()
        pixels[1] = pixels[0]
        with self.assertRaisesRegex(ValueError, "duplicate|重复"):
            plane_math.fit_homography(pixels[:8], self.local_points[:8, :2])

    def test_wrong_shapes_are_rejected(self):
        cases = [
            (self.pixels[:7], self.local_points[:8, :2]),
            (self.pixels[:8], self.local_points[:8]),
            (self.pixels[:8, 0], self.local_points[:8, :2]),
        ]
        for pixels, local_xy in cases:
            with self.subTest(pixels=pixels.shape, local_xy=local_xy.shape):
                with self.assertRaisesRegex(ValueError, "shape|match"):
                    plane_math.fit_homography(pixels, local_xy)

    def test_non_finite_points_are_rejected(self):
        pixels = self.pixels.copy()
        pixels[3, 0] = np.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            plane_math.fit_homography(pixels, self.local_points[:, :2])

        local_points = self.local_points.copy()
        local_points[4, 2] = np.inf
        with self.assertRaisesRegex(ValueError, "finite"):
            plane_math.solve_plane_calibration(
                local_points,
                self.pixels,
                center_tolerance_mm=1.0,
                outer_tolerance_mm=1.0,
            )

    def test_invalid_tolerances_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "tolerance|容差"):
            plane_math.solve_plane_calibration(
                self.local_points,
                self.pixels,
                center_tolerance_mm=-0.1,
                outer_tolerance_mm=1.0,
            )
        for outer_tolerance in (0.0, -0.1, np.nan, np.inf, True, "1.0", None):
            with self.subTest(outer_tolerance_mm=outer_tolerance):
                with self.assertRaisesRegex(ValueError, "outer|tolerance|外圈|容差"):
                    plane_math.solve_plane_calibration(
                        self.local_points,
                        self.pixels,
                        center_tolerance_mm=1.0,
                        outer_tolerance_mm=outer_tolerance,
                    )

    def test_projection_denominator_near_zero_is_rejected(self):
        homography = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
        ])
        with self.assertRaisesRegex(ValueError, "denominator|分母"):
            plane_math.apply_homography(homography, np.array([[0.0, 1.0]]))


if __name__ == "__main__":
    unittest.main()
