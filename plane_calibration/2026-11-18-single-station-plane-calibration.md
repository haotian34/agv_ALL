# Single-Station Three-Plane Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build three crash-recoverable A4 plane calibrations and a read-only pixel-to-current-base-XY validation tool without issuing robot, gripper, or AGV motion.

**Architecture:** A pure NumPy math module owns pose conversion, homography fitting, local-plane fitting, and runtime localization. Device scripts only acquire color images and read Fairino TCP poses; JSONL sessions preserve every acquisition step, while versioned JSON profiles separate the two warehouse planes from the machining plane.

**Tech Stack:** Python 3.11 on the vehicle, NumPy, OpenCV, pyrealsense2, Fairino Python SDK, Python `unittest`; no SciPy dependency in the new modules.

**Spec:** `docs/superpowers/specs/2026-11-18-single-station-plane-calibration-design.md`

## Global Constraints

- All acquisition and validation scripts use `toolcoord2` and never command robot, gripper, or AGV motion.
- Exactly three profile IDs are supported initially: `warehouse_raw_1`, `warehouse_raw_2`, `machine_finished_pick`.
- The two warehouse profiles use the same standard photo RPY; the machining profile uses a different RPY. No profile is selected automatically from height.
- Every plane uses nine numbered points: indices 0–7 are the outer fit points and index 8 is the independent center check.
- Calibration outputs relative geometry in the reference photo TCP frame, not reusable absolute warehouse coordinates.
- Runtime diagnostic Z never authorizes motion and never replaces site-specific taught Z.
- Existing `handeye_*` and `grasp_*` files are not modified.
- Images are fixed at 640×480 for the initial profiles. Pixel coordinates use zero-based `(u,v)`.
- New math modules depend on NumPy only and do not import device SDKs.
- The workspace is not a Git repository. Do not initialize one; after each task, run its verification and record completion instead of committing.

---

### Task 1: Fairino Pose Conversion and Rotation Checks

**Files:**
- Create: `现场调试脚本/plane_math.py`
- Create: `现场调试脚本/test_plane_math.py`

**Interfaces:**
- Produces: `tcp_pose_to_matrix(pose: Sequence[float]) -> np.ndarray`
- Produces: `rpy_angle_difference_degrees(a: Sequence[float], b: Sequence[float]) -> float`
- Produces: `base_points_to_photo_local(photo_pose: Sequence[float], base_points: np.ndarray) -> np.ndarray`
- Produces: `photo_local_to_base(photo_pose: Sequence[float], local_points: np.ndarray) -> np.ndarray`

- [x] **Step 1: Write failing pose tests**

```python
import unittest
import numpy as np
from plane_math import (
    tcp_pose_to_matrix, rpy_angle_difference_degrees,
    base_points_to_photo_local, photo_local_to_base,
)

class PoseTests(unittest.TestCase):
    def test_round_trip_with_nonvertical_pose(self):
        photo = [410, 20, 350, 165, -12, 35]
        local = np.array([[20, -15, -95], [-30, 40, -90]], dtype=float)
        base = photo_local_to_base(photo, local)
        np.testing.assert_allclose(base_points_to_photo_local(photo, base), local, atol=1e-9)

    def test_angle_difference_handles_wrap(self):
        self.assertLess(rpy_angle_difference_degrees([179.8, 0, 0], [-179.8, 0, 0]), 0.41)

    def test_invalid_pose_is_rejected(self):
        with self.assertRaises(ValueError):
            tcp_pose_to_matrix([1, 2, 3])
```

- [x] **Step 2: Run the tests and confirm the import fails**

Run from `现场调试脚本`: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_math.py -v`
Expected: FAIL because `plane_math` does not exist.

- [x] **Step 3: Implement the four pure NumPy functions**

Use `R = Rz(rz) @ Ry(ry) @ Rx(rx)` exactly as in `handeye_solve.py`. Implement rotation difference as `acos(clamp((trace(Ra.T @ Rb)-1)/2,-1,1))`, convert to degrees, and validate finite shapes. The coordinate conversions are:

```python
rotation = transform[:3, :3]
translation = transform[:3, 3]
local = (base_points - translation) @ rotation  # row-vector form of R.T @ (p-t)
base = local_points @ rotation.T + translation
```

- [x] **Step 4: Run the tests and require all three to pass**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_math.py -v`
Expected: `OK` with 3 passing tests.

- [x] **Step 5: Record the checkpoint**

Check off Task 1 only after the test output shows 3 passing tests. Do not connect hardware in this task.

### Task 2: Nine-Point Homography, Local Plane, and Center Holdout

**Files:**
- Modify: `现场调试脚本/plane_math.py`
- Modify: `现场调试脚本/test_plane_math.py`

**Interfaces:**
- Consumes: Task 1 `base_points_to_photo_local`.
- Produces: `fit_homography(pixels: np.ndarray, local_xy: np.ndarray) -> np.ndarray`
- Produces: `apply_homography(homography: np.ndarray, pixels: np.ndarray) -> np.ndarray`
- Produces: `solve_plane_calibration(local_points: np.ndarray, pixels: np.ndarray, center_tolerance_mm: float) -> dict`
- Result keys: `homography`, `plane_coefficients`, `fit_metrics`, `center_validation`.

- [x] **Step 1: Add failing synthetic calibration tests**

Create nine pixels from a known 3×3 grid and known local `(q_x,q_y)` values using a nonsingular homography. Set `q_z = 0.02*q_x - 0.03*q_y - 100`. Include tests equivalent to:

```python
def test_recovers_mapping_and_center_holdout(self):
    result = solve_plane_calibration(self.local_points, self.pixels, center_tolerance_mm=0.01)
    np.testing.assert_allclose(apply_homography(np.array(result["homography"]), self.pixels), self.local_points[:, :2], atol=1e-7)
    np.testing.assert_allclose(result["plane_coefficients"], [0.02, -0.03, -100], atol=1e-9)
    self.assertLess(result["center_validation"]["error_xy_mm"], 0.01)

def test_collinear_points_are_rejected(self):
    with self.assertRaisesRegex(ValueError, "rank|degenerate|共线"):
        solve_plane_calibration(self.collinear_local, self.collinear_pixels, 1.0)

def test_center_outlier_is_rejected_before_final_fit(self):
    bad = self.local_points.copy(); bad[8, :2] += [5.0, -4.0]
    with self.assertRaisesRegex(ValueError, "center|中心"):
        solve_plane_calibration(bad, self.pixels, center_tolerance_mm=0.5)
```

Also add duplicate-point, wrong-shape, non-finite, and negative-tolerance rejection tests.

- [x] **Step 2: Run the new tests and verify missing-function failure**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_math.py -v`
Expected: new tests FAIL with missing `solve_plane_calibration` or `fit_homography`; Task 1 tests remain green.

- [x] **Step 3: Implement normalized DLT and plane fitting**

`fit_homography` must accept exactly matching `(N,2)` arrays with `N>=4`, reject duplicate/degenerate input, normalize each coordinate set before DLT, denormalize `H`, and normalize `H[2,2]` to 1. `apply_homography` must reject projected homogeneous denominators near zero.

`solve_plane_calibration` must:

```python
outer_h = fit_homography(pixels[:8], local_points[:8, :2])
center_xy = apply_homography(outer_h, pixels[8:9])[0]
center_error = center_xy - local_points[8, :2]
if np.linalg.norm(center_error) > center_tolerance_mm:
    raise ValueError(f"中心独立检查失败: {np.linalg.norm(center_error):.3f} mm")
final_h = fit_homography(pixels, local_points[:, :2])
a, b, c = np.linalg.lstsq(
    np.column_stack([local_points[:, :2], np.ones(9)]),
    local_points[:, 2], rcond=None,
)[0]
```

Fit metrics must report per-axis XY RMS, planar RMS, maximum planar error, and maximum point index. Center validation must explicitly store `fit_point_indices=list(range(8))`, predicted/actual local XYZ, and XY error.

- [x] **Step 4: Run all math tests**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_math.py -v`
Expected: all pose, valid calibration, and rejection tests pass.

- [x] **Step 5: Record the checkpoint**

Check off Task 2 only after inspecting that the center error comes from `outer_h`, not the final nine-point fit.

### Task 3: Crash-Safe Acquisition Sessions

**Files:**
- Create: `现场调试脚本/plane_session.py`
- Create: `现场调试脚本/test_plane_session.py`

**Interfaces:**
- Produces: `create_session(path: Path, profile_id: str, reference_photo_pose: Sequence[float], image_path: Path) -> None`
- Produces: `append_session_event(path: Path, expected_profile_id: str, event: dict) -> None`
- Produces: `load_session(path: Path, expected_profile_id: str) -> dict`
- Session result keys: `profile_id`, `reference_photo_pose`, `image_path`, `pixels`, `taught_points`.

- [x] **Step 1: Write failing persistence tests**

```python
def test_resume_preserves_order_and_profile(self):
    create_session(self.path, "warehouse_raw_1", self.photo, self.image)
    append_session_event(self.path, "warehouse_raw_1", {"type":"pixel", "index":0, "pixel":[100,120]})
    append_session_event(self.path, "warehouse_raw_1", {"type":"taught_point", "index":0, "tcp_pose":self.photo})
    loaded = load_session(self.path, "warehouse_raw_1")
    self.assertEqual(loaded["pixels"][0]["pixel"], [100,120])
    self.assertEqual(loaded["taught_points"][0]["index"], 0)

def test_profile_mix_is_rejected(self):
    create_session(self.path, "warehouse_raw_1", self.photo, self.image)
    with self.assertRaisesRegex(ValueError, "profile|配置"):
        append_session_event(self.path, "warehouse_raw_2", {"type":"pixel", "index":0, "pixel":[1,2]})
```

Add an undo test that appends pixel 0, appends `{"type":"undo","target":"pixel"}`, then adds a replacement pixel 0 and verifies that only the replacement remains active. Add tests for active duplicate indices, out-of-order indices, invalid event type, malformed JSONL, nine-pixel bounds, six-value finite TCP poses, and undo of an empty list.

- [x] **Step 2: Run session tests and confirm module failure**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_session.py -v`
Expected: FAIL because `plane_session` does not exist.

- [x] **Step 3: Implement append-only JSONL persistence**

The first line must be a `session_start` event containing profile ID, reference pose, image path, image size `[640,480]`, tool coordinate `toolcoord2`, and UTC/local ISO timestamp. `append_session_event` opens in append mode, writes one compact UTF-8 JSON object plus newline, flushes, and calls `os.fsync`.

`load_session` validates the first event, expected profile ID, event types, indices, pixel bounds, and TCP shape. Accepted acquisition events are `pixel`, `taught_point`, and `undo`; an undo event targets `pixel` or `taught_point` and removes only the most recent active entry of that type. A later replacement of the same index is valid after undo, while active duplicates and out-of-order additions are rejected. Pixel and taught-point arrays are reconstructed in index order; an incomplete session remains loadable for resume, but more than nine active entries are rejected.

- [x] **Step 4: Run all persistence tests**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_session.py -v`
Expected: `OK`, including malformed and mixed-profile rejection.

- [x] **Step 5: Record the checkpoint**

Check off Task 3 after deleting nothing: test temporary directories may clean themselves, but production session code must never overwrite session history.

### Task 4: Versioned Profiles and Static Runtime Localization

**Files:**
- Create: `现场调试脚本/plane_locate.py`
- Create: `现场调试脚本/test_plane_locate.py`
- Create: `现场调试脚本/plane_profiles.json`

**Interfaces:**
- Consumes: Task 1 `rpy_angle_difference_degrees`, `photo_local_to_base`; Task 2 `apply_homography`.
- Produces: `load_profile(path: Path, expected_profile_id: str) -> dict`
- Produces: `locate_pixel(profile: dict, current_photo_pose: Sequence[float], pixel: Sequence[float]) -> dict`
- Output keys: `profile_id`, `image_size`, `schema_version`, `base_xy_mm`, `diagnostic_plane_z_mm`, `base_point_mm`, `pixel`, `motion_authorized=False`. The design spec requires the two additive traceability fields.

- [x] **Step 1: Write failing runtime tests**

Build a synthetic profile with reference RPY `[165,-12,35]`, known H, plane `[0.02,-0.03,-100]`, and image size 640×480. Test:

```python
def test_current_photo_translation_is_applied(self):
    result_a = locate_pixel(self.profile, [400,0,350,165,-12,35], [320,240])
    result_b = locate_pixel(self.profile, [460,30,350,165,-12,35], [320,240])
    np.testing.assert_allclose(np.array(result_b["base_xy_mm"]) - result_a["base_xy_mm"], [60,30], atol=1e-8)
    self.assertFalse(result_b["motion_authorized"])

def test_wrong_profile_and_rpy_are_rejected(self):
    with self.assertRaisesRegex(ValueError, "配置"):
        load_profile(self.path, "machine_finished_pick")
    wrong = [400,0,350,170,-12,35]
    with self.assertRaisesRegex(ValueError, "RPY|姿态"):
        locate_pixel(self.profile, wrong, [320,240])
```

Add out-of-bounds pixels, wrong image size, wrong tool coordinate, invalid H shape, and non-finite pose tests.

- [x] **Step 2: Run runtime tests and confirm module failure**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_locate.py -v`
Expected: FAIL because `plane_locate` does not exist.

- [x] **Step 3: Implement profile validation and localization**

Validate profile ID, exact integer `schema_version==1` (booleans and missing/future versions rejected), `toolcoord=="toolcoord2"`, image size, finite reference pose, 3×3 H, three plane coefficients, and positive `rpy_tolerance_deg`. For a pixel:

```python
q_x, q_y = apply_homography(np.asarray(profile["homography"]), np.asarray([pixel], float))[0]
a, b, c = profile["plane_coefficients"]
q_z = a*q_x + b*q_y + c
base = photo_local_to_base(current_photo_pose, np.array([[q_x,q_y,q_z]]))[0]
```

Reject RPY mismatch before returning coordinates. Set `motion_authorized` to the literal boolean `False` in every returned result.

Add CLI arguments `--profile PATH --profile-id ID --pixel U V` and mutually exclusive `--tcp X Y Z RX RY RZ` versus read-only Fairino TCP acquisition. `--tcp` enables offline testing without hardware. Print JSON with UTF-8 and end with the warning `仅静态验证，诊断Z不得直接用于运动。`

Create `plane_profiles.json` with exactly:

```json
{
  "warehouse_raw_1": {"result": "plane_results/warehouse_raw_1.json", "z_teaching_required": true},
  "warehouse_raw_2": {"result": "plane_results/warehouse_raw_2.json", "z_teaching_required": true},
  "machine_finished_pick": {"result": "plane_results/machine_finished_pick.json", "z_teaching_required": true}
}
```

- [x] **Step 4: Run runtime and math tests**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_math.py test_plane_locate.py -v`
Expected: all tests pass; no device connection occurs when `--tcp` is supplied.

- [x] **Step 5: Record the checkpoint**

Check off Task 4 after running one CLI example with a synthetic profile and confirming `motion_authorized` is false.

### Task 5: Read-Only Interactive A4 Acquisition

**Files:**
- Create: `现场调试脚本/plane_calibrate.py`
- Create: `现场调试脚本/plane_report.py`
- Create: `现场调试脚本/test_plane_acquisition_safety.py`

**Interfaces:**
- Consumes: Task 2 `solve_plane_calibration`; Task 3 session APIs; Task 4 profile schema.
- Produces CLI: `plane_calibrate.py --profile-id ID --center-tolerance-mm FLOAT [--resume SESSION.jsonl]`
- Produces: saved 640×480 PNG, JSONL session, versioned JSON result, and UTF-8 text report.

- [x] **Step 1: Write failing acquisition safety tests**

Read `plane_calibrate.py` as text and assert that motion APIs are absent:

```python
for forbidden in ["MoveL(", "MoveJ(", "MoveGripper(", "ActGripper(", "StartMove(", "nav_to"]:
    self.assertNotIn(forbidden, source)
self.assertIn("GetActualTCPPose", source)
```

Add a schema test using a generated result fixture: result ID, tool coordinate, image size, reference pose, H, plane, fit metrics, center validation, and source session path must round-trip through `load_profile`.

- [x] **Step 2: Run safety tests and verify initial failure**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_acquisition_safety.py -v`
Expected: FAIL because acquisition/report files do not exist.

- [x] **Step 3: Implement the two-phase collector**

Phase A starts RealSense color only at 640×480, reads the reference TCP once, saves a PNG, and lets the user click points 0–8 on the static image with undo. Phase B keeps the saved image visible and reads one TCP per user confirmation for points 0–8. Print each point number and require ordered correspondence. Every click/read immediately calls `append_session_event`.

Use only:

```python
robot = Robot.RPC("192.168.192.160")
err, tcp = robot.GetActualTCPPose(0)
```

and `robot.CloseRPC()` in `finally`. Require an initial typed confirmation that toolcoord2 is active, the A4 target is fixed, a local person guards the physical emergency stop, and all motion will be manual through WebApp. Do not import navigation or gripper modules.

Resume loads existing pixels/taught points and continues at the first missing index. On completion call `solve_plane_calibration(local_points, pixels, center_tolerance_mm)`. A center failure writes no usable result and preserves the session.

- [x] **Step 4: Implement versioned result and text report writing**

Create parent directories `plane_sessions` and `plane_results`. If the same profile result already exists, first copy it to a timestamped backup beside the result; never delete or rewrite prior sessions. Write a temporary result file and atomically replace only the same profile current result. Include schema version, creation time, profile ID, tool coordinate, image size, reference photo pose, RPY tolerance, H, plane, fit metrics, center validation, source PNG, and source JSONL. The report shows per-point residuals and the independent center error and ends with `仅用于静态验证，禁止据此直接自动运动。`

- [x] **Step 5: Run all offline tests**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_math.py test_plane_session.py test_plane_locate.py test_plane_acquisition_safety.py -v`
Expected: all tests pass without RealSense or Fairino hardware.

- [x] **Step 6: Record the checkpoint**

Check off Task 5 only after source inspection confirms there are no motion calls and failed center validation cannot produce a result.

### Task 6: Field Guide and End-to-End Static Verification

**Files:**
- Create: `现场调试脚本/PLANE_CALIBRATION_GUIDE.md`
- Modify: `现场调试脚本/test_plane_acquisition_safety.py`

**Interfaces:**
- Consumes: all prior command-line and profile interfaces.
- Produces: operator instructions for three separate surrogate-plane sessions and read-only static checks.

- [x] **Step 1: Add an end-to-end offline fixture test**

Generate nine synthetic taught base points from known local points and a nonvertical reference pose, solve the calibration, write the profile, then locate the center using a current photo pose translated by `[60,30,0]`. Assert expected base XY and `motion_authorized is False`. Verify the three entries in `plane_profiles.json` have unique IDs and unique result paths.

- [x] **Step 2: Run the complete offline suite**

Run: `D:\anaconda3\envs\agv\python.exe -m unittest test_plane_math.py test_plane_session.py test_plane_locate.py test_plane_acquisition_safety.py -v`
Expected: all tests pass with no hardware.

- [x] **Step 3: Write the field guide with exact operator sequence**

The guide must instruct the user to:

1. Draw nine thin numbered crosses in a 3×3 layout on A4 paper fixed to a rigid board.
2. Keep toolcoord2, camera resolution, target board, and reference geometry unchanged throughout each session.
3. Use the same warehouse RPY for `warehouse_raw_1` and `warehouse_raw_2`; use the separate machining RPY for `machine_finished_pick`.
4. Capture pixels first, then manually teach TCP points in the same numbered order through WebApp with a local emergency-stop observer.
5. Run one profile at a time and verify the printed profile ID before every acquisition.
6. Review outer-fit residuals and center independent error; never delete a point merely to improve metrics.
7. Test additional independent points statically with `plane_locate.py --tcp ...` before any motion integration.
8. Sync new scripts and generated profiles explicitly to `C:\Users\Admin\Desktop\现场调试脚本\`; do not copy them over old handeye files.

Include exact vehicle commands using `D:\anaconda3\envs\agv\python.exe` and a separate section explaining that real site Z, warehouse finished-slot poses, vehicle poses, sensors, and machining handshake remain outside this tool’s authorization.

- [x] **Step 4: Perform final source and artifact verification**

Run the complete suite again and list newly created files. Confirm `handeye_data.jsonl`, `handeye_result.json`, `handeye_report.txt`, and `grasp_offset.json` have unchanged timestamps and hashes from before implementation. Confirm no result profile is marked motion-authorized.

- [x] **Step 5: Record completion and stop before hardware acquisition**

Report test counts, created files, unchanged legacy files, and the exact first field command. Do not claim any of the three physical planes has been calibrated until the user completes acquisition and independent static validation on the vehicle.
