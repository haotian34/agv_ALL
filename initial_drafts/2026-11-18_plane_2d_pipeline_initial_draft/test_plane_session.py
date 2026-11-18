import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from datetime import datetime
from unittest import mock

import plane_session
from plane_session import append_session_event, create_session, load_session


class PlaneSessionTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary_directory.name) / "session.jsonl"
        self.photo = [410.0, 20.0, 350.0, 165.0, -12.0, 35.0]
        self.image = Path(self.temporary_directory.name) / "reference_photo.jpg"
        self.image_bytes = b"offline-reference-image"
        self.image.write_bytes(self.image_bytes)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def create(self):
        create_session(self.path, "warehouse_raw_1", self.photo, self.image)

    def append(self, event):
        append_session_event(self.path, "warehouse_raw_1", event)

    def assert_event_rejected(self, event, pattern="."):
        before = self.path.read_bytes()
        with self.assertRaisesRegex(ValueError, pattern):
            self.append(event)
        self.assertEqual(self.path.read_bytes(), before)

    def test_resume_preserves_order_and_profile(self):
        self.create()
        self.append({"type": "pixel", "index": 0, "pixel": [100, 120]})
        self.append({"type": "taught_point", "index": 0, "tcp_pose": self.photo})

        loaded = load_session(self.path, "warehouse_raw_1")

        self.assertEqual(loaded["pixels"][0]["pixel"], [100, 120])
        self.assertEqual(loaded["taught_points"][0]["index"], 0)
        self.assertEqual(
            set(loaded),
            {
                "profile_id", "reference_photo_pose", "image_path",
                "image_file_size_bytes", "image_sha256", "pixels",
                "taught_points",
            },
        )
        self.assertEqual(loaded["reference_photo_pose"], self.photo)
        self.assertEqual(loaded["image_path"], str(self.image))
        self.assertEqual(loaded["image_file_size_bytes"], len(self.image_bytes))
        self.assertEqual(
            loaded["image_sha256"], hashlib.sha256(self.image_bytes).hexdigest()
        )
        for event in loaded["pixels"] + loaded["taught_points"]:
            parsed = datetime.fromisoformat(event["timestamp"])
            self.assertIsNotNone(parsed.tzinfo)
            self.assertIsNotNone(parsed.utcoffset())

    def test_profile_mix_is_rejected(self):
        self.create()
        with self.assertRaisesRegex(ValueError, "profile|配置"):
            append_session_event(
                self.path,
                "warehouse_raw_2",
                {"type": "pixel", "index": 0, "pixel": [1, 2]},
            )
        with self.assertRaisesRegex(ValueError, "profile|配置"):
            load_session(self.path, "warehouse_raw_2")

    def test_supported_profile_ids_are_exact_and_accepted_at_public_boundaries(self):
        supported = {"warehouse_raw_1", "warehouse_raw_2", "machine_finished_pick"}
        self.assertEqual(set(plane_session.SUPPORTED_PROFILE_IDS), supported)
        self.assertEqual(len(plane_session.SUPPORTED_PROFILE_IDS), 3)

        for profile_id in sorted(supported):
            with self.subTest(profile_id=profile_id):
                path = self.path.with_name(f"{profile_id}.jsonl")
                create_session(path, profile_id, self.photo, self.image)
                append_session_event(
                    path, profile_id, {"type": "pixel", "index": 0, "pixel": [1, 2]}
                )
                self.assertEqual(load_session(path, profile_id)["profile_id"], profile_id)

    def test_create_rejects_every_unsupported_profile(self):
        for profile_id in ("unknown_profile", "warehouse_raw_3"):
            with self.subTest(profile_id=profile_id):
                path = self.path.with_name(f"unsupported-{profile_id}.jsonl")
                with self.assertRaisesRegex(ValueError, "unsupported|supported|支持"):
                    create_session(path, profile_id, self.photo, self.image)
                self.assertFalse(path.exists())

    def test_load_rejects_every_unsupported_expected_profile(self):
        self.create()
        for profile_id in ("unknown_profile", "warehouse_raw_3"):
            with self.subTest(profile_id=profile_id):
                with self.assertRaisesRegex(ValueError, "unsupported|supported|支持"):
                    load_session(self.path, profile_id)

    def test_append_rejects_every_unsupported_expected_profile_without_writing(self):
        self.create()
        before = self.path.read_bytes()
        for profile_id in ("unknown_profile", "warehouse_raw_3"):
            with self.subTest(profile_id=profile_id):
                with self.assertRaisesRegex(ValueError, "unsupported|supported|支持"):
                    append_session_event(
                        self.path,
                        profile_id,
                        {"type": "pixel", "index": 0, "pixel": [1, 2]},
                    )
                self.assertEqual(self.path.read_bytes(), before)

    def test_load_rejects_unsupported_stored_profile(self):
        start = {
            "type": "session_start",
            "profile_id": "unknown_profile",
            "reference_photo_pose": self.photo,
            "image_path": str(self.image),
            "image_file_size_bytes": len(self.image_bytes),
            "image_sha256": hashlib.sha256(self.image_bytes).hexdigest(),
            "image_size": [640, 480],
            "toolcoord": "toolcoord2",
            "timestamp": "2026-11-18T10:00:00+08:00",
        }
        self.path.write_text(json.dumps(start) + "\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "unsupported|supported|支持"):
            load_session(self.path, "warehouse_raw_1")
    def test_session_start_has_acquisition_metadata(self):
        self.create()

        lines = self.path.read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(lines), 1)
        start = json.loads(lines[0])
        self.assertEqual(start["type"], "session_start")
        self.assertEqual(start["profile_id"], "warehouse_raw_1")
        self.assertEqual(start["reference_photo_pose"], self.photo)
        self.assertEqual(start["image_path"], str(self.image))
        self.assertEqual(start["image_file_size_bytes"], len(self.image_bytes))
        self.assertEqual(
            start["image_sha256"], hashlib.sha256(self.image_bytes).hexdigest()
        )
        self.assertEqual(start["image_size"], [640, 480])
        self.assertEqual(start["toolcoord"], "toolcoord2")
        self.assertIsNotNone(datetime.fromisoformat(start["timestamp"]).tzinfo)

    def test_create_never_overwrites_existing_session(self):
        self.create()
        self.append({"type": "pixel", "index": 0, "pixel": [100, 120]})
        before = self.path.read_bytes()

        with self.assertRaises(FileExistsError):
            self.create()

        self.assertEqual(self.path.read_bytes(), before)

    def test_undo_is_appended_and_allows_replacement(self):
        self.create()
        self.append({"type": "pixel", "index": 0, "pixel": [100, 120]})
        self.append({"type": "undo", "target": "pixel"})
        self.append({"type": "pixel", "index": 0, "pixel": [200, 220]})

        loaded = load_session(self.path, "warehouse_raw_1")

        self.assertEqual(len(loaded["pixels"]), 1)
        self.assertEqual(loaded["pixels"][0]["pixel"], [200, 220])
        events = [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual([event["type"] for event in events], [
            "session_start", "pixel", "undo", "pixel",
        ])
        self.assertEqual(events[1]["pixel"], [100, 120])
        for event in events[1:]:
            parsed = datetime.fromisoformat(event["timestamp"])
            self.assertIsNotNone(parsed.tzinfo)
            self.assertIsNotNone(parsed.utcoffset())

    def test_undo_removes_only_most_recent_active_taught_point(self):
        self.create()
        self.append({"type": "taught_point", "index": 0, "tcp_pose": self.photo})
        changed_pose = [value + 1 for value in self.photo]
        self.append({"type": "taught_point", "index": 1, "tcp_pose": changed_pose})
        self.append({"type": "undo", "target": "taught_point"})

        loaded = load_session(self.path, "warehouse_raw_1")

        self.assertEqual(len(loaded["taught_points"]), 1)
        self.assertEqual(loaded["taught_points"][0]["index"], 0)
        self.append({"type": "taught_point", "index": 1, "tcp_pose": changed_pose})
        self.assertEqual(load_session(self.path, "warehouse_raw_1")["taught_points"][1]["tcp_pose"], changed_pose)

    def test_active_duplicate_indices_are_rejected(self):
        cases = [
            ({"type": "pixel", "index": 0, "pixel": [100, 120]}, "pixel"),
            ({"type": "taught_point", "index": 0, "tcp_pose": self.photo}, "taught_point"),
        ]
        for event, target in cases:
            with self.subTest(target=target):
                self.create()
                self.append(event)
                self.assert_event_rejected(event, "duplicate|重复")
                self.path.unlink()

    def test_out_of_order_indices_are_rejected(self):
        cases = [
            {"type": "pixel", "index": 1, "pixel": [100, 120]},
            {"type": "taught_point", "index": 1, "tcp_pose": self.photo},
        ]
        for event in cases:
            with self.subTest(event_type=event["type"]):
                self.create()
                self.assert_event_rejected(event, "order|顺序|index")
                self.path.unlink()

    def test_invalid_event_type_and_undo_target_are_rejected(self):
        self.create()
        self.assert_event_rejected({"type": "session_start"}, "type|事件")
        self.assert_event_rejected({"type": "unknown"}, "type|事件")
        self.assert_event_rejected({"type": "undo", "target": "unknown"}, "target|undo")
        self.assert_event_rejected([], "event|事件")

    def test_malformed_jsonl_is_rejected(self):
        self.create()
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write('{"type":"pixel",broken}\n')

        with self.assertRaisesRegex(ValueError, "malformed|JSON|line"):
            load_session(self.path, "warehouse_raw_1")

    def test_truncated_final_jsonl_record_without_newline_is_rejected(self):
        self.create()
        self.append({"type": "pixel", "index": 0, "pixel": [100, 120]})
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write('{"type":"taught_point","index":0,"tcp_pose":[1,2')

        with self.assertRaisesRegex(ValueError, "malformed|JSON|line"):
            load_session(self.path, "warehouse_raw_1")
    def test_first_event_must_be_valid_session_start(self):
        self.path.write_text('{"type":"pixel","index":0,"pixel":[1,2]}\n', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "session_start|first"):
            load_session(self.path, "warehouse_raw_1")

        self.path.write_text("", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "session_start|first|empty"):
            load_session(self.path, "warehouse_raw_1")

    def test_nine_entries_are_allowed_and_tenth_is_rejected(self):
        self.create()
        for index in range(9):
            self.append({"type": "pixel", "index": index, "pixel": [index, index]})
            self.append({"type": "taught_point", "index": index, "tcp_pose": self.photo})
        loaded = load_session(self.path, "warehouse_raw_1")
        self.assertEqual(len(loaded["pixels"]), 9)
        self.assertEqual(len(loaded["taught_points"]), 9)

        self.assert_event_rejected(
            {"type": "pixel", "index": 9, "pixel": [9, 9]}, "index|nine|9"
        )
        self.assert_event_rejected(
            {"type": "taught_point", "index": 9, "tcp_pose": self.photo}, "index|nine|9"
        )

    def test_pixel_shape_finiteness_and_image_bounds_are_validated(self):
        invalid_pixels = [
            [-1, 0], [640, 0], [0, 480], [0, -1], [1], [1, 2, 3],
            [float("nan"), 0], [0, float("inf")], [True, 0], ["1", 2],
        ]
        for pixel in invalid_pixels:
            with self.subTest(pixel=pixel):
                self.create()
                self.assert_event_rejected(
                    {"type": "pixel", "index": 0, "pixel": pixel}, "pixel|bound|finite|shape"
                )
                self.path.unlink()

    def test_tcp_pose_must_have_six_finite_values(self):
        invalid_poses = [
            [1, 2, 3, 4, 5], [1, 2, 3, 4, 5, 6, 7],
            [1, 2, 3, 4, 5, float("nan")],
            [1, 2, 3, 4, 5, float("inf")],
            [1, 2, 3, 4, 5, True], [1, 2, 3, 4, 5, "6"],
        ]
        for pose in invalid_poses:
            with self.subTest(pose=pose):
                self.create()
                self.assert_event_rejected(
                    {"type": "taught_point", "index": 0, "tcp_pose": pose},
                    "TCP|tcp|pose|finite|six|6",
                )
                self.path.unlink()

    def test_invalid_reference_pose_is_rejected_without_creating_file(self):
        with self.assertRaisesRegex(ValueError, "pose|TCP|finite|six|6"):
            create_session(self.path, "warehouse_raw_1", self.photo[:5], self.image)
        self.assertFalse(self.path.exists())

    def test_create_rejects_missing_reference_image_without_creating_session(self):
        self.image.unlink()
        with self.assertRaisesRegex(ValueError, "image|reference|missing|不存在"):
            self.create()
        self.assertFalse(self.path.exists())

    def test_load_and_append_reject_missing_or_changed_reference_image(self):
        for change in ("missing", "changed"):
            with self.subTest(change=change):
                self.path.unlink(missing_ok=True)
                self.image.write_bytes(self.image_bytes)
                self.create()
                if change == "missing":
                    self.image.unlink()
                else:
                    changed = bytearray(self.image_bytes)
                    changed[0] ^= 0x01
                    self.image.write_bytes(bytes(changed))
                with self.assertRaisesRegex(ValueError, "image|reference|missing|changed|不存在|变化"):
                    load_session(self.path, "warehouse_raw_1")
                with self.assertRaisesRegex(ValueError, "image|reference|missing|changed|不存在|变化"):
                    self.append({"type": "pixel", "index": 0, "pixel": [100, 120]})
                self.path.unlink()
                self.image.write_bytes(self.image_bytes)

    def test_undo_of_empty_list_is_rejected(self):
        for target in ("pixel", "taught_point"):
            with self.subTest(target=target):
                self.create()
                self.assert_event_rejected({"type": "undo", "target": target}, "empty|undo")
                self.path.unlink()

    def test_incomplete_session_remains_loadable(self):
        self.create()
        self.append({"type": "pixel", "index": 0, "pixel": [100, 120]})

        loaded = load_session(self.path, "warehouse_raw_1")

        self.assertEqual(len(loaded["pixels"]), 1)
        self.assertEqual(loaded["taught_points"], [])

    def test_append_flushes_before_fsync(self):
        self.create()
        event = {"type": "pixel", "index": 0, "pixel": [100, 120]}

        def check_flushed(file_descriptor):
            self.assertIsInstance(file_descriptor, int)
            persisted = json.loads(
                self.path.read_text(encoding="utf-8").splitlines()[-1]
            )
            self.assertEqual(persisted["type"], event["type"])
            self.assertEqual(persisted["index"], event["index"])
            self.assertEqual(persisted["pixel"], event["pixel"])
            parsed = datetime.fromisoformat(persisted["timestamp"])
            self.assertIsNotNone(parsed.tzinfo)
            self.assertIsNotNone(parsed.utcoffset())

        with mock.patch("plane_session.os.fsync", side_effect=check_flushed) as fsync:
            self.append(event)

        fsync.assert_called_once()


if __name__ == "__main__":
    unittest.main()
