"""Append-only persistence for single-station plane calibration sessions."""

from datetime import datetime
import hashlib
import json
import math
from numbers import Integral, Real
import os
from pathlib import Path
from typing import Sequence


SUPPORTED_PROFILE_IDS = frozenset({
    "warehouse_raw_1",
    "warehouse_raw_2",
    "machine_finished_pick",
})

_IMAGE_SIZE = [640, 480]
_MAX_POINTS = 9
_EVENT_COLLECTIONS = {"pixel": "pixels", "taught_point": "taught_points"}


def _validate_profile_id(profile_id) -> None:
    if not isinstance(profile_id, str) or profile_id not in SUPPORTED_PROFILE_IDS:
        supported = ", ".join(sorted(SUPPORTED_PROFILE_IDS))
        raise ValueError(
            f"unsupported profile_id {profile_id!r}; supported profile IDs: {supported}"
        )

def _finite_vector(value, length: int, label: str) -> list[float]:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{label} must contain {length} finite numbers")
    try:
        values = list(value)
    except TypeError:
        raise ValueError(f"{label} must contain {length} finite numbers") from None
    if len(values) != length:
        raise ValueError(f"{label} must contain {length} finite numbers")
    if any(
        isinstance(item, bool) or not isinstance(item, Real) or not math.isfinite(item)
        for item in values
    ):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return [float(item) for item in values]


def _event_index(event: dict) -> int:
    index = event.get("index")
    if isinstance(index, bool) or not isinstance(index, Integral):
        raise ValueError("event index must be an integer from 0 through 8")
    index = int(index)
    if not 0 <= index < _MAX_POINTS:
        raise ValueError("event index must be from 0 through 8 (nine entries maximum)")
    return index


def _reference_image_fingerprint(image_path) -> tuple[int, str]:
    path = Path(image_path)
    if not path.is_file():
        raise ValueError(f"reference image missing or not a file: {path}")
    digest = hashlib.sha256()
    size = 0
    try:
        with path.open("rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                size += len(chunk)
                digest.update(chunk)
    except OSError as error:
        raise ValueError(f"reference image unreadable: {path}") from error
    return size, digest.hexdigest()


def _verify_reference_image(
    image_path: str, expected_size: int, expected_sha256: str
) -> None:
    actual_size, actual_sha256 = _reference_image_fingerprint(image_path)
    if actual_size != expected_size:
        raise ValueError(
            "reference image size changed: "
            f"expected {expected_size}, actual {actual_size}, path={image_path}"
        )
    if actual_sha256 != expected_sha256:
        raise ValueError(
            "reference image SHA-256 changed: "
            f"expected {expected_sha256}, actual {actual_sha256}, path={image_path}"
        )


def _validate_event_timestamp(event: dict) -> None:
    if "timestamp" not in event:
        return
    timestamp = event.get("timestamp")
    try:
        parsed = datetime.fromisoformat(timestamp)
        if parsed.tzinfo is None or parsed.utcoffset() is None:
            raise ValueError
    except (TypeError, ValueError):
        raise ValueError("event timestamp must be timezone-aware ISO format") from None


def _stamp_event(event):
    if not isinstance(event, dict):
        return event
    stamped = dict(event)
    if "timestamp" not in stamped:
        stamped["timestamp"] = datetime.now().astimezone().isoformat()
    return stamped


def _apply_event(state: dict, event) -> None:
    if not isinstance(event, dict):
        raise ValueError("session event must be a JSON object")
    _validate_event_timestamp(event)
    event_type = event.get("type")

    if event_type == "undo":
        target = event.get("target")
        if target not in _EVENT_COLLECTIONS:
            raise ValueError("undo target must be pixel or taught_point")
        active = state[_EVENT_COLLECTIONS[target]]
        if not active:
            raise ValueError(f"cannot undo an empty {target} list")
        active.pop()
        return

    if event_type not in _EVENT_COLLECTIONS:
        raise ValueError("event type must be pixel, taught_point, or undo")

    active = state[_EVENT_COLLECTIONS[event_type]]
    index = _event_index(event)
    if any(item["index"] == index for item in active):
        raise ValueError(f"duplicate active {event_type} index {index}")
    if index != len(active):
        raise ValueError(
            f"out-of-order {event_type} index {index}; expected {len(active)}"
        )

    if event_type == "pixel":
        field = "pixel"
        values = _finite_vector(event.get(field), 2, "pixel")
        if not (0 <= values[0] < _IMAGE_SIZE[0] and 0 <= values[1] < _IMAGE_SIZE[1]):
            raise ValueError("pixel coordinates must be within image bounds [640, 480]")
    else:
        field = "tcp_pose"
        values = _finite_vector(event.get(field), 6, "TCP pose")

    stored_event = dict(event)
    stored_event["index"] = index
    stored_event[field] = values
    active.append(stored_event)


def _start_session(start, expected_profile_id: str) -> dict:
    if not isinstance(start, dict) or start.get("type") != "session_start":
        raise ValueError("first event must be session_start")

    profile_id = start.get("profile_id")
    _validate_profile_id(profile_id)
    if profile_id != expected_profile_id:
        raise ValueError(
            f"profile mismatch: session has {profile_id!r}, expected {expected_profile_id!r}"
        )

    reference_pose = _finite_vector(
        start.get("reference_photo_pose"), 6, "reference photo TCP pose"
    )
    image_path = start.get("image_path")
    if not isinstance(image_path, str) or not image_path:
        raise ValueError("session_start image_path must be a non-empty string")
    image_file_size_bytes = start.get("image_file_size_bytes")
    if (
        isinstance(image_file_size_bytes, bool)
        or not isinstance(image_file_size_bytes, Integral)
        or int(image_file_size_bytes) <= 0
    ):
        raise ValueError("session_start image_file_size_bytes must be a positive integer")
    image_sha256 = start.get("image_sha256")
    if (
        not isinstance(image_sha256, str)
        or len(image_sha256) != 64
        or any(character not in "0123456789abcdefABCDEF" for character in image_sha256)
    ):
        raise ValueError("session_start image_sha256 must be 64 hexadecimal characters")
    if start.get("image_size") != _IMAGE_SIZE:
        raise ValueError("session_start image_size must be [640, 480]")
    if start.get("toolcoord") != "toolcoord2":
        raise ValueError("session_start toolcoord must be toolcoord2")

    timestamp = start.get("timestamp")
    try:
        parsed_timestamp = datetime.fromisoformat(timestamp)
        if parsed_timestamp.tzinfo is None or parsed_timestamp.utcoffset() is None:
            raise ValueError
    except (TypeError, ValueError):
        raise ValueError("session_start timestamp must be timezone-aware ISO format") from None

    image_sha256 = image_sha256.lower()
    _verify_reference_image(
        image_path, int(image_file_size_bytes), image_sha256
    )

    return {
        "profile_id": profile_id,
        "reference_photo_pose": reference_pose,
        "image_path": image_path,
        "image_file_size_bytes": int(image_file_size_bytes),
        "image_sha256": image_sha256,
        "pixels": [],
        "taught_points": [],
    }


def _write_json_line(path: Path, event: dict, mode: str) -> None:
    line = json.dumps(
        event, ensure_ascii=False, separators=(",", ":"), allow_nan=False
    ) + "\n"
    with path.open(mode, encoding="utf-8", newline="\n") as handle:
        handle.write(line)
        handle.flush()
        os.fsync(handle.fileno())


def create_session(
    path: Path,
    profile_id: str,
    reference_photo_pose: Sequence[float],
    image_path: Path,
) -> None:
    """Create a session exclusively; an existing history is never overwritten."""
    _validate_profile_id(profile_id)
    reference_pose = _finite_vector(
        reference_photo_pose, 6, "reference photo TCP pose"
    )
    image_file_size_bytes, image_sha256 = _reference_image_fingerprint(image_path)
    start = {
        "type": "session_start",
        "profile_id": profile_id,
        "reference_photo_pose": reference_pose,
        "image_path": str(image_path),
        "image_file_size_bytes": image_file_size_bytes,
        "image_sha256": image_sha256,
        "image_size": _IMAGE_SIZE,
        "toolcoord": "toolcoord2",
        "timestamp": datetime.now().astimezone().isoformat(),
    }
    _write_json_line(Path(path), start, "x")


def append_session_event(path: Path, expected_profile_id: str, event: dict) -> dict:
    """Validate and durably append one event without changing prior history."""
    path = Path(path)
    stamped_event = _stamp_event(event)
    state = load_session(path, expected_profile_id)
    _apply_event(state, stamped_event)
    _write_json_line(path, stamped_event, "a")
    return stamped_event


def load_session(path: Path, expected_profile_id: str) -> dict:
    """Replay and validate a complete or incomplete session in append order."""
    _validate_profile_id(expected_profile_id)
    path = Path(path)
    state = None
    try:
        with path.open("r", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                try:
                    event = json.loads(line)
                except (json.JSONDecodeError, UnicodeDecodeError) as error:
                    raise ValueError(f"malformed JSONL at line {line_number}") from error
                try:
                    if line_number == 1:
                        state = _start_session(event, expected_profile_id)
                    else:
                        _apply_event(state, event)
                except ValueError as error:
                    raise ValueError(f"invalid session line {line_number}: {error}") from error
    except UnicodeDecodeError as error:
        raise ValueError("malformed UTF-8 session JSONL") from error

    if state is None:
        raise ValueError("empty session; first event must be session_start")
    return state
