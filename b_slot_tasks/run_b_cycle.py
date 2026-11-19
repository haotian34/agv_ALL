"""Independent Fairino B-slot Task 1 / Task 2 runner.

Point names (must exist in the WebApp):
  A_middle, B_1high, B_3, B_4low, B_2

Task 1 starts at A_middle holding a part, places it in B_4low, and returns A.
Task 2 starts at A_middle with an open empty gripper, picks from B_4low, and
returns A holding the part. Each invocation executes exactly one task and exits.

For future signal integration, import execute_task_1() or execute_task_2() from
this module. Manual use is: --task 1 or --task 2.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", line_buffering=True)


POINT_NAMES: Tuple[str, ...] = (
    "A_middle",
    "B_1high",
    "B_3",
    "B_4low",
    "B_2",
)
EXPECTED_TOOL = 2
EXPECTED_WOBJ = 0
DEFAULT_IP = "192.168.192.160"
DEFAULT_SPEED = 100.0
MAX_SPEED = 100.0
GLOBAL_SPEED = 30.0
MAX_GLOBAL_SPEED = 100.0
DEFAULT_EXTRACT_SPEED = 100.0
DEFAULT_EXTRACT_ACC = 100.0
DEFAULT_GRIP_FORCE = 70.0
POLL_S = 0.1
MOTION_START_TIMEOUT_S = 5.0
MOTION_TIMEOUT_S = 120.0
SETTLE_POLLS = 3
GRIPPER_STABLE_SAMPLES = 3
GRIPPER_POSITION_TOL = 3
GRIPPER_CURRENT_TOL = 5
GRIPPER_TIMEOUT_S = 30.0
START_GRIPPER_TIMEOUT_S = 2.0
GRIPPER_POLL_S = 0.2
OPEN_MIN_POSITION = 55
CLOSED_MAX_POSITION = 45
START_JOINT_TOL_DEG = 0.5
START_POSITION_TOL_MM = 1.0
START_ORIENTATION_TOL_DEG = 0.5
ENDPOINT_JOINT_TOL_DEG = 0.2
ENDPOINT_POSITION_TOL_MM = 0.2
ENDPOINT_ORIENTATION_TOL_DEG = 0.2
STATIONARY_TCP_SPEED = 1.0
STATIONARY_JOINT_SPEED = 0.2
ZERO6 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

TASK_INFO = {
    1: {
        "name": "任务1：生件放入生件槽",
        "start": "A_middle（已夹持生件）",
        "end": "A_middle（空夹爪）",
        "start_gripper": "close",
        "final_gripper": "open",
        "gripper_plan": "起点闭合夹持 → B_4low张开并检查机器人状态 → 终点张开",
    },
    2: {
        "name": "任务2：从生件槽取出工件",
        "start": "A_middle（空夹爪，夹爪张开）",
        "end": "A_middle（夹持生件）",
        "start_gripper": "open",
        "final_gripper": "close",
        "gripper_plan": "起点张开 → B_4low闭爪并检查闭合反馈 → 终点保持闭合",
    },
}


class CycleError(RuntimeError):
    pass


@dataclass(frozen=True)
class TeachingPoint:
    name: str
    tcp: Tuple[float, float, float, float, float, float]
    joints: Tuple[float, float, float, float, float, float]
    tool_id: int
    wobj_id: int
    speed: float
    acc: float
    extra_axes: Tuple[float, float, float, float]


@dataclass(frozen=True)
class Snapshot:
    tcp: Tuple[float, float, float, float, float, float]
    joints: Tuple[float, float, float, float, float, float]
    tool_id: int
    wobj_id: int
    robot_state: int
    robot_mode: int
    enabled: bool
    main_code: int
    sub_code: int
    emergency_stop: int
    safety_stop0: int
    safety_stop1: int
    motion_done: int
    queue_len: int
    gripper_fault: int
    gripper_active: int
    gripper_status: Optional[int]
    gripper_position: Optional[int]
    gripper_current: Optional[int]
    tcp_speed: Tuple[float, float, float, float, float, float]
    joint_speed: Tuple[float, float, float, float, float, float]
    collision_state: Optional[int]


@dataclass(frozen=True)
class GripperFeedback:
    fault: int
    active: int
    status: Optional[int]
    position: Optional[int]
    current: Optional[int]


def _number(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CycleError("%s is not numeric: %r" % (label, value)) from exc
    if not math.isfinite(result):
        raise CycleError("%s is not finite" % label)
    return result


def _six(values: Any, label: str) -> Tuple[float, float, float, float, float, float]:
    try:
        result = tuple(_number(value, label) for value in list(values))
    except TypeError as exc:
        raise CycleError("%s must contain six values" % label) from exc
    if len(result) != 6:
        raise CycleError("%s must contain six values" % label)
    return result  # type: ignore[return-value]


def _optional_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _normalize_sdk_result(result: Any) -> Tuple[int, Tuple[Any, ...]]:
    if isinstance(result, bool):
        return int(result), ()
    if isinstance(result, int):
        return result, ()
    if not isinstance(result, (tuple, list)) or not result:
        raise CycleError("SDK result must be an error code or a non-empty tuple/list")
    try:
        error = int(result[0])
    except (TypeError, ValueError) as exc:
        raise CycleError("SDK error code is not numeric") from exc
    if len(result) == 2 and isinstance(result[1], (tuple, list)):
        payload = tuple(result[1])
    else:
        payload = tuple(result[1:])
    return error, payload


class FairinoSession:
    def __init__(self, robot: Any):
        self.robot = robot

    def _call(self, name: str, *args: Any) -> Tuple[Any, ...]:
        method = getattr(self.robot, name, None)
        if not callable(method):
            raise CycleError("Fairino SDK is missing %s" % name)
        error, payload = _normalize_sdk_result(method(*args))
        if error != 0:
            raise CycleError("%s returned SDK error %s" % (name, error))
        return payload

    def set_global_speed(self, speed_percent: float) -> int:
        self._call("SetSpeed", int(round(float(speed_percent))))
        return 0

    def communication_ok(self) -> bool:
        payload = self._call("GetSDKComState")
        return bool(payload and int(payload[0]) == 0)

    def snapshot(self) -> Snapshot:
        payload = self._call("GetRobotRealTimeState")
        if not payload:
            raise CycleError("GetRobotRealTimeState returned no data")
        pkg = payload[0]
        return Snapshot(
            tcp=_six(getattr(pkg, "tl_cur_pos"), "tl_cur_pos"),
            joints=_six(getattr(pkg, "jt_cur_pos"), "jt_cur_pos"),
            tool_id=int(getattr(pkg, "tool", -1)),
            wobj_id=int(getattr(pkg, "user", -1)),
            robot_state=int(getattr(pkg, "robot_state", -1)),
            robot_mode=int(getattr(pkg, "robot_mode", -1)),
            enabled=int(getattr(pkg, "rbtEnableState", 0)) == 1,
            main_code=int(getattr(pkg, "main_code", -1)),
            sub_code=int(getattr(pkg, "sub_code", -1)),
            emergency_stop=int(getattr(pkg, "EmergencyStop", -1)),
            safety_stop0=int(getattr(pkg, "safety_stop0_state", -1)),
            safety_stop1=int(getattr(pkg, "safety_stop1_state", -1)),
            motion_done=int(getattr(pkg, "motion_done", -1)),
            queue_len=int(getattr(pkg, "mc_queue_len", -1)),
            gripper_fault=int(getattr(pkg, "gripper_fault", -1)),
            gripper_active=int(getattr(pkg, "gripper_active", 0)),
            gripper_status=_optional_int(getattr(pkg, "gripper_motiondone", None)),
            gripper_position=_optional_int(getattr(pkg, "gripper_position", None)),
            gripper_current=_optional_int(getattr(pkg, "gripper_current", None)),
            tcp_speed=_six(getattr(pkg, "actual_TCP_Speed"), "actual_TCP_Speed"),
            joint_speed=_six(getattr(pkg, "actual_qd"), "actual_qd"),
            collision_state=_optional_int(getattr(pkg, "collisionState", None)),
        )

    def read_points(self, names: Sequence[str]) -> Mapping[str, TeachingPoint]:
        result = {}
        for name in names:
            payload = self._call("GetRobotTeachingPoint", str(name))
            if len(payload) == 1 and isinstance(payload[0], str):
                values = payload[0].split(",")
            elif len(payload) == 1 and isinstance(payload[0], (tuple, list)):
                values = list(payload[0])
            else:
                values = list(payload)
            if len(values) != 20:
                raise CycleError("GetRobotTeachingPoint(%s) returned %d values, expected 20" % (name, len(values)))
            numeric = [_number(value, "point %s" % name) for value in values]
            tool = int(round(numeric[12]))
            wobj = int(round(numeric[13]))
            point = TeachingPoint(
                name=str(name),
                tcp=tuple(numeric[0:6]),  # type: ignore[arg-type]
                joints=tuple(numeric[6:12]),  # type: ignore[arg-type]
                tool_id=tool,
                wobj_id=wobj,
                speed=numeric[14],
                acc=numeric[15],
                extra_axes=tuple(numeric[16:20]),  # type: ignore[arg-type]
            )
            if point.tool_id != EXPECTED_TOOL:
                raise CycleError("%s uses tool %s, expected toolcoord2" % (name, point.tool_id))
            if point.wobj_id != EXPECTED_WOBJ:
                raise CycleError("%s uses work object %s, expected 0" % (name, point.wobj_id))
            if any(abs(value) > 0.001 for value in point.extra_axes):
                raise CycleError("%s external axes are not at zero" % name)
            result[name] = point
        return result

    def move_linear(self, point: TeachingPoint, speed_percent: float, acceleration_percent: float = 100.0) -> int:
        if point.tool_id != EXPECTED_TOOL or point.wobj_id != EXPECTED_WOBJ:
            raise CycleError("LIN target frame mismatch")
        self._call(
            "MoveL",
            list(point.tcp),
            point.tool_id,
            point.wobj_id,
            list(point.joints),
            float(speed_percent),
            0.0,
            100.0,
            0.0,
            0,
            list(point.extra_axes),
            0,
            0,
            list(ZERO6),
            float(acceleration_percent),
            -1,
            0,
            0,
            10,
        )
        return 0

    def gripper_feedback(self) -> GripperFeedback:
        active = self._call("GetGripperActivateStatus")
        done = self._call("GetGripperMotionDone")
        position = self._call("GetGripperCurPosition")
        current = self._call("GetGripperCurCurrent")
        for name, values in (
            ("GetGripperActivateStatus", active),
            ("GetGripperMotionDone", done),
            ("GetGripperCurPosition", position),
            ("GetGripperCurCurrent", current),
        ):
            if len(values) < 2:
                raise CycleError("%s returned too few values" % name)
        return GripperFeedback(
            fault=max(int(active[0]), int(done[0]), int(position[0]), int(current[0])),
            active=int(active[1]),
            status=_optional_int(done[1]),
            position=_optional_int(position[1]),
            current=_optional_int(current[1]),
        )

    def open_gripper(self) -> int:
        self._call("MoveGripper", 1, 60, 30, 20, 30000, 0, 0, 0, 0, 0)
        return 0

    def close_gripper(self, force_percent: float = DEFAULT_GRIP_FORCE) -> int:
        self._call("MoveGripper", 1, 0, 30, int(round(float(force_percent))), 30000, 0, 0, 0, 0, 0)
        return 0

    def stop_motion(self) -> int:
        self._call("StopMotion")
        return 0

    def close(self) -> None:
        close = getattr(self.robot, "CloseRPC", None)
        if callable(close):
            close()


def _angle_delta(a: float, b: float) -> float:
    return abs((b - a + 180.0) % 360.0 - 180.0)


def _pose_error(point: TeachingPoint, state: Snapshot) -> Tuple[float, float, float]:
    position = math.sqrt(sum((state.tcp[i] - point.tcp[i]) ** 2 for i in range(3)))
    orientation = max(_angle_delta(point.tcp[i], state.tcp[i]) for i in range(3, 6))
    joint = max(_angle_delta(point.joints[i], state.joints[i]) for i in range(6))
    return position, orientation, joint


def _common_faults(state: Snapshot) -> list[str]:
    reasons = []
    if state.tool_id != EXPECTED_TOOL:
        reasons.append("tool id is %s, expected 2" % state.tool_id)
    if state.wobj_id != EXPECTED_WOBJ:
        reasons.append("work object is %s, expected 0" % state.wobj_id)
    if not state.enabled:
        reasons.append("robot is not enabled")
    if state.main_code != 0 or state.sub_code != 0:
        reasons.append("robot alarm %s:%s" % (state.main_code, state.sub_code))
    if state.emergency_stop != 0:
        reasons.append("emergency stop is active")
    if state.safety_stop0 != 0 or state.safety_stop1 != 0:
        reasons.append("safety stop is active")
    if state.collision_state not in (None, 0):
        reasons.append("collision state is active")
    if state.gripper_status == 3:
        reasons.append("gripper reports object dropped")
    return reasons


def _stationary_faults(state: Snapshot) -> list[str]:
    reasons = []
    if state.robot_mode != 0:
        reasons.append("机器人不在自动模式")
    if state.robot_state != 1:
        reasons.append("robot is not stopped")
    if state.motion_done != 1:
        reasons.append("motion-done is not settled")
    if state.queue_len != 0:
        reasons.append("motion queue is not empty")
    if max(abs(v) for v in state.tcp_speed) > STATIONARY_TCP_SPEED:
        reasons.append("TCP speed is not stationary")
    if max(abs(v) for v in state.joint_speed) > STATIONARY_JOINT_SPEED:
        reasons.append("joint speed is not stationary")
    return reasons


def _require_stationary(
    session: Any,
    point: TeachingPoint,
    label: str,
    start: bool = False,
    state: Optional[Snapshot] = None,
) -> Snapshot:
    state = state or session.snapshot()
    reasons = _common_faults(state)
    if not session.communication_ok():
        reasons.append("SDK communication is not healthy")
    reasons.extend(_stationary_faults(state))
    position, orientation, joint = _pose_error(point, state)
    joint_tol = START_JOINT_TOL_DEG if start else ENDPOINT_JOINT_TOL_DEG
    position_tol = START_POSITION_TOL_MM if start else ENDPOINT_POSITION_TOL_MM
    orientation_tol = START_ORIENTATION_TOL_DEG if start else ENDPOINT_ORIENTATION_TOL_DEG
    if joint > joint_tol:
        reasons.append("%s joint error %.3f deg" % (label, joint))
    if position > position_tol:
        reasons.append("%s TCP error %.3f mm" % (label, position))
    if orientation > orientation_tol:
        reasons.append("%s orientation error %.3f deg" % (label, orientation))
    if reasons:
        raise CycleError("%s 检查失败：%s" % (label, "; ".join(reasons)))
    return state


def _wait_gripper(
    session: Any,
    mode: str,
    sleep_fn: Callable[[float], None],
    clock_fn: Callable[[], float],
) -> GripperFeedback:
    samples: list[GripperFeedback] = []
    deadline = clock_fn() + GRIPPER_TIMEOUT_S
    while True:
        sample = session.gripper_feedback()
        if sample.status == 3:
            raise CycleError("gripper reports object dropped")
        if sample.position is None or sample.current is None:
            raise CycleError("gripper position/current feedback is missing")
        if mode == "holding" and int(sample.position) > CLOSED_MAX_POSITION:
            raise CycleError(
                "gripper is no longer closed after lifting; workpiece may have dropped "
                "(position=%s current=%s)"
                % (sample.position, sample.current)
            )
        samples.append(sample)
        tail = samples[-GRIPPER_STABLE_SAMPLES:]
        if len(tail) == GRIPPER_STABLE_SAMPLES:
            positions = [int(item.position) for item in tail]
            currents = [int(item.current) for item in tail]
            stable = (
                max(positions) - min(positions) <= GRIPPER_POSITION_TOL
                and max(currents) - min(currents) <= GRIPPER_CURRENT_TOL
            )
            if mode == "open":
                position_ok = min(positions) >= OPEN_MIN_POSITION
            elif mode == "close":
                position_ok = max(positions) <= CLOSED_MAX_POSITION
            elif mode == "holding":
                position_ok = max(positions) <= CLOSED_MAX_POSITION
            else:
                raise CycleError("unknown gripper mode %s" % mode)
            if stable and position_ok:
                return tail[-1]
        if clock_fn() >= deadline:
            last = samples[-1] if samples else None
            detail = (
                "position=%s current=%s status=%s fault=%s active=%s"
                % (
                    getattr(last, "position", None),
                    getattr(last, "current", None),
                    getattr(last, "status", None),
                    getattr(last, "fault", None),
                    getattr(last, "active", None),
                )
            )
            raise CycleError(
                "gripper feedback did not become stable within %.0f s (%s)"
                % (GRIPPER_TIMEOUT_S, detail)
            )
        sleep_fn(GRIPPER_POLL_S)


def _check_start_gripper(
    session: Any,
    task_number: int,
    mode: str,
    sleep_fn: Callable[[float], None],
    clock_fn: Callable[[], float],
) -> GripperFeedback:
    """Fail immediately when the selected task does not match the start state."""
    samples: list[GripperFeedback] = []
    deadline = clock_fn() + START_GRIPPER_TIMEOUT_S
    while True:
        sample = session.gripper_feedback()
        if sample.status == 3:
            raise CycleError("夹爪报告工件掉落状态3")
        if sample.position is None or sample.current is None:
            raise CycleError("夹爪位置/电流反馈缺失")
        position = int(sample.position)
        if mode == "open":
            if position < OPEN_MIN_POSITION:
                raise CycleError(
                    "任务%d起点夹爪状态不符：需要张开空夹爪，实际位置=%d，疑似闭合或夹持中"
                    % (task_number, position)
                )
        elif mode in ("closed", "holding"):
            if position > CLOSED_MAX_POSITION:
                raise CycleError(
                    "任务%d起点夹爪状态不符：需要夹爪闭合，实际位置=%d，夹爪仍张开"
                    % (task_number, position)
                )
        else:
            raise CycleError("unknown start gripper mode %s" % mode)

        samples.append(sample)
        tail = samples[-GRIPPER_STABLE_SAMPLES:]
        if len(tail) == GRIPPER_STABLE_SAMPLES:
            positions = [int(item.position) for item in tail]
            currents = [int(item.current) for item in tail]
            stable = (
                max(positions) - min(positions) <= GRIPPER_POSITION_TOL
                and max(currents) - min(currents) <= GRIPPER_CURRENT_TOL
            )
            if stable:
                return tail[-1]
        if clock_fn() >= deadline:
            raise CycleError(
                "任务%d起点夹爪反馈在 %.1f 秒内未稳定"
                % (task_number, START_GRIPPER_TIMEOUT_S)
            )
        sleep_fn(GRIPPER_POLL_S)


def _move_linear_and_wait(
    session: Any,
    point: TeachingPoint,
    speed: float,
    sleep_fn: Callable[[float], None],
    clock_fn: Callable[[], float],
    acceleration: float = 100.0,
) -> Snapshot:
    before = session.snapshot()
    reasons = _common_faults(before)
    if reasons:
        raise CycleError("pre-move check failed: %s" % "; ".join(reasons))
    session.move_linear(point, speed, acceleration)
    started = clock_fn()
    seen_motion = False
    settled = 0
    last: Optional[Snapshot] = None
    while True:
        last = session.snapshot()
        reasons = _common_faults(last)
        if last.robot_mode != 0:
            reasons.append("robot is not in automatic mode")
        if not session.communication_ok():
            reasons.append("SDK communication is not healthy")
        if reasons:
            raise CycleError("motion safety check failed: %s" % "; ".join(reasons))

        moving = (
            last.robot_state != 1
            or last.motion_done != 1
            or last.queue_len != 0
            or max(abs(v) for v in last.tcp_speed) > STATIONARY_TCP_SPEED
            or max(abs(v) for v in last.joint_speed) > STATIONARY_JOINT_SPEED
        )
        position, orientation, joint = _pose_error(point, last)
        endpoint_ok = (
            position <= ENDPOINT_POSITION_TOL_MM
            and orientation <= ENDPOINT_ORIENTATION_TOL_DEG
            and joint <= ENDPOINT_JOINT_TOL_DEG
        )
        elapsed = clock_fn() - started
        if moving:
            seen_motion = True
            settled = 0
        elif seen_motion:
            settled += 1
        elif endpoint_ok and elapsed >= POLL_S:
            seen_motion = True
            settled = SETTLE_POLLS
        if seen_motion and endpoint_ok and settled >= SETTLE_POLLS:
            break
        if not seen_motion and elapsed > MOTION_START_TIMEOUT_S:
            raise CycleError("motion did not start within %.1f s" % MOTION_START_TIMEOUT_S)
        if elapsed > MOTION_TIMEOUT_S:
            raise CycleError("motion timeout after %.1f s" % MOTION_TIMEOUT_S)
        sleep_fn(POLL_S)
    if last is None:
        raise CycleError("no robot state returned after motion")
    return _require_stationary(session, point, "endpoint %s" % point.name)


def _move_path(
    session: Any,
    points: Mapping[str, TeachingPoint],
    names: Sequence[str],
    speed: float,
    log: dict[str, Any],
    sleep_fn: Callable[[float], None],
    clock_fn: Callable[[], float],
    output_fn: Callable[[str], None],
    acceleration: float = 100.0,
) -> None:
    for name in names:
        point = points[name]
        output_fn(
            "LIN 前往 %s | XYZ=(%.3f, %.3f, %.3f) | RPY=(%.3f, %.3f, %.3f) | 加速度=%.0f%%"
            % ((name,) + point.tcp + (acceleration,))
        )
        state = _move_linear_and_wait(session, points[name], speed, sleep_fn, clock_fn, acceleration)
        log["moves"].append({
            "point": name,
            "tcp": list(state.tcp),
            "joints": list(state.joints),
            "acceleration": float(acceleration),
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        })


def _write_log(log_dir: Path, log: Mapping[str, Any], task_number: int) -> Optional[str]:
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        filename = "b_task%d_%s.json" % (
            int(task_number),
            datetime.now().strftime("%Y%m%d_%H%M%S_%f"),
        )
        path = log_dir / filename
        path.write_text(json.dumps(log, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return str(path.resolve())
    except Exception as exc:
        print("日志写入失败：", exc)
        return None


def _task_info(task_number: int) -> Mapping[str, str]:
    try:
        number = int(task_number)
    except (TypeError, ValueError) as exc:
        raise CycleError("task number must be 1 or 2") from exc
    if number not in TASK_INFO:
        raise CycleError("task number must be 1 or 2")
    return TASK_INFO[number]


def _announce_task(task_number: int, output_fn: Callable[[str], None]) -> None:
    info = _task_info(task_number)
    output_fn("")
    output_fn(info["name"])
    output_fn("起点：%s" % info["start"])
    output_fn("终点：%s" % info["end"])
    output_fn("预期夹爪：%s" % info["gripper_plan"])
    output_fn("本次只执行任务%d，结束后停在 A_middle，不自动执行下一任务。" % task_number)


def _validate_task_parameters(
    speed: float,
    global_speed: float,
    extract_speed: float,
    extract_acceleration: float,
    grip_force: float,
) -> None:
    if not 0.0 < float(speed) <= MAX_SPEED:
        raise CycleError("speed must be in (0, %.0f] percent" % MAX_SPEED)
    if not 0.0 < float(global_speed) <= MAX_GLOBAL_SPEED:
        raise CycleError("global speed must be in (0, %.0f] percent" % MAX_GLOBAL_SPEED)
    if not 0.0 < float(extract_speed) <= 100.0:
        raise CycleError("extract speed must be in (0, 100] percent")
    if not 0.0 < float(extract_acceleration) <= 100.0:
        raise CycleError("extract acceleration must be in (0, 100] percent")
    if not 0.0 < float(grip_force) <= 100.0:
        raise CycleError("grip force must be in (0, 100] percent")


def run_task(
    session: Any,
    task_number: int,
    speed: float = DEFAULT_SPEED,
    global_speed: float = GLOBAL_SPEED,
    extract_speed: float = DEFAULT_EXTRACT_SPEED,
    extract_acceleration: float = DEFAULT_EXTRACT_ACC,
    grip_force: float = DEFAULT_GRIP_FORCE,
    output_fn: Callable[[str], None] = print,
    sleep_fn: Callable[[float], None] = time.sleep,
    clock_fn: Callable[[], float] = time.monotonic,
    log_dir: Optional[Path] = None,
) -> str:
    info = _task_info(task_number)
    task_number = int(task_number)
    _validate_task_parameters(
        speed, global_speed, extract_speed, extract_acceleration, grip_force
    )
    started = datetime.now().astimezone().isoformat(timespec="seconds")
    log: dict[str, Any] = {
        "task": task_number,
        "task_name": info["name"],
        "start_description": info["start"],
        "end_description": info["end"],
        "expected_gripper": info["gripper_plan"],
        "started_at": started,
        "speed": float(speed),
        "global_speed": float(global_speed),
        "extract_speed": float(extract_speed),
        "extract_acceleration": float(extract_acceleration),
        "grip_force": float(grip_force),
        "point_names": list(POINT_NAMES),
        "moves": [],
        "events": [],
    }
    try:
        _announce_task(task_number, output_fn)
        session.set_global_speed(global_speed)
        points = session.read_points(POINT_NAMES)
        missing = [name for name in POINT_NAMES if name not in points]
        if missing:
            raise CycleError("missing WebApp points: %s" % ", ".join(missing))
        log["points"] = {name: asdict(points[name]) for name in POINT_NAMES}

        initial_state = session.snapshot()
        log["initial_state"] = asdict(initial_state)
        start_state = _require_stationary(
            session, points["A_middle"], "A_middle 起点", start=True, state=initial_state
        )
        output_fn(
            "WebApp 点位和 A_middle 起点检查通过；全局速度 %.0f%%，LIN %.0f%%，"
            "首次抬升 LIN %.0f%%、加速度 %.0f%%，夹持力 %.0f%%。"
            % (global_speed, speed, extract_speed, extract_acceleration, grip_force)
        )
        output_fn(
            "tcp=%s joints=%s"
            % (
                tuple(round(v, 3) for v in start_state.tcp),
                tuple(round(v, 3) for v in start_state.joints),
            )
        )

        if task_number == 2:
            _check_start_gripper(session, 2, "open", sleep_fn, clock_fn)
            _require_stationary(session, points["A_middle"], "任务2预检后的 A_middle", start=True)
            output_fn("任务2起点自动检查通过：A_middle、toolcoord2、自动模式、无报警、夹爪张开。")
            log["events"].append({
                "event": "task2_preflight_passed",
                "start_gripper": "open",
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            })

            _move_path(
                session, points, ("B_1high", "B_3", "B_4low"),
                speed, log, sleep_fn, clock_fn, output_fn,
            )
            _require_stationary(session, points["B_4low"], "B_4low before close")
            session.close_gripper(grip_force)
            close_sample = _wait_gripper(session, "close", sleep_fn, clock_fn)
            _require_stationary(session, points["B_4low"], "B_4low after automatic grip check")
            output_fn(
                "夹爪闭爪动作完成且反馈稳定：position=%s current=%s status=%s。"
                % (close_sample.position, close_sample.current, close_sample.status)
            )
            log["events"].append({
                "event": "task2_picked",
                "point": "B_4low",
                "grip_force": float(grip_force),
                "gripper_position": close_sample.position,
                "gripper_current": close_sample.current,
                "gripper_status": close_sample.status,
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            })

            _move_path(
                session, points, ("B_3",), extract_speed,
                log, sleep_fn, clock_fn, output_fn,
                acceleration=extract_acceleration,
            )
            lift_sample = _wait_gripper(session, "holding", sleep_fn, clock_fn)
            _require_stationary(session, points["B_3"], "B_3 after first lift")
            output_fn(
                "首次抬升后夹爪仍保持闭合：position=%s current=%s status=%s，继续返回 A_middle。"
                % (lift_sample.position, lift_sample.current, lift_sample.status)
            )
            log["events"].append({
                "event": "task2_first_lift_closed_verified",
                "point": "B_3",
                "gripper_position": lift_sample.position,
                "gripper_current": lift_sample.current,
                "gripper_status": lift_sample.status,
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            })

            _move_path(
                session, points, ("B_2", "B_1high", "A_middle"),
                speed, log, sleep_fn, clock_fn, output_fn,
            )
            _wait_gripper(session, "holding", sleep_fn, clock_fn)
            _require_stationary(session, points["A_middle"], "任务2返回后的 A_middle", start=True)
            output_fn("任务2返回 A_middle 后夹爪仍保持闭合。")
        else:
            _check_start_gripper(session, 1, "closed", sleep_fn, clock_fn)
            _require_stationary(session, points["A_middle"], "任务1预检后的 A_middle", start=True)
            output_fn("任务1起点自动检查通过：A_middle、toolcoord2、自动模式、无报警、夹爪闭合。")
            log["events"].append({
                "event": "task1_preflight_passed",
                "start_gripper": "closed",
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            })

            _move_path(
                session, points, ("B_1high", "B_3", "B_4low"),
                speed, log, sleep_fn, clock_fn, output_fn,
            )
            _require_stationary(session, points["B_4low"], "B_4low before open")
            session.open_gripper()
            _wait_gripper(session, "open", sleep_fn, clock_fn)
            _require_stationary(
                session, points["B_4low"], "B_4low after automatic placement check"
            )
            output_fn("张爪完成，机器人位置、碰撞、报警和急停状态正常，开始返回 A_middle。")
            log["events"].append({
                "event": "task1_open_and_robot_state_verified",
                "point": "B_4low",
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            })

            _move_path(
                session, points, ("B_3", "B_2", "B_1high", "A_middle"),
                speed, log, sleep_fn, clock_fn, output_fn,
            )
            _wait_gripper(session, "open", sleep_fn, clock_fn)
            _require_stationary(session, points["A_middle"], "任务1返回后的 A_middle", start=True)
            output_fn("任务1返回 A_middle 后夹爪保持张开，程序判定为空夹爪。")

        log["success"] = True
        log["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        path = _write_log(log_dir or (Path(__file__).resolve().parent / "logs"), log, task_number)
        output_fn("任务%d完成，已停在 A_middle。程序结束，不自动执行下一任务。" % task_number)
        return path or ""
    except Exception as exc:
        error = exc if isinstance(exc, CycleError) else CycleError(str(exc))
        log["success"] = False
        log["failure"] = str(error)
        log["finished_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        path = _write_log(log_dir or (Path(__file__).resolve().parent / "logs"), log, task_number)
        output_fn("任务%d程序中止：%s" % (task_number, error))
        output_fn("不会自动张爪、不会自动返回 A、不会执行下一任务。")
        if path:
            output_fn("日志：%s" % path)
        raise error


def _execute_task_safely(session: Any, task_number: int, **kwargs: Any) -> str:
    try:
        return run_task(session, task_number, **kwargs)
    except BaseException:
        try:
            session.stop_motion()
        except Exception:
            pass
        raise


def execute_task_1(session: Any, **kwargs: Any) -> str:
    """Signal-safe entry point: any failure calls StopMotion, never opens the gripper."""
    return _execute_task_safely(session, 1, **kwargs)


def execute_task_2(session: Any, **kwargs: Any) -> str:
    """Signal-safe entry point: any failure calls StopMotion, never opens the gripper."""
    return _execute_task_safely(session, 2, **kwargs)


def parse_args(
    argv: Optional[Sequence[str]] = None,
    allow_task_option: bool = True,
) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fairino B槽任务1/任务2独立执行程序")
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--sdk-root", default="", help="包含 fairino 包的目录")
    if allow_task_option:
        parser.add_argument("--task", type=int, choices=(1, 2), help="手动选择任务1或任务2")
    parser.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED,
        help="MoveL指令速度百分比，默认100，允许范围(0, 100]",
    )
    parser.add_argument(
        "--global-speed",
        type=float,
        default=GLOBAL_SPEED,
        help="控制器全局速度百分比，默认30，允许范围(0, 100]",
    )
    parser.add_argument(
        "--extract-speed",
        type=float,
        default=DEFAULT_EXTRACT_SPEED,
        help="任务2从 B_4low 首次抬起时的LIN速度百分比，默认100，允许范围(0, 100]",
    )
    parser.add_argument(
        "--extract-acc",
        type=float,
        default=DEFAULT_EXTRACT_ACC,
        help="任务2从 B_4low 首次抬起时的加速度百分比，默认100，允许范围(0, 100]",
    )
    parser.add_argument(
        "--grip-force",
        type=float,
        default=DEFAULT_GRIP_FORCE,
        help="任务2夹爪闭合力百分比，默认70，允许范围(0, 100]",
    )
    return parser.parse_args(argv)


def _select_task_interactively() -> Optional[int]:
    print("手动任务选择：")
    print("  1 = 任务1（A已夹件 → B_4low放置 → 返回A空夹爪）")
    print("  2 = 任务2（A空夹爪 → B_4low取件 → 返回A夹持）")
    choice = input("请输入 1 或 2，输入 Q 退出：").strip()
    if choice.upper() == "Q":
        return None
    if choice not in ("1", "2"):
        raise CycleError("invalid task selection %r" % choice)
    return int(choice)


def main(default_task: Optional[int] = None, argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv, allow_task_option=default_task is None)
    task_number = default_task if default_task is not None else args.task
    if task_number is None:
        try:
            task_number = _select_task_interactively()
        except CycleError as exc:
            print("程序未启动：%s" % exc)
            return 2
        if task_number is None:
            print("已退出，未发送任何运动指令。")
            return 0

    if args.sdk_root:
        sys.path.insert(0, str(Path(args.sdk_root).resolve()))
    try:
        from fairino import Robot
    except Exception as exc:
        print("加载 fairino SDK 失败：", exc)
        return 4

    session: Optional[FairinoSession] = None
    try:
        robot = Robot.RPC(str(args.ip))
        session = FairinoSession(robot)
        run_task(
            session,
            task_number,
            speed=float(args.speed),
            global_speed=float(args.global_speed),
            extract_speed=float(args.extract_speed),
            extract_acceleration=float(args.extract_acc),
            grip_force=float(args.grip_force),
        )
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，调用 StopMotion；不自动张爪。")
        if session is not None:
            try:
                session.stop_motion()
            except Exception as exc:
                print("StopMotion 失败：", exc)
        return 130
    except Exception:
        if session is not None:
            try:
                session.stop_motion()
            except Exception:
                pass
        return 3
    finally:
        if session is not None:
            session.close()


if __name__ == "__main__":
    raise SystemExit(main())
