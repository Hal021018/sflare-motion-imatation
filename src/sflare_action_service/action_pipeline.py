#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动作文件解析与 EMI 轨迹规划。

插补和 EMI 轨迹分段参考 TAIJI.py：
1. 原生五次多项式插值与样条预插补；
2. prepare_path 步长强制限速细分，防止关节轴保护；
3. transition_path 起势平滑过渡；
4. return_zero_path 终点回零规划；
5. 发包前按控制器实时返回的限位检查整条轨迹。
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union


# EMI timing follows the validated TAIJI sender.  Fixed-action coordinates and
# behavior remain those of the production action service.
SEND_TIME_SEC = 0.040
DEFAULT_PERIOD_SEC = SEND_TIME_SEC
MAX_JOINT_SPEED_DEG_S = 85.0  # TAIJI.py 的默认单关节限速 (deg/s)
TRANSITION_SEC = 5.0  # TAIJI.py 的默认起势过渡时间
ZERO_POSE_DEG = [0.0] * 7

@dataclass(frozen=True)
class Keyframe:
    time: float
    left: Tuple[float, ...]
    right: Tuple[float, ...]
    type: str = "stop"
    hold: float = 0.0


@dataclass(frozen=True)
class ActionEvent:
    time: float
    target: str  # "left_hand" | "right_hand" | "dual_hand"
    action: str  # "safe_fist" | "open" | "hold_stretch_grip"
    payload: Optional[Dict[str, Any]] = None


@dataclass(frozen=True)
class ActionDocument:
    schema_version: str
    name: str
    scope: str
    units: str
    period: float
    speed_scale: float
    auto_return_zero: bool
    keyframes: Tuple[Keyframe, ...]
    events: Tuple[ActionEvent, ...]
    file_path: str
    content_hash: str
    program: Tuple[Dict[str, Any], ...] = ()


@dataclass(frozen=True)
class ActionPlan:
    name: str
    scope: str
    period: float
    speed_scale: float
    # 按照 TAIJI.py 分段：起势过渡段、主体动作段、回零段
    trans_l: List[List[float]]
    trans_r: List[List[float]]
    action_l: List[List[float]]
    action_r: List[List[float]]
    return_l: List[List[float]]
    return_r: List[List[float]]
    events: Tuple[ActionEvent, ...]


def compute_file_hash(path: Path) -> str:
    hasher = hashlib.sha256()
    hasher.update(path.read_bytes())
    return hasher.hexdigest()


def enforce_runtime_safety(values: Sequence[float]) -> List[float]:
    """Validate shape/finiteness without silently changing action coordinates.

    The production service treats each controller's ``get_limit`` response as
    the authority.  The EMI coordinator applies that same check to every
    planned frame before power-on.
    """
    if len(values) != 7 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError("关节数据必须是有限的 7 轴数值")
    return [float(value) for value in values]


def validate_initial(values: Sequence[float]) -> List[float]:
    """Validate encoder data while preserving the controller-reported pose."""
    try:
        return enforce_runtime_safety(values)
    except ValueError as exc:
        raise ValueError("初始反馈必须为有限的七轴数据") from exc


def prepare_path(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
    period: float,
    speed: float = MAX_JOINT_SPEED_DEG_S,
) -> Tuple[List[List[float]], List[List[float]]]:
    """完全复用自 TAIJI.py 的核心限速细分算法：
    对任何超出 step_limit 的关节角度差进行强制细分重采样，物理上杜绝关节轴超速！
    """
    if not math.isfinite(period) or period <= 0:
        raise ValueError("控制周期必须是大于 0 的有限数")
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("关节速度上限必须是大于 0 的有限数")
    if not left or not right or len(left) != len(right):
        raise ValueError("左右臂轨迹必须非空且帧数一致")

    out_l = [enforce_runtime_safety(left[0])]
    out_r = [enforce_runtime_safety(right[0])]
    step_limit = period * speed

    for goal_l, goal_r in zip(left[1:], right[1:]):
        a_l, a_r = out_l[-1], out_r[-1]
        t_l = enforce_runtime_safety(goal_l)
        t_r = enforce_runtime_safety(goal_r)
        delta = max(
            max(abs(b - a) for a, b in zip(a_l, t_l)),
            max(abs(b - a) for a, b in zip(a_r, t_r)),
        )
        count = max(1, math.ceil(delta / step_limit))
        for i in range(1, count + 1):
            u = i / count
            out_l.append([a + (b - a) * u for a, b in zip(a_l, t_l)])
            out_r.append([a + (b - a) * u for a, b in zip(a_r, t_r)])

    return out_l, out_r


def transition_path(
    left: Sequence[float],
    right: Sequence[float],
    first_l: Sequence[float],
    first_r: Sequence[float],
    period: float = SEND_TIME_SEC,
    speed: float = MAX_JOINT_SPEED_DEG_S,
    transition_sec: float = 3.0,
) -> Tuple[List[List[float]], List[List[float]]]:
    """平滑起步过渡轨迹 (五次多项式 + prepare_path，复用自 TAIJI.py)。"""
    s_l = enforce_runtime_safety(left)
    s_r = enforce_runtime_safety(right)
    g_l = enforce_runtime_safety(first_l)
    g_r = enforce_runtime_safety(first_r)

    count = max(1, math.ceil(transition_sec / period))
    paths = [[], []]
    for i in range(count + 1):
        u = i / count
        s = u**3 * (10.0 - 15.0 * u + 6.0 * u * u)
        for dest, initial, target in zip(paths, (s_l, s_r), (g_l, g_r)):
            dest.append([a + (b - a) * s for a, b in zip(initial, target)])

    return prepare_path(paths[0], paths[1], period, speed)


def return_zero_path(
    final_l: Sequence[float],
    final_r: Sequence[float],
    period: float = SEND_TIME_SEC,
    speed: float = MAX_JOINT_SPEED_DEG_S,
) -> Tuple[List[List[float]], List[List[float]]]:
    """平滑回零轨迹 (复用自 TAIJI.py)。"""
    start_l = enforce_runtime_safety(final_l)
    start_r = enforce_runtime_safety(final_r)
    zero_l = enforce_runtime_safety(ZERO_POSE_DEG)
    zero_r = enforce_runtime_safety(ZERO_POSE_DEG)
    return prepare_path([start_l, zero_l], [start_r, zero_r], period, speed)


def _quintic_hermite_fallback(
    times: Sequence[float],
    points: Sequence[Sequence[float]],
    t_eval: Sequence[float],
    stop_points: Optional[Sequence[bool]] = None,
) -> List[List[float]]:
    """Piecewise quintic Hermite interpolation with optional full-stop nodes."""
    n_pts = len(times)
    if n_pts == 0 or len(points) != n_pts:
        raise ValueError("插值关键帧不能为空且时间/点位数量必须一致")
    num_joints = len(points[0])
    if stop_points is None:
        stop_points = [False] * n_pts
    if len(stop_points) != n_pts:
        raise ValueError("stop_points 数量必须与关键帧一致")

    velocities = [[0.0] * num_joints for _ in range(n_pts)]
    accelerations = [[0.0] * num_joints for _ in range(n_pts)]

    for i in range(1, n_pts - 1):
        if stop_points[i]:
            continue
        dt = times[i + 1] - times[i - 1]
        dt_prev = times[i] - times[i - 1]
        dt_next = times[i + 1] - times[i]
        for j in range(num_joints):
            before = (points[i][j] - points[i - 1][j]) / dt_prev
            after = (points[i + 1][j] - points[i][j]) / dt_next
            if before * after < 0.0:
                # A reversal must have zero velocity, but not zero
                # acceleration.  The TAIJI Hermite acceleration rounds the
                # turning point without inserting a full stop there.
                accelerations[i][j] = 2.0 * (after - before) / dt
            elif before * after > 0.0:
                # Keep a through-point moving, while limiting its tangent so
                # the spline cannot bow outside neighboring keyframe values.
                slope = (points[i + 1][j] - points[i - 1][j]) / dt
                velocities[i][j] = math.copysign(
                    min(abs(slope), 3.0 * abs(before), 3.0 * abs(after)), slope
                )

    result = []
    seg_idx = 0
    for t in t_eval:
        if t <= times[0]:
            result.append([float(v) for v in points[0]])
            continue
        if t >= times[-1]:
            result.append([float(v) for v in points[-1]])
            continue
        while seg_idx < n_pts - 2 and times[seg_idx + 1] < t:
            seg_idx += 1

        t0, t1 = times[seg_idx], times[seg_idx + 1]
        T = t1 - t0
        u = (t - t0) / T
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u

        h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5
        h1 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5
        h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5
        h3 = 0.5 * u3 - u4 + 0.5 * u5
        h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5
        h5 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5

        joint_vals = []
        for j in range(num_joints):
            q0, q1 = points[seg_idx][j], points[seg_idx + 1][j]
            v0, v1 = velocities[seg_idx][j], velocities[seg_idx + 1][j]
            a0, a1 = accelerations[seg_idx][j], accelerations[seg_idx + 1][j]

            val = (
                h0 * q0
                + h1 * T * v0
                + h2 * (T * T) * a0
                + h3 * (T * T) * a1
                + h4 * T * v1
                + h5 * q1
            )
            joint_vals.append(val)
        result.append(joint_vals)

    return result


class ActionRepository:
    """只读动作仓库。"""

    def __init__(self, actions_dir: Union[str, Path]) -> None:
        self.actions_dir = Path(actions_dir).expanduser().resolve()
        self._index: Dict[str, Path] = {}
        self._cache: Dict[str, Tuple[ActionDocument, str]] = {}
        self.reload()

    def reload(self) -> None:
        self._index.clear()
        self._cache.clear()
        if not self.actions_dir.is_dir():
            raise RuntimeError(f"动作目录不存在: {self.actions_dir}")

        for file_path in sorted(self.actions_dir.glob("*.json")):
            name = file_path.stem
            if not name.replace("_", "").isalnum():
                raise RuntimeError(f"非法动作文件名: {file_path.name}")
            if name in self._index:
                raise RuntimeError(f"重名动作: {name}")
            self._index[name] = file_path

    @property
    def available_actions(self) -> List[str]:
        return sorted(self._index.keys())

    def get(self, action_name: str) -> ActionDocument:
        file_path = self._index.get(action_name)
        if file_path is None:
            raise RuntimeError(f"未找到动作: {action_name}")

        current_hash = compute_file_hash(file_path)
        cached = self._cache.get(action_name)
        if cached is not None and cached[1] == current_hash:
            return cached[0]

        doc = parse_action_file(file_path, current_hash)
        self._cache[action_name] = (doc, current_hash)
        return doc


def parse_action_file(file_path: Path, content_hash: str) -> ActionDocument:
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise RuntimeError("动作文件根节点必须是 JSON object")
    schema_version = str(data.get("schema_version", ""))
    if schema_version != "1.0.0":
        raise RuntimeError(f"不支持的动作文件版本: {schema_version!r}")

    name = str(data.get("name", ""))
    if name != file_path.stem:
        raise RuntimeError(f"文件名与内部 name 不符: {file_path.stem} != {name}")

    ctrl = data.get("control", {})
    period = float(ctrl.get("period", SEND_TIME_SEC))
    speed_scale = float(ctrl.get("speed_scale", 1.0))
    if not math.isfinite(period) or period <= 0:
        raise RuntimeError("control.period 必须是大于 0 的有限数")
    if not math.isfinite(speed_scale) or speed_scale <= 0:
        raise RuntimeError("control.speed_scale 必须是大于 0 的有限数")
    scope = str(data.get("scope", ""))
    if scope not in {"dual", "right"}:
        raise RuntimeError(f"不支持的动作 scope: {scope!r}")
    units = str(data.get("units", ""))
    if units != "deg":
        raise RuntimeError(f"不支持的关节单位: {units!r}")
    exit_cfg = data.get("exit", {})
    auto_return_zero = bool(exit_cfg.get("auto_return_zero", True))

    raw_keyframes = data.get("keyframes", [])
    if not isinstance(raw_keyframes, list) or not raw_keyframes:
        raise RuntimeError("动作至少需要一个关键帧")
    keyframes: List[Keyframe] = []
    prev_time: Optional[float] = None
    for idx, item in enumerate(raw_keyframes):
        if not isinstance(item, dict):
            raise RuntimeError(f"关键帧 #{idx} 必须是 JSON object")
        t = float(item.get("time", 0.0))
        if not math.isfinite(t) or t < 0:
            raise RuntimeError(f"关键帧 #{idx} time 必须是非负有限数")
        if prev_time is not None and t <= prev_time:
            raise RuntimeError(f"关键帧 #{idx} 时间必须严格递增 ({t} <= {prev_time})")
        prev_time = t

        left = [float(v) for v in item.get("left", [])]
        right = [float(v) for v in item.get("right", [])]
        if len(right) != 7 or len(left) != (7 if scope == "dual" else 0):
            raise RuntimeError(f"关键帧 #{idx} 关节数据轴数与 scope 不符")
        try:
            if scope == "dual":
                enforce_runtime_safety(left)
            enforce_runtime_safety(right)
        except ValueError as exc:
            raise RuntimeError(f"关键帧 #{idx} 点位非法: {exc}") from exc
        frame_type = str(item.get("type", "stop"))
        if frame_type not in {"stop", "pass"}:
            raise RuntimeError(f"关键帧 #{idx} type 必须是 stop 或 pass")
        hold = float(item.get("hold", 0.0))
        if not math.isfinite(hold) or hold < 0:
            raise RuntimeError(f"关键帧 #{idx} hold 必须是非负有限数")

        keyframes.append(
            Keyframe(
                time=t,
                left=tuple(left),
                right=tuple(right),
                type=frame_type,
                hold=hold,
            )
        )

    raw_events = data.get("events", [])
    if not isinstance(raw_events, list):
        raise RuntimeError("events 必须是 JSON array")
    events: List[ActionEvent] = []
    valid_targets = {"left_hand", "right_hand", "dual_hand"}
    valid_actions = {"open", "safe_fist", "hold_stretch_grip", "half_heart", "like", "point", "chin_rest"}
    for idx, item in enumerate(raw_events):
        if not isinstance(item, dict):
            raise RuntimeError(f"事件 #{idx} 必须是 JSON object")
        event_time = float(item.get("time", 0.0))
        target = str(item.get("target", ""))
        event_action = str(item.get("action", ""))
        if not math.isfinite(event_time) or event_time < 0:
            raise RuntimeError(f"事件 #{idx} time 必须是非负有限数")
        if target not in valid_targets:
            raise RuntimeError(f"事件 #{idx} target 不受支持: {target!r}")
        if event_action not in valid_actions:
            raise RuntimeError(f"事件 #{idx} action 不受支持: {event_action!r}")
        if scope == "right" and target != "right_hand":
            raise RuntimeError(f"事件 #{idx} 与 right scope 不符: {target!r}")
        events.append(
            ActionEvent(
                time=event_time,
                target=target,
                action=event_action,
                payload=item.get("payload"),
            )
        )

    program = data.get("program", [])
    if not isinstance(program, list):
        raise RuntimeError("program 必须是 JSON array")
    if program:
        if scope != "right" or auto_return_zero:
            raise RuntimeError("program 仅支持 right 且必须显式配置回零")
        if events or any(frame.hold for frame in keyframes):
            raise RuntimeError("program 动作的手势和停留必须写入 program，不能混用 events/hold")
        from action_program import validate_program
        validate_program(program, len(keyframes))

    return ActionDocument(
        schema_version=schema_version,
        name=name,
        scope=scope,
        units=units,
        period=period,
        speed_scale=speed_scale,
        auto_return_zero=auto_return_zero,
        keyframes=tuple(keyframes),
        events=tuple(events),
        file_path=str(file_path),
        content_hash=content_hash,
        program=tuple(program),
    )


def plan_action_taiji_style(
    doc: ActionDocument,
    initial_l: Sequence[float],
    initial_r: Sequence[float],
    speed_scale: Optional[float] = None,
) -> ActionPlan:
    """严格按照 TAIJI.py 流水线规划动作：
    1. 关键帧五次插补 (precompute_trajectory)；
    2. transition_path 起势过渡；
    3. prepare_path 主体限速细分；
    4. return_zero_path 回零规划。
    """
    period = doc.period
    scale = doc.speed_scale if speed_scale is None else float(speed_scale)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("speed_scale 必须是大于 0 的有限数")

    times = [kf.time / scale for kf in doc.keyframes]
    left_points = [list(kf.left) for kf in doc.keyframes]
    right_points = [list(kf.right) for kf in doc.keyframes]

    total_time = times[-1]
    if len(times) == 1:
        time_series = [times[0]]
    else:
        num_intervals = max(1, math.ceil(total_time / period))
        time_series = [min(round(i * period, 10), total_time) for i in range(num_intervals + 1)]

    # 1. 全量五次多项式预插补 (复用 TAIJI)
    stop_points = [kf.type == "stop" for kf in doc.keyframes]
    raw_l = (_quintic_hermite_fallback(times, left_points, time_series, stop_points)
             if doc.scope == "dual" else [])
    raw_r = _quintic_hermite_fallback(times, right_points, time_series, stop_points)
    safe_l = [enforce_runtime_safety(q) for q in raw_l]
    safe_r = [enforce_runtime_safety(q) for q in raw_r]

    # 注入关键帧 hold 保持点
    body_l: List[List[float]] = []
    body_r: List[List[float]] = []
    sample_idx = 0
    for kf_idx, kf in enumerate(doc.keyframes):
        target_t = times[kf_idx]
        while sample_idx < len(time_series) and time_series[sample_idx] <= target_t + 1e-5:
            if doc.scope == "dual":
                body_l.append(safe_l[sample_idx])
            body_r.append(safe_r[sample_idx])
            sample_idx += 1
        if kf.hold > 0:
            hold_count = max(1, math.ceil(kf.hold / period))
            for _ in range(hold_count):
                if doc.scope == "dual":
                    body_l.append(enforce_runtime_safety(kf.left))
                body_r.append(enforce_runtime_safety(kf.right))
    while sample_idx < len(time_series):
        if doc.scope == "dual":
            body_l.append(safe_l[sample_idx])
        body_r.append(safe_r[sample_idx])
        sample_idx += 1

    # 2. transition_path 起势过渡 (复用 TAIJI)
    init_r = validate_initial(initial_r)
    if doc.scope == "dual":
        init_l = validate_initial(initial_l)
        trans_l, trans_r = transition_path(init_l, init_r, body_l[0], body_r[0], period, MAX_JOINT_SPEED_DEG_S, TRANSITION_SEC)
    else:
        trans_l = []
        trans_r, _ = transition_path(init_r, init_r, body_r[0], body_r[0], period, MAX_JOINT_SPEED_DEG_S, TRANSITION_SEC)

    # 3. prepare_path 动作主体限速细分 (复用 TAIJI)
    if doc.scope == "dual":
        action_l, action_r = prepare_path(body_l, body_r, period, MAX_JOINT_SPEED_DEG_S)
    else:
        action_l = []
        action_r, _ = prepare_path(body_r, body_r, period, MAX_JOINT_SPEED_DEG_S)

    # 4. return_zero_path 平滑回零 (复用 TAIJI)
    if doc.auto_return_zero:
        if doc.scope == "dual":
            return_l, return_r = return_zero_path(action_l[-1], action_r[-1], period, MAX_JOINT_SPEED_DEG_S)
        else:
            return_l = []
            return_r, _ = return_zero_path(action_r[-1], action_r[-1], period, MAX_JOINT_SPEED_DEG_S)
    else:
        return_l = [action_l[-1]] if doc.scope == "dual" else []
        return_r = [action_r[-1]]

    return ActionPlan(
        name=doc.name,
        scope=doc.scope,
        period=period,
        speed_scale=scale,
        trans_l=trans_l,
        trans_r=trans_r,
        action_l=action_l,
        action_r=action_r,
        return_l=return_l,
        return_r=return_r,
        events=doc.events,
    )
