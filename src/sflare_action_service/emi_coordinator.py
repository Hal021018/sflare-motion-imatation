#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""双臂独立控制卡 EMI 执行器。

适配：
- 选项 B：右臂 (192.168.3.100) 与左臂 (10.251.251.100) 双独立 Controller，各 robot_id=1
- 灵巧手在动作对应时间点发送 485 报文
- 保留 TAIJI.py 中与固定动作执行相关的关键闭环：
  1. FeedbackTracker 接收实际 EMI 反馈与超时中断监控；
  2. DualServoJSender 双线程并发 servo_j 与周期耗时统计；
  3. tick 闭环保护，超时记录不追帧；
  4. 最终 zero 位姿由 tracker.at_target 真实编码器闭环确认到达与收敛；
  5. 停发等待 SERVO_STOP_WAIT_SEC (2.0s) 伺服插补平息；
  6. safe_cleanup_arm 读 1203.1 确认退出 EMI，轮询确认强电=0 再安全释放。
"""

from __future__ import annotations

import math
import csv
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from action_pipeline import (
    ActionDocument,
    ActionPlan,
    ActionEvent,
    enforce_runtime_safety,
    validate_initial,
    plan_action_taiji_style,
    MAX_JOINT_SPEED_DEG_S,
)
from action_session import extract_error_codes


EMI_START_WORD = 0x8800
EMI_ONLINE_SCENE = 1
PROFILE_VALUES = (100, 100, 100, 100, 0.1, 0.1, -1)
SERVO_ACCELERATION = 0.0
SERVO_VELOCITY = 0.04
SERVO_LOOKAHEAD_SEC = 0.03
SERVO_GAIN = 900.0
SERVO_COMMAND_AXES = 15
FEEDBACK_STALE_SEC = 0.5
SERVO_STOP_WAIT_SEC = 2.0  # 停止发包后等待伺服插补平息的时长 (对齐 TAIJI.py)
FINAL_TOLERANCE_DEG = 1.0
FINAL_SETTLE_SEC = 6.0
POWER_OFF_TIMEOUT_SEC = 3.0
SDK_TIMEOUT_MS = 3000
TRACE_OUTPUT_DIR = Path(__file__).resolve().parent / "traces"

RIGHT_ARM_IP = "192.168.3.100"
LEFT_ARM_IP = "10.251.251.100"
CONTROLLER_PORT = 2094


def high_precision_delay(delay_time: float) -> None:
    if delay_time <= 0:
        return
    start = time.perf_counter()
    if delay_time > 0.002:
        time.sleep(delay_time - 0.0015)
    while time.perf_counter() - start < delay_time:
        pass


def format_send_intervals(intervals: Dict[str, List[float]], period: float) -> List[str]:
    """Summarize consecutive send-start intervals within each motion phase."""
    result = []
    for phase, values in intervals.items():
        if not values:
            result.append(f"{phase}：样本不足")
            continue
        ordered = sorted(values)
        p95 = ordered[math.ceil(len(ordered) * 0.95) - 1]
        late = sum(gap > period * 1.25 for gap in values)
        result.append(
            f"{phase}：间隔样本={len(values)}，均值={sum(values) / len(values) * 1000:.3f}ms，"
            f"P95={p95 * 1000:.3f}ms，最大={ordered[-1] * 1000:.3f}ms，"
            f"超过周期25%={late}次"
        )
    return result


def write_playback_trace(
    action_name: str,
    trace_origin: float,
    records: Sequence[Tuple[Any, ...]],
    output_dir: Path,
) -> Path:
    """Write buffered send/feedback samples only after playback has stopped."""
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / (f"{action_name}_{time.strftime('%Y%m%d_%H%M%S')}_"
                         f"{time.time_ns() % 1000000000:09d}.csv")
    header = ["phase", "frame", "arm", "send_start_s", "send_end_s",
              "send_duration_ms", "feedback_time_s", "feedback_age_ms",
              "feedback_count"]
    header += [f"target_J{axis}" for axis in range(1, 8)]
    header += [f"actual_J{axis}" for axis in range(1, 8)]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for phase, frame, arm, start, end, target, sample in records:
            received = sample.get("received_at") if sample else None
            actual = sample.get("joints") if sample else None
            writer.writerow([
                phase, frame, arm, f"{start - trace_origin:.6f}",
                f"{end - trace_origin:.6f}", f"{(end - start) * 1000:.3f}",
                f"{received - trace_origin:.6f}" if received is not None else "",
                f"{(end - received) * 1000:.3f}" if received is not None else "",
                sample.get("count", "") if sample else "",
                *target, *(actual if actual is not None else [""] * 7),
            ])
    return path


def write_feedback_trace(
    path: Path,
    trace_origin: float,
    samples: Sequence[Dict[str, Any]],
) -> None:
    """Export every buffered EMI callback beside the send trace."""
    header = ["feedback_time_s", "arm", "feedback_count", "robot_mode",
              "robot_state", "error_code", "robot_speed_raw"]
    header += [f"actual_J{axis}" for axis in range(1, 8)]
    header += [f"speed_raw_J{axis}" for axis in range(1, 8)]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for sample in samples:
            speeds = sample.get("joint_speed")
            writer.writerow([
                f"{sample['received_at'] - trace_origin:.6f}",
                sample["label"], sample["count"], sample["mode"],
                sample["state"], sample["error_code"],
                sample.get("robot_speed", "") if sample.get("robot_speed") is not None else "",
                *sample["joints"], *(speeds if speeds is not None else [""] * 7),
            ])


def require_ok(result: Any, operation: str) -> None:
    if result != 0:
        raise RuntimeError(f"{operation} 返回失败: {result!r}")


def require_attached(controller: Any, robot: Any, label: str,
                     robot_id: int = 1) -> None:
    """绑定机器人。

    注意：本控制器（单臂固件）上 ``get_attached_id()`` 恒为 0，且
    ``check_error()`` 会长期残留一条 ``-5314``，两者都不代表绑定失败，
    因此这里只校验 ``attach()`` 的返回码，不做附加断言。
    """
    require_ok(robot.attach(), f"{label} attach")


def power_state(robot: Any) -> int:
    raw = robot.get_state(timeout=SDK_TIMEOUT_MS)
    values = raw.toList() if hasattr(raw, "toList") else list(raw)
    if not values or int(values[0]) not in (0, 1, 2, 3):
        raise RuntimeError(f"非法强电反馈: {raw!r}")
    return int(values[0])


def get_arm_joints(robot: Any) -> List[float]:
    raw = robot.get_joint_pos()
    values = list(raw.toList()) if hasattr(raw, "toList") else list(raw)
    return [float(v) for v in values[:7]]


class FeedbackTracker:
    """保存 EMI 回调的数值副本。"""

    def __init__(self, arm_labels: Sequence[str]) -> None:
        self.arm_labels = tuple(arm_labels)
        self._samples: Dict[str, Dict[str, Any]] = {}
        self._history: List[Dict[str, Any]] = []
        self._callback_errors: Dict[str, str] = {}
        self._lock = threading.Lock()
        self.started_at: Optional[float] = None
        self.callbacks: List[Any] = []

    def start(self) -> None:
        self.started_at = time.perf_counter()

    def at_target(self, targets: Dict[str, Sequence[float]]) -> bool:
        """真实编码器到位判断：所有关节误差在 FINAL_TOLERANCE_DEG 内。"""
        with self._lock:
            return all(
                label in self._samples
                and all(
                    abs(a - b) <= FINAL_TOLERANCE_DEG
                    for a, b in zip(self._samples[label]["joints"], target)
                )
                for label, target in targets.items()
            )

    def describe_target_error(self, targets: Dict[str, Sequence[float]]) -> str:
        """Describe the latest encoder error without making another SDK call."""
        with self._lock:
            samples = dict(self._samples)
        now = time.perf_counter()
        details = []
        for label, target in targets.items():
            sample = samples.get(label)
            if sample is None:
                details.append(f"{label}无反馈")
                continue
            errors = [actual - goal for actual, goal in zip(sample["joints"], target)]
            axis = max(range(7), key=lambda index: abs(errors[index]))
            details.append(
                f"{label}最大误差J{axis + 1}={errors[axis]:+.3f}°，"
                f"各轴误差={[round(value, 3) for value in errors]}°，"
                f"反馈龄期={(now - sample['received_at']) * 1000:.1f}ms"
            )
        return "; ".join(details)

    def latest_joint_samples(self, arm_labels: Sequence[str]) -> Dict[str, Dict[str, Any]]:
        """Copy already-received feedback for tracing; never query the SDK."""
        with self._lock:
            return {label: dict(self._samples[label]) for label in arm_labels
                    if label in self._samples}

    def feedback_history(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._history)

    def make_callback(self, expected_label: str) -> Any:
        def callback(data: Any) -> None:
            try:
                raw_pos = data.robot_joint_pos
                joints = tuple(float(raw_pos[index]) for index in range(7))
                if not all(math.isfinite(value) for value in joints):
                    raise ValueError("反馈关节值含 NaN/Inf")
                try:
                    raw_speed = data.robot_joint_speed
                    joint_speed = tuple(float(raw_speed[index]) for index in range(7))
                    if not all(math.isfinite(value) for value in joint_speed):
                        joint_speed = None
                except (AttributeError, IndexError, TypeError, ValueError):
                    joint_speed = None
                try:
                    robot_speed = float(data.robot_speed)
                    if not math.isfinite(robot_speed):
                        robot_speed = None
                except (AttributeError, TypeError, ValueError):
                    robot_speed = None
                sample = {
                    "label": expected_label,
                    "joints": joints,
                    "joint_speed": joint_speed,
                    "robot_speed": robot_speed,
                    "mode": int(data.robot_mode),
                    "state": int(data.robot_state),
                    "error_code": int(data.error_code),
                    "received_at": time.perf_counter(),
                }
                with self._lock:
                    previous = self._samples.get(expected_label)
                    sample["count"] = 1 if previous is None else previous["count"] + 1
                    self._samples[expected_label] = sample
                    self._history.append(sample)
            except BaseException as exc:
                print(
                    f"[EMI反馈回调异常][{expected_label}]: {exc!r}",
                    file=sys.stderr,
                    flush=True,
                )
                with self._lock:
                    self._callback_errors[expected_label] = repr(exc)

        self.callbacks.append(callback)
        return callback

    def check_faults(self, arm_labels: Sequence[str]) -> List[str]:
        with self._lock:
            samples = dict(self._samples)
            errors = dict(self._callback_errors)
        faults = []
        now = time.perf_counter()
        for label in arm_labels:
            if label in errors:
                faults.append(f"{label} 回调解析错误: {errors[label]}")
            sample = samples.get(label)
            if sample is None:
                if self.started_at is not None and now - self.started_at > FEEDBACK_STALE_SEC:
                    faults.append(f"{label} 未在期限内收到首次 EMI 反馈")
                continue
            if sample["error_code"] != 0:
                faults.append(f"{label} EMI error_code={sample['error_code']}")
            age = now - sample["received_at"]
            if age > FEEDBACK_STALE_SEC:
                faults.append(f"{label} EMI 反馈中断 {age:.3f}s")
        return faults


class DualServoJSender:
    """双线程并行下发 15 轴 servo_j 并等待两臂返回。"""

    def __init__(self, left_robot: Any, right_robot: Any, period: float) -> None:
        self.robots = (("左臂", left_robot), ("右臂", right_robot))
        self.period = period
        self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="taiji-servo-j")
        warmup_barrier = threading.Barrier(2)
        list(self.pool.map(lambda _: warmup_barrier.wait(), range(2)))

    @staticmethod
    def command(values: Sequence[float]) -> List[float]:
        safe = enforce_runtime_safety(values)
        return safe + [0.0] * (SERVO_COMMAND_AXES - 7)

    def _call(self, label: str, robot: Any, command: List[float], gate: threading.Event) -> int:
        gate.wait()
        result = robot.servo_j(
            command,
            SERVO_ACCELERATION,
            SERVO_VELOCITY,
            t=self.period,
            lookahead_time=SERVO_LOOKAHEAD_SEC,
            gain=SERVO_GAIN,
        )
        require_ok(result, f"{label} servo_j")
        return result

    def send(self, left: Sequence[float], right: Sequence[float]) -> float:
        gate = threading.Event()
        commands = (self.command(left), self.command(right))
        futures = [
            self.pool.submit(self._call, label, robot, command, gate)
            for (label, robot), command in zip(self.robots, commands)
        ]
        started = time.perf_counter()
        gate.set()
        failures = []
        for (label, _), future in zip(self.robots, futures):
            try:
                future.result()
            except Exception as exc:
                failures.append(f"{label}: {exc}")
        if failures:
            raise RuntimeError("; ".join(failures))
        return time.perf_counter() - started

    def close(self) -> None:
        # ``cancel_futures`` was added in Python 3.9; the deployed service also
        # supports Python 3.7.
        self.pool.shutdown(wait=True)


class RightServoJSender:
    """Send only the right arm's EMI packets for right-scope actions."""

    def __init__(self, robot: Any, period: float) -> None:
        self.robot = robot
        self.period = period

    def send(self, left: Sequence[float], right: Sequence[float]) -> float:
        started = time.perf_counter()
        require_ok(
            self.robot.servo_j(
                DualServoJSender.command(right),
                SERVO_ACCELERATION,
                SERVO_VELOCITY,
                t=self.period,
                lookahead_time=SERVO_LOOKAHEAD_SEC,
                gain=SERVO_GAIN,
            ),
            "右臂 servo_j",
        )
        return time.perf_counter() - started

    def close(self) -> None:
        pass


class DexterousHand:
    """灵巧手 485 串口。"""

    FIST_THUMB_FLEX = 7.0
    THUMB_OPEN_LEAD_SEC = 0.5  # Lead time, not a feedback-confirmed arrival.

    def __init__(self, controller: Any, label: str) -> None:
        self.controller = controller
        self.label = label
        self.wait = time.sleep

    def _send(self, command: int, data: bytes = b"") -> None:
        frame = bytearray((0x55, 0xAA, 0x02, 0x01, command, len(data)))
        frame.extend(data)
        checksum = 0
        for val in frame[2:]:
            checksum ^= val
        frame.append(checksum)
        result = self.controller.com_send("com2", bytes(frame).hex().upper())
        require_ok(result, f"{self.label} 发送手势报文")

    def _positions(self, values: Sequence[float], speed: int = 120) -> None:
        payload = bytearray()
        for v in values:
            pos = int(max(0.0, min(10.0, float(v))) * 65535 / 10)
            payload.extend(pos.to_bytes(2, "little"))
            payload.append(speed)
        self._send(0x50, payload)

    def open(self) -> None:
        # SerialCtrl 0x4C: individual finger logical position + speed.
        # Do not re-close four fingers when their current pose is unknown.
        self._finger_position(0, 0.0)
        self._finger_position(5, 6.0)
        self.wait(self.THUMB_OPEN_LEAD_SEC)
        self._positions((0, 0, 0, 0, 0, 6))

    def _finger_position(self, finger: int, value: float, speed: int = 120) -> None:
        position = int(max(0.0, min(10.0, float(value))) * 65535 / 10)
        self._send(0x4C, bytes((finger,)) + position.to_bytes(2, "little") + bytes((speed,)))

    def safe_fist(self) -> None:
        self._positions((self.FIST_THUMB_FLEX, 10, 10, 10, 10, 5.5))

    def hold_stretch_grip(self) -> None:
        self._positions((self.FIST_THUMB_FLEX, 10, 10, 10, 10, 5.5))

    def half_heart(self) -> None:
        self._positions((2, 3, 3, 3, 3, 8), speed=150)

    def like(self) -> None:
        self._positions((0, 10, 10, 10, 10, 0), speed=150)

    def point(self) -> None:
        self._positions((4, 0, 10, 10, 10, 6), speed=150)

    def chin_rest(self) -> None:
        self._positions((4, 2, 2, 2, 2, 6), speed=150)


def safe_cleanup_arm(
    label: str,
    controller: Any,
    robot: Any,
    errors: List[str],
    emi_attempted: bool = True,
) -> bool:
    """严格下电与退出确认。"""
    if emi_attempted and robot is not None and controller is not None:
        try:
            state = power_state(robot)
            emi = str(controller.idn_read("1203.1", "1.0.0")).strip()
            if state == 1 and emi == "1":
                require_ok(robot.emi_end(timeout=SDK_TIMEOUT_MS), f"{label} emi_end")
            elif state != 0 and emi != "0":
                errors.append(f"{label} EMI 退出状态异常: power={state}, EMI={emi}")
        except Exception as exc:
            print(f"[{label}] EMI 退出异常: {exc!r}", file=sys.stderr, flush=True)
            errors.append(f"{label} EMI 退出失败: {exc}")

    for attempt in range(3):
        try:
            if controller is not None:
                require_ok(
                    controller.set_power_enable(False, 1, timeout=SDK_TIMEOUT_MS),
                    f"{label} 去使能",
                )
            deadline = time.monotonic() + POWER_OFF_TIMEOUT_SEC
            while time.monotonic() < deadline:
                if robot is None or power_state(robot) == 0:
                    print(f"[{label}] 已确认强电状态=0")
                    return True
                time.sleep(0.05)
            raise RuntimeError("等待强电状态=0 超时")
        except Exception as exc:
            print(f"[{label}] 下电尝试 {attempt+1}/3 异常: {exc!r}", file=sys.stderr, flush=True)

    errors.append(f"{label} 未能确认下电；禁止主动断连")
    return False


class EmiCoordinator:
    """双臂 EMI 执行协调器 (严格对齐 TAIJI.py 的 run 执行时序)。"""

    def __init__(
        self,
        sdk_module: Optional[Any] = None,
        logger: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.log = logger or (lambda msg: print(f"[EMI] {msg}", flush=True))
        self.event_callback = None
        self.sdk = sdk_module
        self.right_controller: Any = None
        self.left_controller: Any = None
        self.right_robot: Any = None
        self.left_robot: Any = None
        self.right_hand: Optional[DexterousHand] = None
        self.left_hand: Optional[DexterousHand] = None
        self.last_error: Optional[str] = None
        self.safe_to_reuse = True
        self.chassis = None
        self.diagnostic_codes = frozenset()
        self.diagnostics_complete = False
        self.recovery_permitted = None
        self.recovery_outcome = None
        self.recovery_codes = frozenset()

    def _load_sdk(self) -> Any:
        if self.sdk is not None:
            return self.sdk
        try:
            import PallasSDK
            self.sdk = PallasSDK
            return self.sdk
        except ImportError as exc:
            raise RuntimeError(f"未能导入 PallasSDK: {exc}") from exc

    def _init_hands(self, events: Sequence[ActionEvent], wait=None) -> None:
        """Initialize exactly the hand resources required by the original action."""
        com_port = getattr(getattr(self.sdk, "ComPort", None), "Com_485", 1)
        try:
            self.right_controller.com_close("com2")
        except Exception:
            pass
        require_ok(
            self.right_controller.com_open("com2", com_port, 115200, 8, 1, 0, True),
            "右手 485 初始化",
        )
        self.right_hand = DexterousHand(self.right_controller, "右手")
        self.right_hand.wait = wait or time.sleep
        self.right_hand.open()

        needs_left_hand = any(
            event.target in {"left_hand", "dual_hand"} for event in events
        )
        if not needs_left_hand:
            return
        try:
            self.left_controller.com_close("com2")
        except Exception:
            pass
        require_ok(
            self.left_controller.com_open("com2", com_port, 115200, 8, 1, 0, True),
            "左手 485 初始化",
        )
        self.left_hand = DexterousHand(self.left_controller, "左手")
        self.left_hand.wait = wait or time.sleep

    def _dispatch_hand_event(self, event: ActionEvent) -> None:
        act = event.action
        target = event.target
        self.log(f"触发灵巧手事件: {target} -> {act}")
        if target in {"right_hand", "dual_hand"}:
            if self.right_hand is None:
                raise RuntimeError("动作要求右手，但右手串口未初始化")
            method = getattr(self.right_hand, act, None)
            if not callable(method):
                raise RuntimeError(f"不支持的右手动作: {act}")
            method()
        if target in {"left_hand", "dual_hand"}:
            if self.left_hand is None:
                raise RuntimeError("动作要求左手，但左手串口未初始化")
            method = getattr(self.left_hand, act, None)
            if not callable(method):
                raise RuntimeError(f"不支持的左手动作: {act}")
            method()

    def get_current_joint_pos(self) -> Tuple[List[float], List[float]]:
        if self.right_robot is None:
            raise RuntimeError("右臂控制器尚未连接，不能读取实时关节位置")
        left = get_arm_joints(self.left_robot) if self.left_robot is not None else []
        return (left, get_arm_joints(self.right_robot))

    @staticmethod
    def _validate_plan_limits(plan: ActionPlan, left_robot: Any, right_robot: Any) -> None:
        """Apply the production service's controller-reported target preflight."""
        trajectories = [("右臂", right_robot, plan.trans_r, plan.action_r, plan.return_r)]
        if plan.scope == "dual":
            trajectories.insert(0, ("左臂", left_robot, plan.trans_l, plan.action_l, plan.return_l))
        for label, robot, *segments in trajectories:
            limits = []
            for axis in range(1, 8):
                raw = [float(value) for value in robot.get_limit(axis, timeout=SDK_TIMEOUT_MS)]
                if len(raw) < 2 or not all(math.isfinite(value) for value in raw):
                    raise RuntimeError(f"{label} J{axis} 控制器限位反馈无效: {raw}")
                limits.append((min(raw), max(raw)))
            for segment in segments:
                for frame_index, frame in enumerate(segment):
                    for axis, (value, (low, high)) in enumerate(zip(frame, limits), start=1):
                        if value < low or value > high:
                            raise RuntimeError(
                                f"{label}轨迹点越界: frame={frame_index}, J{axis}={value:.3f}, "
                                f"limit=[{low:.3f}, {high:.3f}]"
                            )

    def play_document(
        self,
        document: ActionDocument,
        cancel_event: Optional[threading.Event] = None,
    ) -> bool:
        """Connect, read the real starting pose, then plan and play the action."""
        return self.play(None, cancel_event=cancel_event, document=document)

    def recover_limits_on_demand(
        self,
        cancel_event: Optional[threading.Event] = None,
        *,
        margin: float = 3.0,
        jog_speed: float = 50.0,
        poll: float = 0.05,
        max_seconds: float = 20.0,
    ) -> dict:
        """按需脱限：把停在软限位外的轴 Jog 回限位内侧，不回零。

        与 ``play()`` 完全独立：不上电控动作、不启动 EMI、不写轨迹。
        急停、越界判定、方向与超时都由 ``GameLimitRecovery`` 负责；本方法只管
        SDK 生命周期：连接 → 上电 → 关节 Jog 模式 → 脱限 → 停 Jog/下电/断连。
        """
        from game_limit_recovery import GameLimitRecovery

        GameLimitRecovery.validate_parameters(margin, jog_speed, poll, max_seconds)

        sdk = self._load_sdk()
        Controller = sdk.Controller

        self.last_error = None
        self.safe_to_reuse = True
        self.right_controller = Controller()
        connected = False
        powered = False
        jog_configured = False
        emi_was_enabled = False
        emi_disabled = False
        cleanup_errors = []
        try:
            require_ok(
                self.right_controller.connect(RIGHT_ARM_IP, CONTROLLER_PORT),
                "右臂 connect",
            )
            connected = True
            require_ok(self.right_controller.set_data_unit(1), "右臂 set_data_unit")
            self.right_robot = self.right_controller.add_robot(1)
            recovery = GameLimitRecovery(self.right_controller, self.right_robot, self.log)
            # Read-only no-op path: never enable power, attach, change Jog/EMI
            # or write parameters when no physical joint is outside its limits.
            recovery._validate_emergency_stop()
            before, limits, _ = recovery.read_state()
            if cancel_event is not None and cancel_event.is_set():
                raise RuntimeError("按需脱限已取消")
            if not recovery.overtravel_axes(before, limits):
                return recovery.run(cancel_event, margin=margin, jog_speed=jog_speed,
                                    poll=poll, max_seconds=max_seconds)
            initial_emi = str(self.right_controller.idn_read('1203.1', '1.0.0')).strip()
            if initial_emi not in {'0', '1'}:
                raise RuntimeError('EMI状态未知，拒绝切换Jog')
            emi_was_enabled = initial_emi == '1'
            require_attached(self.right_controller, self.right_robot, "右臂", 1)
            require_ok(self.right_robot.set_speed(jog_speed), "右臂 set_speed")
            require_ok(
                self.right_controller.idn_write("1203.150", "1.0.0", "4"),
                "右臂 idn_write",
            )
            recovery._validate_emergency_stop()
            powered = True  # Even a failed enable may have reached the controller.
            require_ok(
                self.right_controller.set_power_enable(True, 1, timeout=SDK_TIMEOUT_MS),
                "右臂 上电",
            )
            # 仅在原来启用时临时关闭 EMI，收尾恢复原状态。
            if emi_was_enabled:
                require_ok(self.right_robot.emi_enable(False), '关闭EMI以脱限')
                emi_disabled = True
            require_ok(self.right_robot.set_frame_type(1), "右臂 set_frame_type")
            require_ok(self.right_robot.set_jog_mode(0, True), "右臂 set_jog_mode")
            jog_configured = True

            recovery = GameLimitRecovery(
                self.right_controller, self.right_robot, self.log)
            result = recovery.run(
                cancel_event,
                margin=margin,
                jog_speed=jog_speed,
                poll=poll,
                max_seconds=max_seconds,
            )
            self.log(f"按需脱限完成：{result['recovered_axes']}")
            return result
        except Exception as exc:
            self.last_error = str(exc)
            raise
        finally:
            try:
                if jog_configured and self.right_robot is not None:
                    try:
                        require_ok(self.right_robot.move_stop_jog(), '停止Jog')
                    finally:
                        require_ok(self.right_robot.set_jog_mode(1, False), '退出Jog模式')
                        require_ok(self.right_robot.set_frame_type(2), '还原关节坐标系')
                if emi_disabled and emi_was_enabled and not (cancel_event and cancel_event.is_set()):
                    recovery._validate_emergency_stop()
                    require_ok(self.right_robot.emi_enable(True), '恢复原EMI状态')
            except Exception as exc:
                cleanup_errors.append(str(exc))
            finally:
                try:
                    if powered and self.right_controller is not None:
                        confirmed = safe_cleanup_arm('右臂', self.right_controller, self.right_robot,
                                                     cleanup_errors, emi_attempted=emi_was_enabled)
                        self.safe_to_reuse = self.safe_to_reuse and confirmed
                finally:
                    if connected and self.right_controller is not None and self.safe_to_reuse:
                        try:
                            require_ok(self.right_controller.disconnect(), '脱限后断连')
                        except Exception as exc:
                            self.safe_to_reuse = False
                            cleanup_errors.append(str(exc))
                    if self.safe_to_reuse:
                        self.right_robot = None
                        self.right_controller = None
                    if cleanup_errors:
                        self.last_error = '; '.join(cleanup_errors)
                        raise RuntimeError(self.last_error)

    def play(
        self,
        plan: Optional[ActionPlan],
        cancel_event: Optional[threading.Event] = None,
        document: Optional[ActionDocument] = None,
    ) -> bool:
        """执行一个双臂 EMI 动作计划。"""
        cancel = cancel_event or threading.Event()
        sdk = self._load_sdk()
        Controller = sdk.Controller
        Profile = getattr(sdk, "Profile", None)

        self.last_error = None
        self.safe_to_reuse = True
        self.diagnostic_codes = frozenset()
        self.diagnostics_complete = False
        self.recovery_outcome = None
        self.recovery_codes = frozenset()
        right_connected = False
        left_connected = False
        attempted = set()
        errors: List[str] = []
        scope = document.scope if document is not None else plan.scope if plan is not None else "dual"
        if scope not in {"dual", "right"}:
            raise ValueError(f"不支持的 EMI 动作 scope: {scope}")
        labels = ("左臂", "右臂") if scope == "dual" else ("右臂",)
        tracker = FeedbackTracker(labels)
        sender: Optional[DualServoJSender] = None
        motion_completed = False
        overrun_count = 0
        max_send_sec = 0.0
        late_tick_count = 0
        max_tick_gap_sec = 0.0
        last_tick_at = None
        stats_logged = False
        phase_intervals: Dict[str, List[float]] = {
            "起势": [], "主体": [], "收尾": [], "终点等待": [],
            "灵巧手过渡": [],
        }
        phase_last_tick: Dict[str, float] = {}
        trace_origin: Optional[float] = None
        trace_records: List[Tuple[Any, ...]] = []
        trace_frame = 0
        program_runner = None

        def wait_hand(seconds):
            if cancel.wait(seconds):
                raise RuntimeError("收到停止/急停请求，中断灵巧手动作")

        try:
            # 1. 选项 B：分别连接双臂独立 Controller
            self.right_controller = Controller()
            if scope == "dual":
                self.left_controller = Controller()
            require_ok(
                self.right_controller.connect(RIGHT_ARM_IP, CONTROLLER_PORT),
                "右臂 connect",
            )
            right_connected = True
            if scope == "dual":
                require_ok(
                    self.left_controller.connect(LEFT_ARM_IP, CONTROLLER_PORT),
                    "左臂 connect",
                )
                left_connected = True

            require_ok(self.right_controller.set_data_unit(1), "右臂 set_data_unit")
            if scope == "dual":
                require_ok(self.left_controller.set_data_unit(1), "左臂 set_data_unit")

            # 2. 独立添加两臂 robot=1 并配置参数 (完全对齐 TAIJI)
            self.right_robot = self.right_controller.add_robot(1)
            if scope == "dual":
                self.left_robot = self.left_controller.add_robot(1)
            require_attached(self.right_controller, self.right_robot, "右臂", 1)
            if scope == "dual":
                require_attached(self.left_controller, self.left_robot, "左臂", 1)

            require_ok(self.right_robot.set_speed(100), "右臂 set_speed")
            if scope == "dual":
                require_ok(self.left_robot.set_speed(100), "左臂 set_speed")

            if Profile is not None:
                require_ok(
                    self.right_robot.set_profile(Profile(*PROFILE_VALUES)),
                    "右臂 set_profile",
                )
                if scope == "dual":
                    require_ok(
                        self.left_robot.set_profile(Profile(*PROFILE_VALUES)),
                        "左臂 set_profile",
                    )

            require_ok(
                self.right_controller.idn_write("1203.150", "1.0.0", "4"),
                "右臂 idn_write",
            )
            if scope == "dual":
                require_ok(
                    self.left_controller.idn_write("1203.150", "1.0.0", "4"),
                    "左臂 idn_write",
                )

            # 注册反馈回调 (完全复用 TAIJI)
            self.right_robot.set_emi_feedback_callback(tracker.make_callback("右臂"))
            if scope == "dual":
                self.left_robot.set_emi_feedback_callback(tracker.make_callback("左臂"))

            if document is not None:
                current_l, current_r = self.get_current_joint_pos()
                if document.program:
                    from action_program import ProgramRunner
                    program_runner = ProgramRunner(document, current_r, self.chassis, cancel)
                    # Every branch/loop edge is interpolated and checked before power-on.
                    plan = program_runner.validation_plan()
                else:
                    plan = plan_action_taiji_style(document, current_l, current_r)
            if plan is None:
                raise RuntimeError("EMI 动作计划为空")

            self._validate_plan_limits(plan, self.left_robot, self.right_robot)
            if cancel.is_set():
                raise RuntimeError("收到停止/急停请求，中断动作")
            self._init_hands(plan.events, wait=wait_hand)
            if cancel.is_set():
                raise RuntimeError("收到停止/急停请求，中断动作")

            # 3. 创建双线程并行发送器
            sender = (DualServoJSender(self.left_robot, self.right_robot, plan.period)
                      if scope == "dual" else RightServoJSender(self.right_robot, plan.period))

            # 4. 上电使能与确认 (完全复用 TAIJI)
            arms = [("右臂", self.right_controller, self.right_robot)]
            if scope == "dual":
                arms.append(("左臂", self.left_controller, self.left_robot))
            for label, ctrl, robot in arms:
                require_ok(
                    ctrl.set_power_enable(True, 1, timeout=SDK_TIMEOUT_MS),
                    f"{label} 上电",
                )
                deadline = time.monotonic() + 5.0
                while power_state(robot) != 1:
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f"{label} 上电状态确认超时")
                    time.sleep(0.05)
                require_ok(robot.abort(), f"{label} abort")

            if self.event_callback is not None:
                self.event_callback('power', 'on')

            # 上电后姿态变化严格校验 (完全复用 TAIJI)
            initial_r = plan.trans_r[0]
            actual_r = validate_initial(get_arm_joints(self.right_robot))
            step_limit = plan.period * MAX_JOINT_SPEED_DEG_S
            if scope == "dual":
                initial_l = plan.trans_l[0]
                actual_l = validate_initial(get_arm_joints(self.left_robot))
                if max(abs(a - b) for a, b in zip(actual_l, initial_l)) > step_limit:
                    raise RuntimeError("左臂上电后姿态变化过大，停止启动")
            if max(abs(a - b) for a, b in zip(actual_r, initial_r)) > step_limit:
                raise RuntimeError("右臂上电后姿态变化过大，停止启动")

            if program_runner is not None:
                program_runner.prepare_chassis()

            # 5. 启动双臂 EMI 并在 0.5s 内校验启动反馈 (完全复用 TAIJI)
            tracker.start()
            startup = time.perf_counter()
            for label, _, robot in arms:
                attempted.add(label)
                require_ok(
                    robot.emi_start(
                        EMI_START_WORD, EMI_ONLINE_SCENE, plan.period, 0, 0
                    ),
                    f"{label} emi_start",
                )

            if time.perf_counter() - startup > FEEDBACK_STALE_SEC:
                raise RuntimeError("双臂 EMI 启动耗时超过反馈期限")

            # 6. tick 循环
            initial_events = [event for event in plan.events if event.time <= 0.0]
            events_by_frame = {}
            for ev in plan.events:
                if ev.time <= 0.0:
                    continue
                f_idx = max(0, int(ev.time / plan.period))
                events_by_frame.setdefault(f_idx, []).append(ev)

            body_frame_counter = 0
            hold_targets = {
                "left": plan.trans_l[0] if scope == "dual" else [],
                "right": plan.trans_r[0],
            }

            def tick(left: Sequence[float], right: Sequence[float], phase: str) -> None:
                nonlocal overrun_count, max_send_sec, late_tick_count
                nonlocal max_tick_gap_sec, last_tick_at
                nonlocal trace_origin, trace_frame
                if cancel.is_set():
                    raise RuntimeError("收到停止/急停请求，中断发包")
                faults = tracker.check_faults(labels)
                if faults:
                    raise RuntimeError("; ".join(faults))
                if sender is None:
                    raise RuntimeError("servo_j 发送器未初始化")

                now = time.perf_counter()
                if trace_origin is None:
                    trace_origin = now
                if phase in phase_last_tick:
                    phase_intervals[phase].append(now - phase_last_tick[phase])
                phase_last_tick[phase] = now
                if last_tick_at is not None:
                    gap = now - last_tick_at
                    max_tick_gap_sec = max(max_tick_gap_sec, gap)
                    if gap > plan.period * 1.25:
                        late_tick_count += 1
                last_tick_at = now

                elapsed = sender.send(left, right)
                hold_targets["left"], hold_targets["right"] = tuple(left), tuple(right)
                send_end = time.perf_counter()
                samples = tracker.latest_joint_samples(labels)
                trace_records.append((phase, trace_frame, "右臂", now, send_end,
                                      tuple(right), samples.get("右臂")))
                if scope == "dual":
                    trace_records.append((phase, trace_frame, "左臂", now, send_end,
                                          tuple(left), samples.get("左臂")))
                trace_frame += 1
                max_send_sec = max(max_send_sec, elapsed)
                if elapsed > plan.period:
                    overrun_count += 1
                    if overrun_count == 1 or overrun_count % 25 == 0:
                        print(
                            f"[servo_j周期超时] 双臂耗时={elapsed*1000:.3f}ms "
                            f"> 周期={plan.period*1000:.3f}ms；累计={overrun_count}，不补发",
                            file=sys.stderr,
                            flush=True,
                        )
                high_precision_delay(plan.period - elapsed)

            def hold_during_hand(seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    tick(hold_targets["left"], hold_targets["right"], "灵巧手过渡")

            for hand in (self.right_hand, self.left_hand):
                if hand is not None:
                    hand.wait = hold_during_hand

            if program_runner is not None:
                self.log(f"开始发送动作流程 [{plan.name}]...")
                program_runner.run(tick, self._dispatch_hand_event, tracker,
                                   notify=lambda phase: self.event_callback('phase', phase)
                                   if self.event_callback else None)
            else:
                # Existing wave and dual-arm playback remain unchanged.
                for event in initial_events:
                    self._dispatch_hand_event(event)
                self.log("开始发送起势过渡段...")
                if self.event_callback:
                    self.event_callback('phase', 'moving')
                for index, right in enumerate(plan.trans_r):
                    tick(plan.trans_l[index] if scope == "dual" else [], right, "起势")

                self.log(f"开始发送动作主体 [{plan.name}]...")
                for index, right in enumerate(plan.action_r[1:], start=1):
                    if body_frame_counter in events_by_frame:
                        for event in events_by_frame[body_frame_counter]:
                            self._dispatch_hand_event(event)
                    tick(plan.action_l[index] if scope == "dual" else [], right, "主体")
                    body_frame_counter += 1

                self.log("主体完成，开始发送收尾轨迹...")
                if self.event_callback and len(plan.return_r) > 1:
                    self.event_callback('phase', 'reset')
                for index, right in enumerate(plan.return_r[1:], start=1):
                    tick(plan.return_l[index] if scope == "dual" else [], right, "收尾")

            # 7. 动作终点到位反馈闭环轮询确认
            zero_r = plan.return_r[-1]
            zero_l = plan.return_l[-1] if scope == "dual" else []
            targets = {"右臂": zero_r}
            if scope == "dual":
                targets["左臂"] = zero_l
            deadline = time.perf_counter() + FINAL_SETTLE_SEC
            stable_since = None
            longest_stable_sec = 0.0
            self.log("正在等待编码器反馈稳定到达动作终点...")
            while True:
                tick(zero_l, zero_r, "终点等待")
                now = time.perf_counter()
                if tracker.at_target(targets):
                    stable_since = now if stable_since is None else stable_since
                    longest_stable_sec = max(longest_stable_sec, now - stable_since)
                    if now - stable_since >= 0.2:
                        self.log("动作臂已稳定到达终点 (连续 0.2s 误差<=1.0°)")
                        break
                else:
                    stable_since = None
                if now >= deadline:
                    raise RuntimeError(
                        "动作终点未在期限内到位："
                        f"最长连续到位={longest_stable_sec:.3f}s；"
                        f"{tracker.describe_target_error(targets)}"
                    )

            self.log(
                f"播放统计：发包调用超时={overrun_count}次，最大返回耗时={max_send_sec*1000:.3f}ms；"
                f"主循环间隔超限={late_tick_count}次，最大间隔={max_tick_gap_sec*1000:.3f}ms"
            )
            for detail in format_send_intervals(phase_intervals, plan.period):
                self.log(f"逐帧发送间隔：{detail}")
            stats_logged = True

            # 8. 停止发包后等待 2.0 秒伺服平息
            self.log(f"发包完成，停发等待 {SERVO_STOP_WAIT_SEC:.1f}s 伺服平息...")
            time.sleep(SERVO_STOP_WAIT_SEC)
            if cancel.is_set():
                raise RuntimeError("收到停止/急停请求，中断动作")
            motion_completed = True

        except Exception as exc:
            error = str(exc)
            if program_runner is not None and program_runner.yaw_started:
                reason = program_runner.chassis.chassis_failure_reason()
                if reason is not None:
                    error = "底盘控制失败: " + reason
            self.log(f"执行异常退出: {error}")
            errors.append(error)
            self.last_error = error
        finally:
            # After sender.close(), no hand cleanup may re-enter the EMI sender.
            for hand in (self.right_hand, self.left_hand):
                if hand is not None:
                    hand.wait = wait_hand
            if program_runner is not None:
                try:
                    if not program_runner.stop_chassis():
                        errors.append("底盘线程退出未确认")
                        self.safe_to_reuse = False
                except Exception as exc:
                    errors.append(f"底盘停止失败: {exc}")
                    self.safe_to_reuse = False
            if sender is not None and not stats_logged:
                self.log(
                    f"播放统计（异常）：发包调用超时={overrun_count}次，"
                    f"最大返回耗时={max_send_sec*1000:.3f}ms；"
                    f"主循环间隔超限={late_tick_count}次，"
                    f"最大间隔={max_tick_gap_sec*1000:.3f}ms"
                )
                if plan is not None:
                    for detail in format_send_intervals(phase_intervals, plan.period):
                        self.log(f"逐帧发送间隔（异常）：{detail}")
            if sender is not None:
                try:
                    sender.close()
                except Exception as exc:
                    errors.append(f"sender 关闭异常: {exc}")
                    self.safe_to_reuse = False

            for label, controller, hand in (
                ("右手", self.right_controller, self.right_hand),
                ("左手", self.left_controller, self.left_hand),
            ):
                if hand is None or controller is None:
                    continue
                try:
                    if motion_completed and not cancel.is_set():
                        try:
                            hand.open()
                            time.sleep(0.2)
                        except RuntimeError as exc:
                            if not (cancel.is_set() and "收到停止/急停请求" in str(exc)):
                                raise
                            # Expected cancellation during the thumb lead must
                            # still close COM2 and perform normal power cleanup.
                    require_ok(controller.com_close("com2"), f"{label}关闭串口")
                except Exception as exc:
                    errors.append(f"{label}清理失败: {exc}")
                    self.safe_to_reuse = False

            # 9. safe_cleanup_arm
            confirmed_off = True
            for label, ctrl, robot, connected in (
                ("右臂", self.right_controller, self.right_robot, right_connected),
                ("左臂", self.left_controller, self.left_robot, left_connected),
            ):
                if not connected:
                    continue
                if label in attempted:
                    ok = safe_cleanup_arm(label, ctrl, robot, errors, emi_attempted=True)
                    confirmed_off = confirmed_off and ok
                elif ctrl is not None:
                    ok = safe_cleanup_arm(label, ctrl, robot, errors, emi_attempted=False)
                    confirmed_off = confirmed_off and ok

            if confirmed_off:
                if self.event_callback and (right_connected or left_connected):
                    self.event_callback('power', 'off')
                # Read diagnostics only after motion and power-off confirmation.
                diagnostic_reads_ok = True
                diagnostic_by_arm = {}
                for label, ctrl, connected in (
                    ("右臂", self.right_controller, right_connected),
                    ("左臂", self.left_controller, left_connected),
                ):
                    if not connected or ctrl is None:
                        continue
                    try:
                        diagnostic = ctrl.check_error(timeout=1000)
                        self.diagnostic_codes |= extract_error_codes(diagnostic)
                        diagnostic_by_arm[label] = repr(diagnostic)
                        self.log(f"{label} 控制器诊断 check_error={diagnostic!r}")
                    except Exception as exc:
                        diagnostic_reads_ok = False
                        self.log(f"{label} 控制器诊断读取失败（不影响动作结果）：{exc}")
                self.diagnostics_complete = (diagnostic_reads_ok and right_connected
                                             and (scope != "dual" or left_connected))
                # Recovery has exclusive SDK ownership only after sender.close(),
                # chassis stop and confirmed power-off. No changes to normal EMI ticks.
                from action_faults import LIMIT_CODES
                from controller_guard import recovery_arm
                from limit_recovery import LimitRecovery, ActionCancelled, UnsafeToDisconnect
                combined = self.diagnostic_codes | extract_error_codes(self.last_error or '')
                allowed_codes = LIMIT_CODES | {-5029}
                recover_now = (bool(combined & LIMIT_CODES)
                               and not (combined - allowed_codes)
                               and self.safe_to_reuse and self.diagnostics_complete
                               and Profile is not None
                               and callable(self.recovery_permitted)
                               and not cancel.is_set() and self.recovery_permitted())
                if recover_now:
                    recovery_arms = []
                    for label, ctrl, robot, connected in (
                        ('右臂', self.right_controller, self.right_robot, right_connected),
                        ('左臂', self.left_controller, self.left_robot, left_connected),
                    ):
                        if connected and robot is not None:
                            recovery_arms.append(recovery_arm(label, ctrl, robot,
                                                             diagnostic_by_arm.get(label, '')))
                    recovery = LimitRecovery(recovery_arms,
                                             Profile(*PROFILE_VALUES) if Profile is not None else None,
                                             cancel, self.recovery_permitted, self.log)
                    try:
                        for arm in recovery_arms:
                            arm.robot.emi_enable(False)
                            if str(arm.controller.target.idn_read('1203.1', '1.0.0')).strip() != '0':
                                raise RuntimeError(arm.label + ' EMI未确认退出，拒绝Jog')
                        recovered = recovery.recover_verified_limit()
                        self.recovery_outcome = 'limit_recovered' if recovered else 'limit_recovery_failed'
                    except ActionCancelled:
                        self.recovery_outcome = 'limit_recovery_failed'
                    except Exception as exc:
                        self.recovery_outcome = 'limit_recovery_failed'
                        self.recovery_codes |= extract_error_codes(exc)
                        self.log(f'限位恢复异常: {exc}')
                        if isinstance(exc, UnsafeToDisconnect):
                            self.safe_to_reuse = False
                    finally:
                        self.recovery_codes |= recovery._last_recovery_codes
                        # Recovery can re-enable power; repeat the verified shutdown
                        # before any disconnect, even when recovery was cancelled.
                        for label, ctrl, robot, connected in (
                            ('右臂', self.right_controller, self.right_robot, right_connected),
                            ('左臂', self.left_controller, self.left_robot, left_connected),
                        ):
                            if connected and not safe_cleanup_arm(label, ctrl, robot, errors, emi_attempted=False):
                                confirmed_off = False
                                self.safe_to_reuse = False
                if not confirmed_off:
                    self.log('恢复后下电未确认，禁止断连')
                for label, ctrl, connected in (
                    ("右臂", self.right_controller, right_connected),
                    ("左臂", self.left_controller, left_connected),
                ):
                    if not connected or not confirmed_off:
                        continue
                    try:
                        require_ok(ctrl.disconnect(), f"{label} disconnect")
                    except Exception as exc:
                        errors.append(f"{label} 断连失败: {exc}")
                        self.safe_to_reuse = False
                if confirmed_off:
                    self.log("动作臂已确认下电并安全断连")
            elif right_connected or left_connected:
                self.log("下电未确认：禁止主动断连！需要排查")
                self.safe_to_reuse = False

            if trace_records and trace_origin is not None and plan is not None:
                try:
                    trace_path = write_playback_trace(
                        plan.name, trace_origin, trace_records, TRACE_OUTPUT_DIR,
                    )
                    self.log(f"逐帧诊断文件：{trace_path}")
                    feedback_path = trace_path.with_name(
                        trace_path.stem + "_feedback.csv"
                    )
                    feedback_samples = tracker.feedback_history()
                    write_feedback_trace(feedback_path, trace_origin, feedback_samples)
                    self.log(f"EMI原始反馈文件：{feedback_path}（{len(feedback_samples)}条）")
                except Exception as exc:
                    self.log(f"逐帧诊断文件写入失败（不影响动作结果）：{exc}")

        if errors and self.last_error is None:
            self.last_error = "; ".join(errors)
        return len(errors) == 0
