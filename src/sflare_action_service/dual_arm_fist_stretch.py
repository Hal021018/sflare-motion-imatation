#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS entry point for the fixed actions owned by the EMI test service."""

import json
import os
import threading
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String

from action_pipeline import ActionDocument, ActionRepository, Keyframe
from emi_coordinator import EmiCoordinator
from action_session import ActionSession, PRIORITY_ZERO_BLOCKING_CODES, extract_error_codes
from action_faults import active_fault_codes, failure_status, NETWORK_FATAL_CODES
from controller_guard import probe_available
from emi_coordinator import RIGHT_ARM_IP, LEFT_ARM_IP, CONTROLLER_PORT


TOPIC_NAME = "/dual_arm_xcdl_action"
STATUS_TOPIC = "/dual_arm_xcdl_action_status"
E_STOP_TOPIC = "/e_stop"
SHUTDOWN_JOIN_TIMEOUT_SEC = 10.0

# Keep the command names and aliases used by the production service.
ACTION_NAMES: Dict[str, str] = {
    "fist": "fist", "抱拳": "fist",
    "stretch": "stretch", "伸懒腰": "stretch",
    "stretch_ready": "stretch_ready", "伸懒腰准备": "stretch_ready",
    "wave": "wave", "挥手": "wave", "bnzf": "wave",
    "handshake": "handshake", "握手": "handshake",
    "like": "like", "点赞": "like",
    "forward_wave": "forward_wave", "forward+wave": "forward_wave",
    "dance": "dance", "跳舞": "dance",
    "sway": "sway", "左右摇摆": "sway",
    "heart": "heart", "比心": "heart",
    "forward": "forward", "前进推行": "forward",
    "turn_grip": "turn_grip", "P7": "turn_grip", "P7动作": "turn_grip",
    "doubt": "doubt", "疑问": "doubt",
    "raise_head": "raise_head", "举手过头": "raise_head",
    "shakehand": "shakehand", "连续握手": "shakehand",
    "slight_raise": "slight_raise", "微抬": "slight_raise",
    "diagonal_point": "diagonal_point", "斜前方指": "diagonal_point",
    "chin_rest": "chin_rest", "托腮": "chin_rest",
    "handover": "handover", "前伸递物": "handover",
}
DUAL_ACTIONS = frozenset({"fist", "stretch", "stretch_ready"})
STOP_COMMANDS = frozenset({"stop", "cancel", "停止", "急停", "emergency_stop", "emergency stop"})
GOZERO_COMMANDS = frozenset({"gozero", "go zero", "zero", "回零", "复位", "返回", "reset"})
RECOVER_COMMANDS = frozenset({"recover", "power_recover", "恢复", "恢复上电"})
# 按需脱限：把停在软限位外的轴 Jog 回限位内，不回零。
RECOVER_LIMIT_COMMANDS = frozenset({"recover_limit", "limit_recover", "脱限", "恢复限位"})


def limit_recovery_params() -> dict:
    """从环境变量读取按需脱限参数；非法值直接拒绝，避免带病运动。"""
    def positive(name: str, default: str) -> float:
        raw = os.environ.get(name, default).strip()
        value = float(raw)
        if not value > 0:
            raise ValueError(f"{name} 必须为正数，当前={raw!r}")
        return value

    return {
        "margin": positive("GAME_LIMIT_MARGIN_DEG", "3.0"),
        "jog_speed": positive("GAME_LIMIT_JOG_SPEED_PERCENT", "50.0"),
        "poll": positive("GAME_LIMIT_POLL_SEC", "0.05"),
        "max_seconds": positive("GAME_LIMIT_MAX_SECONDS", "20.0"),
    }


def arm_mode() -> str:
    mode = os.environ.get("PALLAS_ARM_MODE", "").strip().lower()
    if mode not in {"single", "dual"}:
        raise RuntimeError("PALLAS_ARM_MODE must be explicitly set to single or dual")
    return mode


def decode_command(raw_data: str) -> Tuple[str, str]:
    raw = raw_data.strip()
    try:
        payload = json.loads(raw)
    except (TypeError, ValueError):
        return raw, "auto"
    if not isinstance(payload, dict) or not isinstance(payload.get("command"), str):
        return raw, "auto"
    return payload["command"].strip(), str(payload.get("target", "auto")).strip().lower()


def zero_document(scope: str) -> ActionDocument:
    """The original gozero command remains a system command, not a named pose file."""
    zero = (0.0,) * 7
    return ActionDocument(
        schema_version="1.0.0", name="gozero", scope=scope, units="deg",
        period=0.040, speed_scale=1.0, auto_return_zero=False,
        keyframes=(Keyframe(time=0.0, left=zero if scope == "dual" else (), right=zero),),
        events=(), file_path="<system:gozero>", content_hash="",
    )


class EmiActionNode(Node):
    """Own one action session and let its worker own every SDK call."""

    def __init__(self) -> None:
        super().__init__("emi_fixed_action_service")
        self.mode = arm_mode()
        self.repository = ActionRepository(Path(__file__).resolve().parent / "actions")
        from chassis_control import ChassisController
        # Subscribe at startup so a chassis action can require already-fresh odometry.
        self.chassis = ChassisController(self)
        self._lock = threading.Lock()
        self._session: Optional[ActionSession] = None
        self._stopping = False
        self._e_stop_pressed: Optional[bool] = None
        self._recover_required = False
        self._fault_alarm_latched = False
        self._status_text = "ready"
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, STATUS_TOPIC, qos)
        self.event_publisher = self.create_publisher(String, '/dual_arm_xcdl_action_events', 50)
        self.create_subscription(String, TOPIC_NAME, self._on_command, 10)
        self.create_subscription(Int32, E_STOP_TOPIC, self._on_e_stop, qos)
        self.create_timer(1.0, self._republish_status)
        self._publish_locked("ready")

    @property
    def _worker(self):
        # Read-only compatibility for the existing one-shot diagnostic helper.
        return self._session.thread if self._session is not None else None

    def _publish_action_event(self, action, kind, state):
        """Bounded phase events from execution/verified power state, never log parsing."""
        try:
            message = String()
            message.data = json.dumps({'action': action, 'type': kind, 'state': state})
            self.event_publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning('动作事件发布失败: '+str(exc))

    @property
    def _cancel(self):
        return self._session.cancel_event if self._session is not None else None

    def _alarm_locked(self):
        if self._e_stop_pressed is True or getattr(self, '_fault_alarm_latched', False):
            return "physical_alarm"
        return "recover_required" if self._recover_required else None

    def _publish_locked(self, status: str) -> None:
        if self._stopping:
            return
        if status in {"ready", "busy"}:
            status = self._alarm_locked() or status
        self._status_text = status
        message = String()
        message.data = status
        self.publisher.publish(message)

    def _republish_status(self) -> None:
        with self._lock:
            if not self._stopping and self._status_text != "handle_busy":
                self._publish_locked(self._status_text)

    def _on_e_stop(self, message: Int32) -> None:
        if message.data not in (0, 1):
            return
        pressed = message.data == 1
        with self._lock:
            if self._stopping:
                return
            previous = self._e_stop_pressed
            self._e_stop_pressed = pressed
            if pressed:
                self._recover_required = True
                session = self._session
                if session is not None:
                    session.e_stop_interrupted = True
                    session.recover_after_cleanup = False
                    session.intent = "stop"
                    session.cancel_event.set()
                self._publish_locked("physical_alarm")
            elif previous is True:
                self._publish_locked("recover_required")
        if pressed:
            # Original ROS-only chassis stop notification; no SDK in callbacks.
            self.chassis._publish_chassis_speed(0.0, 0.0)

    def _acknowledge_recover_locked(self) -> bool:
        if self._e_stop_pressed is not False:
            return False
        self._recover_required = False
        self._fault_alarm_latched = False
        return True

    def _start_session_locked(self, command: str, action: str, scope=None):
        if scope is None:
            scope = ("dual" if self.mode == "dual" else "right") if action == "gozero" else (
                "dual" if action in DUAL_ACTIONS else "right")
        session = ActionSession(command, action, scope=scope, zeroing=action == "gozero")
        session.thread = threading.Thread(
            target=self._run, args=(session,), name="emi-action-" + action, daemon=True)
        self._session = session
        # 脱限是对外的独立流程，状态上与普通 busy 区分开
        self._publish_locked("releasing_limit" if action == "recover_limit" else "busy")
        try:
            session.thread.start()
        except Exception:
            self._session = None
            self._publish_locked("failed")
            raise
        return session

    def _request_priority_locked(self, command: str, intent: str):
        alarm = self._alarm_locked()
        if intent == "gozero" and alarm is not None:
            self._publish_locked(alarm)
            return
        session = self._session
        if session is None:
            if intent == "stop":
                self._publish_locked("ready")
            else:
                self._start_session_locked(command, "gozero")
            return
        if intent == "stop":
            session.intent = "stop"
            session.cancel_event.set()
        elif session.intent != "stop" and not session.zeroing:
            session.intent = "gozero"
            session.cancel_event.set()
        # stop dominates gozero; repeated zero requests are merged.
        self._publish_locked("busy")

    def _on_command(self, message: String) -> None:
        command, target = decode_command(message.data)
        if not command:
            return
        action = ACTION_NAMES.get(command)
        with self._lock:
            if self._stopping:
                return
            if (self._session is not None and self._session.monitoring_connection
                    and command not in STOP_COMMANDS):
                # No command queue or parallel SDK owner during availability probing.
                return
            if action in DUAL_ACTIONS and self.mode == "single":
                self._publish_locked(self._alarm_locked() or (
                    "busy" if self._session is not None else "single_not_support"))
                return
            allowed = {"auto", "all", "right"}
            allowed |= {"dual", "dual_arm"} if self.mode == "dual" else {"single", "single_arm"}
            if target not in allowed:
                return
            if command in STOP_COMMANDS:
                self._request_priority_locked(command, "stop")
                return
            if command in GOZERO_COMMANDS:
                self._request_priority_locked(command, "gozero")
                return
            if command in RECOVER_LIMIT_COMMANDS:
                if self._alarm_locked():
                    self._publish_locked(self._alarm_locked())
                elif self._session is not None:
                    # 同一时刻只允许一个 SDK 所有者；脱限不排队
                    self._publish_locked("busy")
                else:
                    self._start_session_locked(command, "recover_limit", scope="right")
                return
            if command in RECOVER_COMMANDS:
                session = self._session
                if self._e_stop_pressed is not False:
                    self._publish_locked(self._alarm_locked() or "recover_required")
                elif session is not None and session.e_stop_interrupted:
                    session.recover_after_cleanup = True
                    self._publish_locked("recover_required")
                elif session is not None:
                    self._publish_locked("busy")
                elif self._acknowledge_recover_locked():
                    # Acknowledge only; no SDK/clear_alarm/power/movement.
                    self._publish_locked("ready")
                return
            if self._alarm_locked():
                self._publish_locked(self._alarm_locked())
            elif self._session is not None:
                self._publish_locked("busy")
            elif action is None:
                self._publish_locked("unknown_command")
            else:
                self._start_session_locked(command, action)

    def _failure_status(self, error, diagnostic_codes=()):
        return failure_status(error, diagnostic_codes, self._e_stop_pressed)

    def _recovery_permitted(self, session):
        with self._lock:
            return (self._session is session and not self._stopping
                    and self._e_stop_pressed is False and not self._recover_required
                    and not session.cancel_event.is_set())

    def _exit_on_fatal(self, status):
        self.get_logger().fatal(status + ': 安全清理完成，退出进程；由启动管理器决定重启')
        with self._lock:
            self._publish_locked(status)
            self._stopping = True
        os._exit(1)

    def _monitor_connection(self, session, coordinator):
        targets = [('右臂', RIGHT_ARM_IP)]
        if self.mode == 'dual':
            targets.append(('左臂', LEFT_ARM_IP))
        with self._lock:
            session.monitoring_connection = True
        while not session.cancel_event.is_set():
            with self._lock:
                if self._stopping or self._session is not session or self._alarm_locked():
                    return self._alarm_locked() or 'recover_required'
            state, safe = probe_available(coordinator._load_sdk().Controller, targets, CONTROLLER_PORT)
            if not safe:
                self._exit_on_fatal('unsafe_stop')
                return 'unsafe_stop'
            with self._lock:
                if self._stopping or self._session is not session or self._alarm_locked():
                    return self._alarm_locked() or 'recover_required'
                if state == 'ready':
                    return 'ready'
                if self._status_text != state:
                    self._publish_locked(state)
            session.cancel_event.wait(5.0)
        return 'ready'

    def _finish_session_locked(self, session, status):
        if self._session is not session:
            return
        if session.e_stop_interrupted and status in {'ready', 'recover_required'}:
            if (session.recover_after_cleanup and not self._stopping
                    and self._acknowledge_recover_locked()):
                status = "ready"
            else:
                status = self._alarm_locked() or "recover_required"
        if status == "physical_alarm":
            self._recover_required = True
            self._fault_alarm_latched = True
        self._session = None
        self._publish_locked(self._alarm_locked() or status)

    def _run_limit_recovery(self, session: ActionSession) -> None:
        """按需脱限会话：独立于动作播放，结束后把状态收敛为 ready/失败。"""
        status = "limit_recovery_failed"
        coordinator = None
        try:
            params = limit_recovery_params()
            coordinator = EmiCoordinator(logger=self.get_logger().info)
            coordinator.recovery_permitted = lambda: self._recovery_permitted(session)
            result = coordinator.recover_limits_on_demand(session.cancel_event, **params)
            status = "ready"
            self.get_logger().info(
                f"按需脱限完成: axes={result['recovered_axes']} "
                f"speed={result['jog_speed_percent']}% margin={result['margin_deg']}°")
        except Exception as exc:
            self.get_logger().error(f"按需脱限失败: {exc}")
            status = "limit_recovery_failed"
        finally:
            if coordinator is not None and not coordinator.safe_to_reuse:
                self._exit_on_fatal('unsafe_stop')
                return
            with self._lock:
                if self._session is session:
                    self._finish_session_locked(session, status)

    def _run(self, session: ActionSession) -> None:
        status = "failed"
        try:
            while True:
                with self._lock:
                    if self._session is not session:
                        return
                if session.action == "recover_limit":
                    self._run_limit_recovery(session)
                    return
                document = (zero_document(session.scope) if session.action == "gozero"
                            else self.repository.get(session.action))
                from motion_backend import coordinator_class
                coordinator = coordinator_class(session.action, session.scope, emi_class=EmiCoordinator)(logger=self.get_logger().info)
                coordinator.event_callback = lambda kind, state: self._publish_action_event(
                    session.action, kind, state)
                coordinator.chassis = self.chassis
                coordinator.recovery_permitted = lambda: self._recovery_permitted(session)
                completed = coordinator.play_document(document, session.cancel_event)
                if not coordinator.safe_to_reuse:
                    self.get_logger().fatal("动作控制器清理未确认")
                    with self._lock:
                        self._stopping = True
                    os._exit(1)
                    return
                error = coordinator.last_error or ""
                raw_codes = extract_error_codes(error) | coordinator.diagnostic_codes
                recovery_codes = getattr(coordinator, 'recovery_codes', frozenset())
                all_codes = raw_codes | recovery_codes
                estop_timeout = (session.e_stop_interrupted and -5029 in all_codes
                                 and all_codes <= {-5029, -20039})
                if all_codes & NETWORK_FATAL_CODES and not estop_timeout:
                    self._exit_on_fatal('fatal_network')
                    return
                if (-20035 in all_codes and not all_codes &
                        {-5201, -135, -5176, -161, -20236, -20038, -20039}):
                    status = self._monitor_connection(session, coordinator)
                    with self._lock:
                        self._finish_session_locked(session, status)
                    return
                with self._lock:
                    if self._session is not session:
                        return
                    codes = active_fault_codes(
                        all_codes,
                        self._e_stop_pressed)
                    blocking_codes = codes & (PRIORITY_ZERO_BLOCKING_CODES | {-5029})
                    recovery_outcome = getattr(coordinator, 'recovery_outcome', None)
                    if recovery_codes:
                        status = self._failure_status('', recovery_codes)
                    elif codes & {-5201, -135}:
                        session.recover_after_cleanup = False
                        status = 'physical_alarm'
                    elif self._stopping or session.e_stop_interrupted or self._alarm_locked():
                        status = self._alarm_locked() or "recover_required"
                    elif recovery_outcome is not None:
                        status = recovery_outcome
                    elif (session.intent == "gozero" and not session.zeroing
                          and coordinator.diagnostics_complete
                          and not blocking_codes
                          and "底盘控制失败" not in error):
                        # Cleanup is confirmed; keep this worker as sole SDK owner.
                        # Clear only the consumed request under the same state lock.
                        session.zeroing = True
                        session.intent = "none"
                        session.action = "gozero"
                        session.cancel_event.clear()
                        continue
                    elif blocking_codes:
                        status = self._failure_status(error, codes)
                        self.get_logger().error(f"控制器故障 codes={sorted(codes)}: {error}")
                    elif completed or (session.intent == "stop"
                                       and "收到停止/急停请求" in error):
                        status = "ready"
                    else:
                        status = self._failure_status(error)
                        if error:
                            self.get_logger().error(error)
                    if (recovery_outcome is None and session.intent == "gozero" and not session.zeroing
                            and blocking_codes):
                        status = self._failure_status(str(sorted(codes)))
                    elif (recovery_outcome is None and session.intent == "gozero" and not session.zeroing
                          and not coordinator.diagnostics_complete
                          and not session.e_stop_interrupted and not self._alarm_locked()):
                        self.get_logger().error("优先级回零预检未完成；拒绝建立新的回零会话")
                        status = "failed"
                    self._finish_session_locked(session, status)
                    return
        except Exception as exc:
            self.get_logger().error(f"{session.action} failed: {exc}")
            status = self._failure_status(exc)
        finally:
            # Identity guard prevents a stale worker overwriting a newer session.
            with self._lock:
                if self._session is session:
                    self._finish_session_locked(session, status)

    def shutdown(self) -> bool:
        with self._lock:
            self._stopping = True
            session = self._session
            if session is not None:
                session.intent = "stop"
                session.cancel_event.set()
            worker = session.thread if session is not None else None
        if worker is not None:
            worker.join(timeout=SHUTDOWN_JOIN_TIMEOUT_SEC)
        return worker is None or not worker.is_alive()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmiActionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.shutdown():
            os._exit(1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
