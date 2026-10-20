"""动作服务的状态机与错误码映射（不依赖 ROS）。

``action_client.ActionServiceClient`` 只负责提供 publish/wait/status 三个原语，
所有判定逻辑都在这里，便于离线单测。
"""

from __future__ import annotations

from typing import Callable, Optional
from collections import deque
import threading
import time


class StatusMailbox:
    """Keep every transition after publish, including fast busy->ready pairs."""
    def __init__(self, clock=time.monotonic, max_age=3.0):
        self.condition = threading.Condition()
        self.clock, self.max_age = clock, max_age
        self.events = deque(maxlen=1024)
        self.sequence = self.cursor = 0
        self.latest = None
        self.received_at = None

    def put(self, value):
        with self.condition:
            self.sequence += 1
            self.latest, self.received_at = value, self.clock()
            self.events.append((self.sequence, value))
            self.condition.notify_all()

    def begin(self):
        with self.condition:
            self.cursor = self.sequence

    def status(self):
        with self.condition:
            if self.received_at is None or self.clock() - self.received_at > self.max_age:
                return None
            return self.latest

    def wait(self, predicate, timeout):
        deadline = self.clock() + timeout
        with self.condition:
            while True:
                for sequence, value in self.events:
                    if sequence > self.cursor:
                        self.cursor = sequence
                        if predicate(value):
                            return value
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return None
                self.condition.wait(min(remaining, .1))


READY = "ready"
BUSY = "busy"
RELEASING_LIMIT = "releasing_limit"
LIMIT_RECOVERY_FAILED = "limit_recovery_failed"

# 服务状态 → 游戏错误码。只列“不能继续”的终态；ready/busy/releasing_limit 不在此表，
# 它们由调用方按上下文判断（busy 是“忙”，releasing_limit 是脱限进行中）。
STATUS_ERROR_CODES = {
    "physical_alarm": "EMERGENCY_STOP",
    "recover_required": "EMERGENCY_STOP",
    "power_not_ready": "POWER_NOT_READY",
    "connection_failed": "SERVICE_UNAVAILABLE",
    "handle_busy": "SERVICE_UNAVAILABLE",
    "limit_recovery_failed": "OVERTRAVEL",
    "invalid_target": "ACTION_REJECTED",
    "unknown_command": "ACTION_REJECTED",
    "single_not_support": "ACTION_REJECTED",
    "fatal_network": "ACTION_FAILED",
    "failed": "ACTION_FAILED",
}


class ActionServiceError(RuntimeError):
    """携带稳定错误码的动作服务失败。"""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def error_code_for(status: Optional[str]) -> Optional[str]:
    """把服务状态映射为游戏错误码；ready/瞬时态返回 None。"""
    return STATUS_ERROR_CODES.get(status) if status is not None else None


class ActionSequencer:
    """不依赖 ROS 的动作状态机。

    参数:
        publish: 把命令字符串发给动作服务。
        wait_for: ``wait_for(predicate, timeout)``，返回满足条件的第一个状态，
            超时返回 None。
        status: 返回当前缓存状态；从未收到过返回 None。
    """

    def __init__(
        self,
        publish: Callable[[str], None],
        wait_for: Callable[[Callable[[str], bool], float], Optional[str]],
        status: Callable[[], Optional[str]],
    ) -> None:
        self._publish = publish
        self._wait_for = wait_for
        self._status_getter = status

    def status(self) -> Optional[str]:
        """返回当前缓存的服务状态；从未收到过返回 None。"""
        return self._status_getter()

    def ensure_ready(self) -> str:
        """当前状态必须是 ready，否则抛出对应错误码。"""
        current = self.status()
        if current is None:
            raise ActionServiceError(
                "SERVICE_UNAVAILABLE", "尚未收到动作服务状态，服务可能未启动")
        if current != READY:
            if current in (BUSY, RELEASING_LIMIT):
                code = "ROBOT_BUSY"
            else:
                code = error_code_for(current) or "SERVICE_UNAVAILABLE"
            raise ActionServiceError(code, f"动作服务状态为 {current}")
        return current

    def play(self, name: str, start_timeout: float, total_timeout: float) -> str:
        """发布动作名并等待 busy → ready。

        启动超时抛 ``ACTION_NOT_STARTED``；执行中进入故障状态按映射抛错；
        整体超时抛 ``ACTION_TIMEOUT``。
        """
        self.ensure_ready()
        self._publish(name)

        started = self._wait_for(lambda value: value != READY, start_timeout)
        if started is None:
            raise ActionServiceError(
                "ACTION_NOT_STARTED", f"{name} 未在 {start_timeout:.0f}s 内开始执行")
        code = error_code_for(started)
        if code is not None:
            raise ActionServiceError(code, f"{name} 启动失败：服务状态 {started}")

        finished = self._wait_for(
            lambda value: value == READY or error_code_for(value) is not None,
            total_timeout,
        )
        if finished is None:
            raise ActionServiceError(
                "ACTION_TIMEOUT", f"{name} 未在 {total_timeout:.0f}s 内完成")
        if finished != READY:
            code = error_code_for(finished) or "ACTION_FAILED"
            raise ActionServiceError(code, f"{name} 执行失败：服务状态 {finished}")
        return finished

    def recover_limits(self, timeout: float, start_timeout: float = 5.0) -> str:
        """发布 recover_limit 并等待 ready。

        先等状态离开 ready（正常应为 releasing_limit），避免把发布前的缓存 ready
        误当成“已完成”；随后再等回 ready 或故障态。
        """
        self.ensure_ready()
        self._publish("recover_limit")
        started = self._wait_for(lambda value: value != READY, start_timeout)
        if started is None:
            raise ActionServiceError(
                "ACTION_NOT_STARTED", f"解除限位未在 {start_timeout:.0f}s 内开始")
        code = error_code_for(started)
        if code is not None:
            raise ActionServiceError(code, f"解除限位失败：服务状态 {started}")

        finished = self._wait_for(
            lambda value: value == READY or error_code_for(value) is not None,
            timeout,
        )
        if finished is None:
            raise ActionServiceError(
                "ACTION_TIMEOUT", f"解除限位未在 {timeout:.0f}s 内完成")
        if finished != READY:
            code = error_code_for(finished) or "ACTION_FAILED"
            raise ActionServiceError(code, f"解除限位失败：服务状态 {finished}")
        return finished

    def stop(self) -> None:
        """请求中止当前动作；服务端不排队，重复调用无副作用。"""
        self._publish("stop")
