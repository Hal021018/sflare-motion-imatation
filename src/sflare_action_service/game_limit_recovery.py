"""按需脱限：把越过控制器软限位的轴 Jog 回限位内，不回零。

与 ``limit_recovery.py`` 的区别：

- ``limit_recovery`` 是**动作失败后**的自动补救，必须先有 -5031/-5032 报警证据，
  且只在 EMI 会话收尾时由协调器调用。
- 本模块是**按需**的：游戏开局前 ``self_check`` 发现某轴停在限位外，就发
  ``recover_limit`` 命令，直接按“当前位置 vs 控制器限位”判定方向和目标，
  不要求报警码。

安全前提（每条都必须在代码里成立，否则不运动）：

1. 控制器明确报告急停已释放（``get_status()[0] == 0``）；未知或读取失败一律拒绝。
2. 只对**确实越界**的轴动作：``position < lower`` 才向正方向，``position > upper``
   才向负方向；在限位内的轴绝不触碰。
3. 方向由越界侧唯一确定，不存在“猜方向”。
4. 连续 Jog + 位置轮询，进入 ``lower+margin`` / ``upper-margin`` 立即停；
   每个轮询都重新读急停。
5. 单轴最长 ``max_seconds``，另有“无进展”保护；任何异常路径都先 ``move_stop_jog``。
6. 不续跑任何动作、不回零、不改变姿态目标。

调用方（``EmiCoordinator.recover_limits_on_demand``）负责上电、Jog 模式与退出清理。
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable, List, Optional, Sequence, Tuple


DEFAULT_MARGIN_DEG = 3.0
DEFAULT_JOG_SPEED_PERCENT = 50.0
DEFAULT_POLL_SEC = 0.05
DEFAULT_MAX_SECONDS_PER_AXIS = 20.0
STAGNANT_POLLS_LIMIT = 40          # 40 * 50ms = 2s 无位移即判无进展
STAGNANT_EPSILON_DEG = 0.02


class LimitRecoveryError(RuntimeError):
    """按需脱限失败，调用方必须保持服务不可用状态。"""


class GameLimitRecovery:
    """在已上电、已进入关节 Jog 模式的连接上执行按需脱限。"""

    def __init__(
        self,
        controller: Any,
        robot: Any,
        log: Optional[Callable[[str], None]] = None,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.controller = controller
        self.robot = robot
        self.log = log or (lambda msg: None)
        self._monotonic = monotonic
        self._sleep = sleep

    # ---- 状态读取 -------------------------------------------------------
    def read_state(self) -> Tuple[List[float], List[Tuple[float, float]], int]:
        """读取当前 7 轴位置与逐轴限位；数据异常直接拒绝动作。"""
        axis_count = int(self.robot.get_axis_number())
        if axis_count <= 0:
            raise LimitRecoveryError(f"控制器轴数异常: {axis_count}")
        positions = [float(value) for value in self.robot.get_joint_pos()][:axis_count]
        if len(positions) != axis_count or not all(map(math.isfinite, positions)):
            raise LimitRecoveryError(f"关节位置无效: {positions}")
        limits: List[Tuple[float, float]] = []
        for axis in range(1, axis_count + 1):
            raw = [float(value) for value in self.robot.get_limit(axis)]
            if len(raw) < 2 or not all(map(math.isfinite, raw[:2])):
                raise LimitRecoveryError(f"J{axis} 限位反馈无效: {raw}")
            limits.append((min(raw[0], raw[1]), max(raw[0], raw[1])))
        return positions, limits, axis_count

    @staticmethod
    def overtravel_axes(
        positions: Sequence[float],
        limits: Sequence[Tuple[float, float]],
    ) -> List[Tuple[int, float, float, float]]:
        """返回 (轴号, 当前位置, 下限, 上限)，只包含**确实越界**的轴。"""
        result = []
        for axis, (position, (lower, upper)) in enumerate(zip(positions, limits), start=1):
            if position < lower or position > upper:
                result.append((axis, position, lower, upper))
        return result

    def _validate_emergency_stop(self) -> None:
        """急停必须由控制器明确报告为释放；未知值按危险处理。"""
        status = self.controller.get_status()
        if not status:
            raise LimitRecoveryError("控制器状态为空，无法确认急停")
        value = int(status[0])
        if value != 0:
            raise LimitRecoveryError(f"急停未释放或状态未知: status={value}")

    # ---- 主流程 ---------------------------------------------------------

    @staticmethod
    def validate_parameters(margin, jog_speed, poll, max_seconds):
        if not all(math.isfinite(v) and v > 0 for v in (margin, jog_speed, poll, max_seconds)):
            raise ValueError('脱限参数必须为正有限数')
        if jog_speed > 100:
            raise ValueError('Jog百分比不得超过100')
    def run(
        self,
        cancel_event: Optional[Any] = None,
        *,
        margin: float = DEFAULT_MARGIN_DEG,
        jog_speed: float = DEFAULT_JOG_SPEED_PERCENT,
        poll: float = DEFAULT_POLL_SEC,
        max_seconds: float = DEFAULT_MAX_SECONDS_PER_AXIS,
    ) -> dict:
        """把越界轴逐轴 Jog 回限位内，返回恢复前后的状态。

        任一轴失败即抛出 ``LimitRecoveryError``；调用方负责下电与断连。
        """
        self.validate_parameters(margin, jog_speed, poll, max_seconds)
        if cancel_event is not None and cancel_event.is_set():
            raise LimitRecoveryError('脱限请求已取消，拒绝发送Jog')

        self._validate_emergency_stop()
        before, limits, axis_count = self.read_state()
        targets = self.overtravel_axes(before, limits)
        self.log(f"按需脱限：越界轴 {[(a, round(p, 3), round(lo, 1), round(hi, 1)) for a, p, lo, hi in targets]}")

        recovered: List[int] = []
        for axis, position, lower, upper in targets:
            if margin * 2 >= (upper - lower):
                raise LimitRecoveryError(f"J{axis} 限位区间过窄，margin={margin} 不合法")
            moving_positive = position < lower
            goal = lower + margin if moving_positive else upper - margin
            direction = 1.0 if moving_positive else -1.0
            self._jog_axis(axis, direction, goal, jog_speed, poll, max_seconds, cancel_event)
            recovered.append(axis)

        self._validate_emergency_stop()
        after, limits, _ = self.read_state()
        remaining = self.overtravel_axes(after, limits)
        if remaining:
            raise LimitRecoveryError(
                "脱限后仍存在越界轴: "
                + ", ".join(f"J{a}={p:.3f}∉[{lo:.1f},{hi:.1f}]" for a, p, lo, hi in remaining)
            )
        return {
            "recovered_axes": recovered,
            "positions_before": before,
            "positions_after": after,
            "limits": [[lo, hi] for lo, hi in limits],
            "axis_count": axis_count,
            "jog_speed_percent": jog_speed,
            "margin_deg": margin,
        }

    def _jog_axis(
        self,
        axis: int,
        direction: float,
        goal: float,
        jog_speed: float,
        poll: float,
        max_seconds: float,
        cancel_event: Optional[Any] = None,
    ) -> None:
        """单轴连续 Jog，直到跨过目标位置；任何退出路径都停 Jog。"""
        name = f"J{axis}"
        self.log(f"{name} 向 {'正' if direction > 0 else '负'}方向 Jog 至 {goal:.3f}°")
        start = float(self.robot.get_joint_pos()[axis - 1])
        if not math.isfinite(start):
            raise LimitRecoveryError('Jog起点反馈非有限数')
        budget = abs(goal - start) + 1.0
        deadline = self._monotonic() + max_seconds
        last = start
        stagnant = 0
        try:
            if cancel_event is not None and cancel_event.is_set():
                raise LimitRecoveryError('Jog发送前收到停止')
            self._validate_emergency_stop()
            result = self.robot.move_along(name, direction * jog_speed)
            if result != 0:
                raise LimitRecoveryError(f'{name} Jog返回失败: {result}')
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    raise LimitRecoveryError(f"{name} 脱限被停止请求中断")
                self._validate_emergency_stop()
                position = float(self.robot.get_joint_pos()[axis - 1])
                if not math.isfinite(position):
                    raise LimitRecoveryError(f'{name} Jog位置反馈NaN/Inf')
                if abs(position - start) > budget:
                    raise LimitRecoveryError(f'{name} 超过恢复行程预算')
                if direction * (position - last) < -0.1:
                    raise LimitRecoveryError(f'{name} 向错误方向移动')
                if (direction > 0 and position >= goal) or (direction < 0 and position <= goal):
                    break
                if self._monotonic() >= deadline:
                    raise LimitRecoveryError(f"{name} 在 {max_seconds}s 内未到达 {goal:.3f}°")
                if last is not None and abs(position - last) < STAGNANT_EPSILON_DEG:
                    stagnant += 1
                    if stagnant >= STAGNANT_POLLS_LIMIT:
                        raise LimitRecoveryError(f"{name} 连续无位移，疑似被阻挡")
                else:
                    stagnant = 0
                last = position
                self._sleep(poll)
        finally:
            try:
                result = self.robot.move_stop_jog()
                if result != 0:
                    raise LimitRecoveryError(f'{name} 停止Jog返回失败: {result}')
            except Exception as exc:  # 停不下来必须让调用方知道
                raise LimitRecoveryError(f"{name} 停止 Jog 失败: {exc}") from exc
