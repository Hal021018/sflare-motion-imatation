"""Verified limit recovery extracted from the original service; no action poses."""
from __future__ import annotations
import math
import re
import time
from typing import Iterable, List, Tuple
from action_session import extract_error_codes

class MotionFailure(RuntimeError):
    pass

class UnsafeToDisconnect(MotionFailure):
    pass

class ActionCancelled(MotionFailure):
    pass

SPEED = 100
MOTION_COMMAND_TIMEOUT_MS = 5000
LIMIT_CHECK_AXIS_COUNT = 7
RECOVERY_ALARM_SETTLE_SEC = 1.0
RECOVERY_POWER_SETTLE_SEC = 1.0
LIMIT_RECOVERY_MARGIN_DEG = 3.0
LIMIT_RECOVERY_JOG_SPEED = 15.0
LIMIT_RECOVERY_INITIAL_PULSE_SEC = 0.15
LIMIT_RECOVERY_MIN_PULSE_SEC = 0.10
LIMIT_RECOVERY_MAX_PULSE_SEC = 0.50
LIMIT_RECOVERY_MAX_DURATION_SEC = 90.0
LIMIT_RECOVERY_MAX_PULSES = 600
LIMIT_RECOVERY_MAX_EXTRA_TRAVEL_DEG = 1.0
LIMIT_RECOVERY_PROGRESS_WINDOW_SEC = 2.0
LIMIT_RECOVERY_MIN_WINDOW_PROGRESS_DEG = 0.02
LIMIT_RECOVERY_MAX_STAGNANT_WINDOWS = 4
LIMIT_RECOVERY_WRONG_DIRECTION_DEG = 0.10
LIMIT_RECOVERY_DESIRED_STEP_MIN_DEG = 0.05
LIMIT_RECOVERY_DESIRED_STEP_MAX_DEG = 0.50
AUTO_LIMIT_RECOVERY_CODES = frozenset({-5031, -5032})
NON_BLOCKING_ENCODER_CLEAR_CODES = frozenset({-5047, -5059})

def _require_finite_axes(
    values: Iterable[float],
    label: str,
    minimum_axes: int = LIMIT_CHECK_AXIS_COUNT,
) -> List[float]:
    """Return a finite feedback/target vector or fail closed.

    PallasSDK normally reports seven arm joints, but an empty, truncated, or
    non-finite response must never be reduced to zero by a safety predicate.
    Keeping this check in one helper makes every position/speed/limit caller
    use the same minimum-axis and finite-value contract.
    """
    try:
        converted = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise MotionFailure(f"{label}：反馈包含不可解析的关节值") from exc
    if len(converted) < minimum_axes:
        raise MotionFailure(
            f"{label}：反馈轴数不足（收到 {len(converted)}，至少需要 {minimum_axes}）"
        )
    if not all(math.isfinite(value) for value in converted):
        raise MotionFailure(f"{label}：反馈包含 NaN/Inf 等非有限关节值")
    return converted

class LimitRecovery:
    def __init__(self, arms, profile, cancel, permitted, logger):
        self.active_arms = tuple(arms)
        self.profile = profile
        self.cancel = cancel
        self.permitted = permitted
        self.info = self.warn = logger
        self._motion_active = False
        self._last_recovery_codes = frozenset()
        self._last_controller_error_text = {a.label: a.diagnostic for a in arms}
        self._last_controller_error_codes = {}
        self._last_motion_error_codes = {}

    def _check_interruption(self, phase):
        if self.cancel.is_set() or not self.permitted():
            raise ActionCancelled(phase + ': recovery cancelled or permission revoked')

    def _wait_interruptibly(self, seconds, phase):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._check_interruption(phase)
            self.cancel.wait(min(0.05, max(0.0, deadline - time.monotonic())))
        self._check_interruption(phase)

    def _controller_errors(self, arm):
        self._check_interruption('pre-recovery diagnostics')
        diagnostic = repr(arm.controller.check_error(timeout=1000))
        # Unlike a historical limit code, a new physical/power/network error
        # must veto alarm clearing and power-on, even during recovery.
        from action_faults import active_fault_codes
        codes = active_fault_codes(extract_error_codes(diagnostic), False)
        if codes & {-5201, -135, -5176, -161, -20236, -20035, -20038, -20039, -5306}:
            self._last_recovery_codes |= codes
            raise MotionFailure('blocking fault before recovery: ' + diagnostic)
        return diagnostic

    def _abort_and_confirm_stopped(self, reason):
        # Called only after sender.close() has joined every EMI send worker.
        for arm in self.active_arms:
            try:
                arm.robot.abort(timeout=MOTION_COMMAND_TIMEOUT_MS)
            except Exception as exc:
                self.warn(reason + ': abort: ' + str(exc))
        deadline = time.monotonic() + 5.0
        settled = 0
        while time.monotonic() < deadline:
            quiet = True
            for arm in self.active_arms:
                try:
                    speed = _require_finite_axes(arm.robot.get_joint_speed(timeout=1000), arm.label)
                    quiet = quiet and max(abs(v) for v in speed[:7]) <= 0.05
                except Exception:
                    quiet = False
            settled = settled + 1 if quiet else 0
            if settled >= 2:
                self._motion_active = False
                return True
            time.sleep(0.1)
        return False

    def _limit_candidates(
        self,
        arm: Arm,
        *,
        include_near_limit: bool,
    ) -> List[Tuple[int, float, float, float, float]]:
        """Return joints that can only be Jogged away from a configured limit."""
        if arm.robot is None:
            return []
        joints = _require_finite_axes(
            arm.robot.get_joint_pos(timeout=1000).toList(),
            f"{arm.label} 限位恢复位置反馈",
        )
        candidates = []
        checked_axis_count = LIMIT_CHECK_AXIS_COUNT
        for axis in range(1, checked_axis_count + 1):
            raw_limits = _require_finite_axes(
                arm.robot.get_limit(axis, timeout=1000),
                f"{arm.label} J{axis} 限位反馈",
                minimum_axes=2,
            )
            lower, upper = min(raw_limits), max(raw_limits)
            current = joints[axis - 1]
            if current < lower:
                candidates.append((axis, 1.0, current, lower, upper))
            elif current > upper:
                candidates.append((axis, -1.0, current, lower, upper))
            elif include_near_limit and current <= lower + LIMIT_RECOVERY_MARGIN_DEG:
                candidates.append((axis, 1.0, current, lower, upper))
            elif include_near_limit and current >= upper - LIMIT_RECOVERY_MARGIN_DEG:
                candidates.append((axis, -1.0, current, lower, upper))
        return candidates

    @staticmethod
    def _reported_limit_axes(diagnostic: str) -> Tuple[int, ...]:
        """Extract controller-reported axes such as ``Robot:1,Axis:4``.

        Axis information only narrows a candidate. Position and configured
        limit validation are still mandatory before any Jog is sent.
        """
        axes = set()
        for raw_axis in re.findall(
            r"Robot\s*:\s*\d+\s*,\s*Axis\s*:\s*(\d+)",
            diagnostic,
            flags=re.IGNORECASE,
        ):
            axis = int(raw_axis)
            if 1 <= axis <= LIMIT_CHECK_AXIS_COUNT:
                axes.add(axis)
        return tuple(sorted(axes))

    def _restore_motion_settings(self, arm: Arm) -> bool:
        """Stop Jog and restore normal joint-mode settings after recovery.

        A failed cleanup is never treated as a recovered limit: leaving a
        controller in Jog or the wrong coordinate frame would make the next
        command unsafe to interpret.
        """
        if arm.robot is None:
            self.warn(f"{arm.label} -5032 恢复后无法还原：robot 未连接")
            return False
        failures = []
        try:
            arm.robot.move_stop_jog(timeout=MOTION_COMMAND_TIMEOUT_MS)
        except Exception as exc:
            failures.append(exc)
            self.warn(f"{arm.label} -5032 恢复后停止 Jog 失败：{exc}")
        try:
            arm.robot.set_jog_mode(1, False)
            arm.robot.set_frame_type(2)
            arm.robot.set_speed(SPEED)
            arm.robot.set_profile(self.profile)
        except Exception as exc:
            failures.append(exc)
            self.warn(f"{arm.label} -5032 恢复后还原运动模式失败：{exc}")
        if failures:
            codes = set(self._last_recovery_codes)
            for failure in failures:
                codes.update(extract_error_codes(failure))
            self._last_recovery_codes = frozenset(codes)
            return False
        return True

    def _recover_axis_into_limit(
        self,
        arm: Arm,
        axis: int,
        direction: float,
        current: float,
        lower: float,
        upper: float,
    ) -> bool:
        """Closed-loop Jog one verified axis back into its safe limit range.

        ``direction`` is derived exclusively from the measured overrun/limit
        side.  The method never reverses it.  Each Jog pulse is stopped before
        position is read, so completion, interruption and no-progress handling
        all leave the axis with an explicit stop command.
        """
        if arm.robot is None:
            return False

        target = (
            lower + LIMIT_RECOVERY_MARGIN_DEG
            if direction > 0
            else upper - LIMIT_RECOVERY_MARGIN_DEG
        )

        def reached(value: float) -> bool:
            return value >= target if direction > 0 else value <= target

        if reached(current):
            return True

        start_position = current
        planned_travel = abs(target - start_position)
        max_allowed_travel = (
            planned_travel + LIMIT_RECOVERY_MAX_EXTRA_TRAVEL_DEG
        )
        started = time.monotonic()
        window_started = started
        window_position = current
        last_current = current
        pulse_sec = LIMIT_RECOVERY_INITIAL_PULSE_SEC
        stagnant_windows = 0
        pulse_count = 0

        while pulse_count < LIMIT_RECOVERY_MAX_PULSES:
            self._check_interruption(f"{arm.label} J{axis} 限位恢复")
            elapsed = time.monotonic() - started
            if elapsed >= LIMIT_RECOVERY_MAX_DURATION_SEC:
                self.warn(
                    f"{arm.label} J{axis} 自适应脱限超时："
                    f"{elapsed:.1f}s，当前位置 {current:.3f}°，目标 {target:.3f}°"
                )
                return False

            self._motion_active = True
            try:
                arm.robot.move_along(
                    f"J{axis}",
                    direction * LIMIT_RECOVERY_JOG_SPEED,
                    timeout=MOTION_COMMAND_TIMEOUT_MS,
                )
                self._wait_interruptibly(
                    pulse_sec,
                    f"{arm.label} J{axis} 限位恢复脉冲",
                )
            finally:
                try:
                    arm.robot.move_stop_jog(timeout=MOTION_COMMAND_TIMEOUT_MS)
                finally:
                    self._motion_active = False

            pulse_count += 1
            recovered_position = _require_finite_axes(
                arm.robot.get_joint_pos(timeout=1000).toList(),
                f"{arm.label} J{axis} 限位恢复位置反馈",
            )
            current = recovered_position[axis - 1]
            actual_travel = abs(current - start_position)
            step_progress = direction * (current - last_current)
            total_progress = direction * (current - window_position)
            now = time.monotonic()

            if actual_travel >= max_allowed_travel:
                self.warn(
                    f"{arm.label} J{axis} 超过自适应脱限角度硬限制："
                    f"累计 {actual_travel:.3f}° >= "
                    f"允许 {max_allowed_travel:.3f}° "
                    f"(计划 {planned_travel:.3f}° + "
                    f"余量 {LIMIT_RECOVERY_MAX_EXTRA_TRAVEL_DEG:.3f}°)；"
                    "立即停止并拒绝自动完成"
                )
                return False

            if step_progress < -LIMIT_RECOVERY_WRONG_DIRECTION_DEG:
                self.warn(
                    f"{arm.label} J{axis} 实测向错误方向移动 "
                    f"{step_progress:.3f}°；立即停止自适应脱限"
                )
                return False

            if reached(current):
                self.info(
                    f"{arm.label} J{axis} 已回到限位内安全区："
                    f"{current:.3f}°，目标 {target:.3f}°，"
                    f"脉冲 {pulse_count} 次，用时 {now - started:.1f}s"
                )
                return True

            # Adapt the next pulse using actual encoder progress.  Small but
            # positive motion is not a stall: it asks for a longer bounded
            # pulse, which fixes the old 0.1°-per-pulse false positive.
            if step_progress > 0.001:
                remaining = max(0.0, direction * (target - current))
                desired_step = min(
                    LIMIT_RECOVERY_DESIRED_STEP_MAX_DEG,
                    max(
                        LIMIT_RECOVERY_DESIRED_STEP_MIN_DEG,
                        remaining * 0.25,
                    ),
                )
                pulse_sec = min(
                    LIMIT_RECOVERY_MAX_PULSE_SEC,
                    max(
                        LIMIT_RECOVERY_MIN_PULSE_SEC,
                        pulse_sec * desired_step / step_progress,
                    ),
                )

            if now - window_started >= LIMIT_RECOVERY_PROGRESS_WINDOW_SEC:
                if total_progress < LIMIT_RECOVERY_MIN_WINDOW_PROGRESS_DEG:
                    stagnant_windows += 1
                    pulse_sec = min(
                        LIMIT_RECOVERY_MAX_PULSE_SEC,
                        max(
                            LIMIT_RECOVERY_MIN_PULSE_SEC,
                            pulse_sec * 1.5,
                        ),
                    )
                    self.warn(
                        f"{arm.label} J{axis} 脱限进展缓慢："
                        f"窗口位移 {total_progress:.3f}°，"
                        f"下一脉冲 {pulse_sec:.2f}s，"
                        f"无进展窗口 {stagnant_windows}/"
                        f"{LIMIT_RECOVERY_MAX_STAGNANT_WINDOWS}"
                    )
                else:
                    stagnant_windows = 0

                if stagnant_windows >= LIMIT_RECOVERY_MAX_STAGNANT_WINDOWS:
                    self.warn(
                        f"{arm.label} J{axis} 连续 "
                        f"{LIMIT_RECOVERY_MAX_STAGNANT_WINDOWS} 个时间窗口"
                        "无有效位移；停止脱限，避免持续顶住机构"
                    )
                    return False

                window_started = now
                window_position = current

            last_current = current

        self.warn(
            f"{arm.label} J{axis} 自适应脱限达到脉冲上限 "
            f"{LIMIT_RECOVERY_MAX_PULSES}；当前位置 {current:.3f}°，"
            f"目标 {target:.3f}°"
        )
        return False

    def recover_verified_limit(self) -> bool:
        """Jog only an arm proven to be at a directional software limit.

        This deliberately does not resume the interrupted action or issue gozero.
        If controller diagnostics do not identify -5031/-5032 and a measured
        near-limit joint, it fails closed and asks for manual inspection.
        """
        self._last_recovery_codes = frozenset()
        if self._motion_active and not self._abort_and_confirm_stopped(
            "-5032 限位恢复前停止",
        ):
            raise UnsafeToDisconnect("-5032 限位恢复前未确认双臂静止")

        affected: List[Tuple[Arm, List[Tuple[int, float, float, float, float]]]] = []
        for arm in self.active_arms:
            diagnostic = self._controller_errors(arm)
            previous_diagnostic = self._last_controller_error_text.get(arm.label, "")
            diagnostic_text = "; ".join(
                text for text in (previous_diagnostic, diagnostic) if text
            )
            self._last_controller_error_text[arm.label] = diagnostic_text
            diagnostic_codes = extract_error_codes(diagnostic_text)
            self._last_controller_error_codes[arm.label] = diagnostic_codes
            source_codes = (
                self._last_motion_error_codes.get(arm.label, frozenset())
                | self._last_controller_error_codes.get(arm.label, frozenset())
            )
            limit_codes = source_codes & AUTO_LIMIT_RECOVERY_CODES
            if not limit_codes:
                continue
            try:
                actual_overruns = self._limit_candidates(
                    arm,
                    include_near_limit=False,
                )
                near_limit_candidates = self._limit_candidates(
                    arm,
                    include_near_limit=True,
                )
            except Exception as exc:
                self._last_recovery_codes = extract_error_codes(exc)
                self.warn(f"{arm.label} 读取 -5031/-5032 限位状态失败：{exc}")
                return False
            # A measured out-of-range joint is unambiguous: it may be
            # recovered in the sole direction that leads back into range even
            # when the controller alarm omits Axis detail.  Near-limit joints
            # are deliberately ignored in this branch.  If several joints are
            # actually beyond different bounds, recover each one toward its
            # own safe direction instead of treating the two alarm codes as a
            # contradiction.
            if actual_overruns:
                self.warn(
                    f"{arm.label} -5031/-5032 恢复：实际越界轴 "
                    f"{[candidate[0] for candidate in actual_overruns]}，"
                    "将按安全方向逐轴脱限"
                )
                affected.append((arm, actual_overruns))
                continue

            # Without a real overrun, keep the original fail-closed rule:
            # use a reported Axis, or exactly one directional near-limit axis.
            # -5031 is a positive over-travel (upper bound), -5032 is the
            # mirror image at the lower bound.
            if limit_codes == frozenset({-5031}):
                direction = -1.0
            elif limit_codes == frozenset({-5032}):
                direction = 1.0
            else:
                self.warn(
                    f"{arm.label} received contradictory directional limit codes "
                    f"{sorted(limit_codes)} without a measured overrun; "
                    "refusing automatic Jog"
                )
                return False
            candidates = [
                candidate
                for candidate in near_limit_candidates
                if candidate[1] == direction
            ]
            reported_axes = self._reported_limit_axes(diagnostic_text)
            if len(reported_axes) > 1:
                self.warn(
                    f"{arm.label} limit alarm reports multiple axes "
                    f"{reported_axes}; refusing automatic Jog"
                )
                return False
            if reported_axes:
                candidates = [
                    candidate for candidate in candidates if candidate[0] in reported_axes
                ]
            elif len(candidates) != 1:
                self.warn(
                    f"{arm.label} limit alarm has no Axis detail and "
                    f"{len(candidates)} near-limit candidates; refusing automatic Jog"
                )
                return False
            if not candidates:
                self.warn(
                    f"{arm.label} 收到 -5031/-5032，但实测关节不在软件限位 "
                    f"{LIMIT_RECOVERY_MARGIN_DEG:.1f}° 范围内；不自动移动"
                )
                return False
            affected.append((arm, candidates))

        if not affected:
            self.warn("收到 -5032，但无法确认故障臂及实际限位关节；不自动移动")
            return False

        for arm, candidates in affected:
            try:
                self._check_interruption(f"{arm.label} -5032 限位恢复")
                # Re-check the same physical/power/network veto immediately
                # before clear_alarm; candidate collection can take SDK time.
                latched_errors = self._controller_errors(arm)
                latched_errors = [] if latched_errors in {'[]', '()', 'None'} else latched_errors
                if latched_errors:
                    try:
                        arm.controller.clear_alarm(
                            robot_index=1,
                            axis_index=0,
                            error_type=1,
                        )
                    except Exception as clear_exc:
                        clear_codes = extract_error_codes(clear_exc)
                        if clear_codes and clear_codes.issubset(
                            NON_BLOCKING_ENCODER_CLEAR_CODES
                        ):
                            self.warn(
                                f"{arm.label} -5031/-5032 恢复：编码器清警"
                                f"返回 {sorted(clear_codes)}；仅记录，"
                                "不阻断重新上电和安全方向 Jog"
                            )
                        else:
                            raise
                else:
                    self.info(
                        f"{arm.label} -5032 恢复：无已锁定报警，跳过 clear_alarm"
                    )
                self._wait_interruptibly(RECOVERY_ALARM_SETTLE_SEC, f"{arm.label} -5032 清警等待")
                arm.controller.set_power_enable(True, robot_id=1)
                self._wait_interruptibly(RECOVERY_POWER_SETTLE_SEC, f"{arm.label} -5032 上电等待")
                arm.robot.emi_enable(False)
                arm.robot.set_speed(LIMIT_RECOVERY_JOG_SPEED)
                arm.robot.set_frame_type(1)
                arm.robot.set_jog_mode(0, True)
                escaped_all = True
                for axis, direction, current, lower, upper in candidates:
                    self.warn(
                        f"{arm.label} -5031/-5032 验证：J{axis}={current:.2f}，"
                        f"限位 [{lower:.2f}, {upper:.2f}]；"
                        "启动闭环自适应安全方向 Jog"
                    )
                    escaped = self._recover_axis_into_limit(
                        arm,
                        axis,
                        direction,
                        current,
                        lower,
                        upper,
                    )
                    if not escaped:
                        escaped_all = False
                        self.warn(
                            f"{arm.label} J{axis} 未能安全回到限位内；"
                            "停止后续自动运动"
                        )
                        break
                self._motion_active = False
                if not self._restore_motion_settings(arm):
                    if not self._abort_and_confirm_stopped(f"{arm.label} -5032 恢复后模式还原失败"):
                        raise UnsafeToDisconnect(
                            f"{arm.label} -5032 恢复后模式还原失败且未确认静止"
                        )
                    return False
                if not escaped_all:
                    return False
            except ActionCancelled:
                self._motion_active = False
                self._restore_motion_settings(arm)
                raise
            except Exception as exc:
                self._last_recovery_codes = extract_error_codes(exc)
                self.warn(f"{arm.label} -5032 限位恢复失败：{exc}")
                self._motion_active = False
                self._restore_motion_settings(arm)
                if not self._abort_and_confirm_stopped(
                    f"{arm.label} -5032 限位恢复异常",
                ):
                    raise UnsafeToDisconnect(
                        f"{arm.label} -5032 限位恢复异常后未确认静止"
                    ) from exc
                return False

        self.info(
            "-5031/-5032 限位恢复完成；原动作已终止，"
            "不会自动续做，请重新发送下一条指令"
        )
        return True
