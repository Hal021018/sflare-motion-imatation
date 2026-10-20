"""按需脱限（game_limit_recovery）的离线测试。

只使用假控制器/假关节，不连接任何硬件。
"""

import threading
import unittest

from game_limit_recovery import (
    GameLimitRecovery,
    LimitRecoveryError,
    DEFAULT_MARGIN_DEG,
)


class FakeClock:
    """可推进的单调时钟，配合注入的 sleep 使用。"""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeRobot:
    """关节位置随 Jog 方向推进的假机械臂。"""

    def __init__(self, positions, limits, step=1.0) -> None:
        self.positions = [float(v) for v in positions]
        self.limits = [tuple(map(float, pair)) for pair in limits]
        self.step = step
        self.jog = None
        self.jog_calls = []
        self.stop_calls = 0
        self.move_j_calls = 0

    def get_axis_number(self) -> int:
        return len(self.positions)

    def get_joint_pos(self):
        if self.jog is not None:
            axis, direction = self.jog
            self.positions[axis] += direction * self.step
        return list(self.positions)

    def get_limit(self, axis: int):
        return list(self.limits[axis - 1])

    def move_along(self, name: str, speed: float) -> int:
        axis = int(name[1:]) - 1
        self.jog = (axis, 1 if speed > 0 else -1)
        self.jog_calls.append((name, speed))
        return 0

    def move_stop_jog(self) -> int:
        self.jog = None
        self.stop_calls += 1
        return 0

    def move_j(self, *_args, **_kwargs):  # 脱限不允许调用点对点运动
        self.move_j_calls += 1
        return 0


class FakeController:
    def __init__(self, status=(0,), flip_after=None) -> None:
        self.status = list(status)
        self.flip_after = flip_after
        self.calls = 0

    def get_status(self):
        self.calls += 1
        if self.flip_after is not None and self.calls > self.flip_after:
            return [1]
        return list(self.status)


def make(positions, limits, status=(0,), step=1.0, flip_after=None, clock=None):
    robot = FakeRobot(positions, limits, step=step)
    controller = FakeController(status, flip_after=flip_after)
    clock = clock or FakeClock()
    recovery = GameLimitRecovery(
        controller, robot, monotonic=clock, sleep=lambda s: clock.advance(s))
    return recovery, robot, controller, clock


LIMITS = [(-70, 200), (-3, 120), (-55, 55), (-3, 120), (-85, 85), (-35, 35), (-20, 20)]


class TestGameLimitRecovery(unittest.TestCase):
    def test_below_lower_jogs_positive(self) -> None:
        """低于下限 → 向正方向 Jog，回到下限+margin 后停。"""
        positions = [-80.0] + [0.0] * 6
        recovery, robot, _, _ = make(positions, LIMITS)
        result = recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(result["recovered_axes"], [1])
        self.assertEqual(len(robot.jog_calls), 1)
        self.assertGreater(robot.jog_calls[0][1], 0)
        self.assertGreaterEqual(robot.positions[0], -70 + 3.0)
        self.assertEqual(robot.stop_calls, 1)
        self.assertEqual(robot.move_j_calls, 0, "脱限不得调用点对点运动")

    def test_above_upper_jogs_negative(self) -> None:
        """高于上限 → 向负方向 Jog，回到上限-margin 后停。"""
        positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 40.0]
        recovery, robot, _, _ = make(positions, LIMITS)
        result = recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(result["recovered_axes"], [7])
        self.assertLess(robot.jog_calls[0][1], 0)
        self.assertLessEqual(robot.positions[6], 20 - 3.0)
        self.assertEqual(robot.stop_calls, 1)

    def test_multiple_axes_sequential(self) -> None:
        positions = [-80.0, 130.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        recovery, robot, _, _ = make(positions, LIMITS)
        result = recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(result["recovered_axes"], [1, 2])
        self.assertEqual([call[0] for call in robot.jog_calls], ["J1", "J2"])
        self.assertEqual(robot.stop_calls, 2)

    def test_within_limits_no_motion(self) -> None:
        recovery, robot, _, _ = make([0.0] * 7, LIMITS)
        result = recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(result["recovered_axes"], [])
        self.assertEqual(robot.jog_calls, [])
        self.assertEqual(robot.stop_calls, 0)

    def test_emergency_stop_refuses(self) -> None:
        recovery, robot, _, _ = make([-80.0] + [0.0] * 6, LIMITS, status=(1,))
        with self.assertRaises(LimitRecoveryError):
            recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(robot.jog_calls, [])

    def test_empty_status_refuses(self) -> None:
        recovery, robot, _, _ = make([-80.0] + [0.0] * 6, LIMITS, status=())
        with self.assertRaises(LimitRecoveryError):
            recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(robot.jog_calls, [])

    def test_emergency_during_motion_stops_jog(self) -> None:
        """Jog 过程中急停按下 → 抛错，且必须停 Jog。"""
        recovery, robot, _, _ = make(
            [-80.0] + [0.0] * 6, LIMITS, step=0.1, flip_after=3)
        with self.assertRaises(LimitRecoveryError):
            recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(robot.stop_calls, 1)

    def test_timeout_stops_jog(self) -> None:
        """长时间未到目标 → 超时抛错，且必须停 Jog。"""
        recovery, robot, _, _ = make([-80.0] + [0.0] * 6, LIMITS, step=0.001)
        with self.assertRaises(LimitRecoveryError):
            recovery.run(margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=1.0)
        self.assertEqual(robot.stop_calls, 1)

    def test_pre_cancel_never_starts_jog(self) -> None:
        cancel = threading.Event()
        cancel.set()
        recovery, robot, _, _ = make([-80.0] + [0.0] * 6, LIMITS, step=0.1)
        with self.assertRaises(LimitRecoveryError):
            recovery.run(cancel, margin=3.0, jog_speed=50.0, poll=0.05, max_seconds=10.0)
        self.assertEqual(robot.stop_calls, 0)
        self.assertEqual(robot.jog_calls, [])

    def test_overtravel_axes_only_reports_true_violations(self) -> None:
        positions = [-70.0, 120.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 恰好等于限位，不算越界
        self.assertEqual(
            GameLimitRecovery.overtravel_axes(positions, LIMITS), [])

    def test_invalid_parameters_rejected(self) -> None:
        recovery, _, _, _ = make([0.0] * 7, LIMITS)
        for kwargs in ({"margin": 0}, {"jog_speed": 0}, {"poll": 0}, {"max_seconds": 0}):
            with self.assertRaises(ValueError):
                recovery.run(**{"margin": 3, "jog_speed": 50, "poll": 0.05,
                                "max_seconds": 10, **kwargs})

    def test_nonfinite_parameters_rejected(self):
        for value in (float('nan'), float('inf')):
            for field in ('margin', 'jog_speed', 'poll', 'max_seconds'):
                recovery, robot, _, _ = make([0.] * 7, LIMITS)
                with self.assertRaises(ValueError):
                    recovery.run(**{field: value})
                self.assertEqual(robot.jog_calls, [])

    def test_wrong_direction_stops(self):
        recovery, robot, _, _ = make([-80.] + [0.] * 6, LIMITS, step=-1.)
        with self.assertRaisesRegex(LimitRecoveryError, '错误方向'):
            recovery.run()
        self.assertEqual(robot.stop_calls, 1)

    def test_excess_displacement_stops(self):
        recovery, robot, _, _ = make([-80.] + [0.] * 6, LIMITS, step=30.)
        with self.assertRaisesRegex(LimitRecoveryError, '行程预算'):
            recovery.run()
        self.assertEqual(robot.stop_calls, 1)

    def test_margin_too_large_for_narrow_limit(self) -> None:
        narrow = [(-1.0, 1.0)] + LIMITS[1:]
        recovery, _, _, _ = make([-5.0] + [0.0] * 6, narrow)
        with self.assertRaises(LimitRecoveryError):
            recovery.run(margin=DEFAULT_MARGIN_DEG, jog_speed=50.0,
                         poll=0.05, max_seconds=10.0)


if __name__ == "__main__":
    unittest.main()
