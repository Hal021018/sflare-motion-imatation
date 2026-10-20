#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯单文件离线自测试套件（内置 Fake PallasSDK）。

不连接真实控制器，不发送真实硬件运动，纯内存运行，执行完毕可随时删除。
测试覆盖：
1. ActionRepository 目录扫描与名称校验
2. ActionParser 严格语义防错
3. TAIJI 五次 Hermite 插值数学精度（节点误差几乎为0）
4. TAIJI prepare_path 步长强制限速细分（防止单周期超速）
5. EmiCoordinator 完整流程回放 (连接->attach->上电->abort->EMI->并发servo_j->FeedbackTracker反馈->at_target零位收敛->停发平息->安全退出下电)
6. 急停/取消中断发包与断电自愈
"""

from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock
from pathlib import Path
from typing import Any, List, Sequence

from action_pipeline import (
    ActionRepository,
    MAX_JOINT_SPEED_DEG_S,
    enforce_runtime_safety,
    prepare_path,
    _quintic_hermite_fallback,
    plan_action_taiji_style,
)
from emi_coordinator import (EmiCoordinator, FeedbackTracker, format_send_intervals,
                             require_attached,
                             write_playback_trace, write_feedback_trace)


FAKE_CONTROLLER_LIMITS = (
    (-65.0, 195.0),
    (-2.0, 115.0),
    (-50.0, 50.0),
    (-3.0, 95.0),
    (-85.0, 85.0),
    (-43.0, 41.0),
    (-45.0, 27.0),
)


class FakeCtypesList:
    def __init__(self, data: Sequence[float]) -> None:
        self._data = list(data)

    def toList(self) -> List[float]:
        return list(self._data)


class FakeFeedbackData:
    def __init__(self, joints: Sequence[float]) -> None:
        self.robot_joint_pos = list(joints)
        self.robot_joint_speed = [0.1] * 7
        self.robot_speed = 0.25
        self.robot_mode = 1
        self.robot_state = 1
        self.error_code = 0


class FakeRobot:
    def __init__(self, arm_id: int) -> None:
        self.arm_id = arm_id
        self.servo_j_packets: List[List[float]] = []
        self.emi_active = False
        self.power_on = False
        self.feedback_callback = None
        self.current_pos = [0.0] * 7

    def attach(self) -> int:
        return 0

    def get_attached_id(self) -> int:
        """与真实 SDK 一致：绑定成功时返回机器人 ID。"""
        return self.arm_id

    def set_speed(self, speed: int) -> int:
        return 0

    def set_profile(self, profile: Any) -> int:
        return 0

    def abort(self) -> int:
        return 0

    def get_joint_pos(self) -> FakeCtypesList:
        return FakeCtypesList(self.current_pos)

    def get_state(self, timeout: int = 1000) -> FakeCtypesList:
        return FakeCtypesList([1.0 if self.power_on else 0.0])

    def get_limit(self, axis: int, timeout: int = 1000) -> List[float]:
        return list(
            FakeController.limit_overrides.get(axis, FAKE_CONTROLLER_LIMITS[axis - 1])
        )

    def set_emi_feedback_callback(self, callback: Any) -> None:
        self.feedback_callback = callback

    def emi_start(self, word: int, scene: int, period: float, a: int, b: int) -> int:
        self.emi_active = True
        # 启动时立刻触发首次反馈以满足 FEEDBACK_STALE_SEC 校验
        if callable(self.feedback_callback):
            self.feedback_callback(FakeFeedbackData(self.current_pos))
        return 0

    def emi_end(self, timeout: int = 1000) -> int:
        self.emi_active = False
        return 0

    def servo_j(
        self,
        cmd: Sequence[float],
        acc: float,
        vel: float,
        t: float,
        lookahead_time: float,
        gain: float,
    ) -> int:
        if not self.emi_active:
            raise RuntimeError("servo_j 在未启动 EMI 时被调用")
        self.current_pos = list(cmd[:7])
        self.servo_j_packets.append(list(cmd))
        if callable(self.feedback_callback):
            self.feedback_callback(FakeFeedbackData(self.current_pos))
        return 0


class FakeController:
    instances: List[Any] = []
    initial_pos = [0.0] * 7
    connect_results: List[int] = []
    com_send_result = 0
    limit_overrides = {}

    def __init__(self) -> None:
        self.connected = False
        self.check_error_calls = 0
        self.robots = {}
        self.emi_state = "1"
        self.__class__.instances.append(self)

    def connect(self, ip: str, port: int) -> int:
        if self.__class__.connect_results:
            result = self.__class__.connect_results.pop(0)
            if result != 0:
                return result
        self.connected = True
        return 0

    def disconnect(self) -> int:
        self.connected = False
        return 0

    def check_error(self, timeout: int = 1000) -> List[str]:
        self.check_error_calls += 1
        return []

    def set_data_unit(self, unit: int) -> int:
        return 0

    def idn_write(self, idn: str, axis: str, val: str) -> int:
        return 0

    def idn_read(self, idn: str, axis: str) -> str:
        # 1203.1 返回当前 EMI 状态 (1 为开启，0 为关闭)
        return self.emi_state if self.connected else "0"

    def set_power_enable(self, enable: bool, robot_id: int = 1, timeout: int = 1000) -> int:
        if robot_id in self.robots:
            self.robots[robot_id].power_on = enable
            if not enable:
                self.emi_state = "0"
        return 0

    def add_robot(self, robot_id: int) -> FakeRobot:
        robot = FakeRobot(robot_id)
        robot.current_pos = list(self.__class__.initial_pos)
        self.robots[robot_id] = robot
        return robot

    def com_open(self, port: str, *args, **kwargs) -> int:
        return 0

    def com_close(self, port: str) -> int:
        return 0

    def com_send(self, port: str, hex_data: str) -> int:
        return self.__class__.com_send_result


class FakeSDKModule:
    Controller = FakeController
    Profile = lambda *args: None


class TestOfflinePipeline(unittest.TestCase):
    def setUp(self) -> None:
        trace_dir = tempfile.TemporaryDirectory()
        self.addCleanup(trace_dir.cleanup)
        trace_patch = mock.patch("emi_coordinator.TRACE_OUTPUT_DIR", Path(trace_dir.name))
        trace_patch.start()
        self.addCleanup(trace_patch.stop)
        FakeController.instances = []
        FakeController.initial_pos = [0.0] * 7
        FakeController.connect_results = []
        FakeController.com_send_result = 0
        FakeController.limit_overrides = {}
        self.actions_dir = Path(__file__).parent / "actions"
        self.repo = ActionRepository(self.actions_dir)

    def test_17_target_error_diagnostics(self) -> None:
        tracker = FeedbackTracker(("右臂",))
        tracker.make_callback("右臂")(FakeFeedbackData([1.5, 0, 0, 0, 0, 0, 0]))
        message = tracker.describe_target_error({"右臂": [0.0] * 7})
        self.assertIn("最大误差J1=+1.500°", message)
        self.assertIn("反馈龄期=", message)

    def test_01_repository_scan(self) -> None:
        """动作仓库覆盖16个单臂动作及原3个双臂动作。"""
        available = self.repo.available_actions
        self.assertEqual(available, sorted([
            "fist", "stretch", "stretch_ready", "wave", "handshake", "like",
            "forward_wave", "dance", "sway", "heart", "forward", "turn_grip",
            "doubt", "raise_head", "shakehand", "slight_raise", "diagonal_point",
            "chin_rest", "handover",
        ]))

    def test_02_parse_strictness(self) -> None:
        """测试 2: 动作文件解析严格度与内容校验"""
        doc = self.repo.get("fist")
        self.assertEqual(doc.name, "fist")
        self.assertEqual(doc.units, "deg")
        self.assertEqual(len(doc.keyframes), 2)
        self.assertTrue(len(doc.content_hash) > 0)

        self.assertEqual(
            doc.keyframes[0].left,
            (90.0, 16.0, -35.0, 93.0, 10.0, 0.0, 0.0),
        )
        self.assertEqual(
            doc.keyframes[1].left,
            (87.0, 13.0, -43.0, 94.5, 18.0, 8.0, 0.0),
        )
        # The planner must preserve the production action's J6/J7 coordinates.
        self.assertEqual(enforce_runtime_safety(doc.keyframes[1].left)[5], 8.0)

    def test_03_taiji_hermite_interpolation_nodes(self) -> None:
        """测试 3: TAIJI 原版五次 Hermite 插值必须严格命中关键帧"""
        times = [0.0, 2.0, 4.0]
        points = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 20.0, -15.0, 30.0, 5.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        t_eval = [0.0, 2.0, 4.0]

        res = _quintic_hermite_fallback(times, points, t_eval)
        for actual, expected in zip(res[0], points[0]):
            self.assertAlmostEqual(actual, expected, places=5)
        for actual, expected in zip(res[1], points[1]):
            self.assertAlmostEqual(actual, expected, places=5)
        for actual, expected in zip(res[2], points[2]):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_04_prepare_path_speed_limiter(self) -> None:
        """测试 4: prepare_path 必须强制将大步长细分为合规单周期变化量 (<= 2.4°)"""
        # 给定一个大幅度跳跃 (0° -> 24°)
        big_jump_l = [[0.0] * 7, [24.0] * 7]
        big_jump_r = [[0.0] * 7, [24.0] * 7]
        period = 0.040
        speed_limit = 60.0  # 60 deg/s -> 单周期步长 2.4°

        safe_l, safe_r = prepare_path(big_jump_l, big_jump_r, period, speed_limit)
        # 验证单周期最大步长绝对不会超过 2.4°
        for i in range(len(safe_l) - 1):
            delta_l = max(abs(b - a) for a, b in zip(safe_l[i], safe_l[i + 1]))
            delta_r = max(abs(b - a) for a, b in zip(safe_r[i], safe_r[i + 1]))
            self.assertLessEqual(delta_l, period * speed_limit + 1e-4)
            self.assertLessEqual(delta_r, period * speed_limit + 1e-4)

    def test_05_taiji_plan_pipeline(self) -> None:
        """测试 5: plan_action_taiji_style 必须生成合规的三段流水线"""
        doc = self.repo.get("stretch")
        initial = [0.0] * 7
        plan = plan_action_taiji_style(doc, initial, initial)

        self.assertTrue(len(plan.trans_l) > 0)
        self.assertTrue(len(plan.action_l) > 0)
        self.assertTrue(len(plan.return_l) > 0)
        # 最后一帧严格为全零
        self.assertEqual(plan.return_l[-1], [0.0] * 7)
        self.assertEqual(plan.return_r[-1], [0.0] * 7)

    def test_06_emi_coordinator_taiji_execution(self) -> None:
        """测试 6: 使用 Fake SDK 跑通完整的 TAIJI 闭环 (含 FeedbackTracker 与 at_target 收敛)"""
        doc = self.repo.get("fist")
        plan = plan_action_taiji_style(doc, [0.0] * 7, [0.0] * 7)

        coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
        with mock.patch("emi_coordinator.high_precision_delay", lambda _: None), mock.patch(
            "emi_coordinator.SERVO_STOP_WAIT_SEC", 0.0
        ):
            success = coordinator.play(plan)

        self.assertTrue(success)
        self.assertTrue(coordinator.safe_to_reuse)
        self.assertTrue(all(ctrl.check_error_calls == 1 for ctrl in FakeController.instances))

    def test_07_emi_cancel_safety(self) -> None:
        """测试 7: 发包过程中收到取消事件，必须立即退出并调用 safe_cleanup_arm 安全断连"""
        doc = self.repo.get("stretch")
        plan = plan_action_taiji_style(doc, [0.0] * 7, [0.0] * 7)

        coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
        cancel_event = threading.Event()

        def trigger_stop():
            time.sleep(0.08)
            cancel_event.set()

        th = threading.Thread(target=trigger_stop)
        th.start()

        result = coordinator.play(plan, cancel_event=cancel_event)
        th.join()
        self.assertFalse(result)
        self.assertTrue(coordinator.safe_to_reuse)
        self.assertTrue(all(not ctrl.connected for ctrl in FakeController.instances))

    def test_08_live_initial_pose_is_used_for_transition(self) -> None:
        """规划必须在连接后使用编码器位置，不能用未连接时伪造的全零。"""
        FakeController.initial_pos = [5.0, 4.0, -3.0, 40.0, 2.0, 1.0, -1.0]
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
        document = self.repo.get("stretch_ready")
        with mock.patch("emi_coordinator.high_precision_delay", lambda _: None), mock.patch(
            "emi_coordinator.SERVO_STOP_WAIT_SEC", 0.0
        ):
            success = coordinator.play_document(document)
        self.assertTrue(success)
        self.assertEqual(
            coordinator.left_robot.servo_j_packets[0][:7],
            FakeController.initial_pos,
        )
        self.assertEqual(
            coordinator.right_robot.servo_j_packets[0][:7],
            FakeController.initial_pos,
        )

    def test_09_invalid_equal_keyframe_times_are_rejected(self) -> None:
        payload = {
            "schema_version": "1.0.0",
            "name": "bad",
            "scope": "dual",
            "units": "deg",
            "control": {"period": 0.04, "speed_scale": 1.0},
            "exit": {"auto_return_zero": True},
            "keyframes": [
                {"time": 0.0, "left": [0.0] * 7, "right": [0.0] * 7},
                {"time": 0.0, "left": [1.0] * 7, "right": [1.0] * 7},
            ],
            "events": [],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            repository = ActionRepository(directory)
            with self.assertRaisesRegex(RuntimeError, "严格递增"):
                repository.get("bad")

    def test_10_controller_limit_rejects_before_motion(self) -> None:
        """控制器动态限位不允许轨迹时，不能初始化手、上电或发包。"""
        FakeController.limit_overrides = {1: (-10.0, 10.0)}
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
        success = coordinator.play_document(self.repo.get("fist"))

        self.assertFalse(success)
        self.assertIn("轨迹点越界", coordinator.last_error)
        self.assertIsNone(coordinator.right_hand)
        self.assertTrue(all(not robot.servo_j_packets for ctrl in FakeController.instances for robot in ctrl.robots.values()))
        self.assertTrue(all(not ctrl.connected for ctrl in FakeController.instances))

    def test_11_hand_command_failure_stops_before_power(self) -> None:
        """灵巧手初始化失败不能被吞掉，也不能继续上电或发送轨迹。"""
        FakeController.com_send_result = -1
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
        success = coordinator.play_document(self.repo.get("fist"))

        self.assertFalse(success)
        self.assertIn("发送手势报文", coordinator.last_error)
        self.assertTrue(all(not robot.power_on for ctrl in FakeController.instances for robot in ctrl.robots.values()))
        self.assertTrue(all(not robot.servo_j_packets for ctrl in FakeController.instances for robot in ctrl.robots.values()))
        self.assertTrue(all(not ctrl.connected for ctrl in FakeController.instances))

    def test_12_partial_connection_is_released(self) -> None:
        """左控制器连接失败时，已经连接的右控制器必须被释放。"""
        FakeController.connect_results = [0, -1]
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
        success = coordinator.play_document(self.repo.get("fist"))

        self.assertFalse(success)
        self.assertIn("左臂 connect", coordinator.last_error)
        self.assertTrue(coordinator.safe_to_reuse)
        self.assertTrue(all(not ctrl.connected for ctrl in FakeController.instances))

    def test_13_right_action_points_match_original(self) -> None:
        wave = self.repo.get("wave")
        handshake = self.repo.get("handshake")
        self.assertEqual(wave.scope, "right")
        self.assertEqual(handshake.scope, "right")
        self.assertEqual([frame.right for frame in wave.keyframes], [
            (104, 20, -7, 73, -77, 0, 0),
            (104, 20, 21, 73, -77, 0, 0),
            (104, 20, -43, 73, -77, 0, 0),
            (104, 20, 21, 73, -77, 0, 0),
            (104, 20, -43, 73, -77, 0, 0),
            (0, 0, 0, 0, 0, 0, 0),
        ])
        self.assertEqual([frame.right[3] for frame in handshake.keyframes[:-1]],
                         [21, 15, 21, 15, 21, 15, 21])
        self.assertEqual(handshake.keyframes[-1].right, (0.0,) * 7)
        # 游戏版副本把展示停留从 3s 延长到 5s（见 actions.yaml 的 hold_sec）
        self.assertIn({"op": "wait", "seconds": 5.0, "phase": "still"}, handshake.program)

    def test_13b_game_actions_hold_five_seconds(self) -> None:
        """游戏候选动作的展示停留统一为 5 秒，且位于回零 move 之前。"""
        for name in ("diagonal_point", "handshake", "handover", "like",
                     "raise_head", "doubt"):
            program = self.repo.get(name).program
            self.assertIn({"op": "wait", "seconds": 5.0, "phase": "still"}, program, name)
            last_move = max(i for i, s in enumerate(program) if s["op"] == "move")
            self.assertTrue(
                any(s["op"] == "wait" and s["seconds"] == 5.0
                    for s in program[:last_move]),
                f"{name} 的 5s 停留必须在回零 move 之前")

    def test_14_right_emi_never_connects_left(self) -> None:
        for name in ("wave", "handshake"):
            FakeController.instances = []
            coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
            with mock.patch("emi_coordinator.high_precision_delay", lambda _: None), mock.patch(
                "emi_coordinator.SERVO_STOP_WAIT_SEC", 0.0
            ):
                success = coordinator.play_document(self.repo.get(name))
            self.assertTrue(success, coordinator.last_error)
            self.assertEqual(len(FakeController.instances), 1)
            self.assertIsNone(coordinator.left_controller)
            self.assertIsNone(coordinator.left_robot)
            self.assertTrue(coordinator.right_robot.servo_j_packets)
            self.assertEqual(coordinator.right_robot.servo_j_packets[-1][:7], [0.0] * 7)

    def test_15_cancel_before_power_keeps_hand_and_arm_still(self) -> None:
        cancel = threading.Event()
        cancel.set()
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule)
        success = coordinator.play_document(self.repo.get("wave"), cancel)

        self.assertFalse(success)
        self.assertIsNone(coordinator.right_hand)
        self.assertTrue(coordinator.safe_to_reuse)
        self.assertEqual(len(FakeController.instances), 1)
        self.assertTrue(all(not robot.power_on and not robot.servo_j_packets
                            for robot in FakeController.instances[0].robots.values()))
        self.assertFalse(FakeController.instances[0].connected)

    def test_16_smooth_turns_preserve_points_limits_and_holds(self) -> None:
        """右臂往返点平滑换向，且不越过原点位或丢失握手末端停留。"""
        for name in ("wave",):
            doc = self.repo.get(name)
            self.assertTrue(all(frame.type == "pass" for frame in doc.keyframes[1:-1]))
            times = [frame.time for frame in doc.keyframes]
            points = [frame.right for frame in doc.keyframes]
            for index in range(len(times) - 1):
                samples = [times[index] + (times[index + 1] - times[index]) * fraction / 10
                           for fraction in range(11)]
                interpolated = _quintic_hermite_fallback(
                    times, points, samples,
                    [frame.type == "stop" for frame in doc.keyframes],
                )
                for axis in range(7):
                    low = min(points[index][axis], points[index + 1][axis])
                    high = max(points[index][axis], points[index + 1][axis])
                    self.assertTrue(all(low - 1e-6 <= point[axis] <= high + 1e-6
                                        for point in interpolated))
            plan = plan_action_taiji_style(doc, [0.0] * 7, [0.0] * 7)
            for axis in range(7):
                low = min(frame.right[axis] for frame in doc.keyframes)
                high = max(frame.right[axis] for frame in doc.keyframes)
                self.assertGreaterEqual(min(point[axis] for point in plan.action_r), low - 1e-6)
                self.assertLessEqual(max(point[axis] for point in plan.action_r), high + 1e-6)
            for previous, current in zip(plan.action_r, plan.action_r[1:]):
                self.assertLessEqual(
                    max(abs(a - b) for a, b in zip(previous, current)),
                    doc.period * MAX_JOINT_SPEED_DEG_S + 1e-6,
                )
            if name == "wave":
                # Uniform timing: the speed guard must not splice extra linear
                # frames into the precomputed Hermite trajectory.
                self.assertEqual(doc.period, 0.020)
                expected_frames = round(doc.keyframes[-1].time / doc.period) + 1
                self.assertEqual(len(plan.action_r), expected_frames)
                self.assertEqual(len(plan.trans_r), 251)
                self.assertAlmostEqual((len(plan.action_r) - 1) * doc.period, 11.4)

        handshake = self.repo.get("handshake")
        from action_program import ProgramRunner
        runner = ProgramRunner(handshake, [0.0] * 7, None, threading.Event())
        runner.current = list(handshake.keyframes[-2].right)
        held = []
        runner._hold(3.0, lambda left, right, phase: held.append(list(right)))
        self.assertEqual(len(held), 150)
        self.assertTrue(all(point == list(handshake.keyframes[-2].right) for point in held))
        for path in runner.paths.values():
            for previous, current in zip(path, path[1:]):
                self.assertLessEqual(max(abs(a-b) for a, b in zip(previous, current)),
                                     handshake.period * MAX_JOINT_SPEED_DEG_S + 1e-6)

    def test_18_wave_uses_bounded_hermite(self) -> None:
        doc = self.repo.get("wave")
        plan = plan_action_taiji_style(doc, [0.0] * 7, [0.0] * 7)
        times = [frame.time for frame in doc.keyframes]
        points = [frame.right for frame in doc.keyframes]
        samples = [index * doc.period for index in range(len(plan.action_r))]
        bounded = _quintic_hermite_fallback(
            times, points, samples,
            [frame.type == "stop" for frame in doc.keyframes],
        )
        for actual, hermite in zip(plan.action_r, bounded):
            for axis in range(7):
                self.assertAlmostEqual(actual[axis], hermite[axis], places=8)
        self.assertEqual(len(plan.action_r), 571)

    def test_19_send_interval_summary(self) -> None:
        lines = format_send_intervals({"起势": [0.02, 0.021, 0.03], "收尾": []}, 0.02)
        self.assertIn("间隔样本=3", lines[0])
        self.assertIn("超过周期25%=1次", lines[0])
        self.assertIn("样本不足", lines[1])

    def test_20_buffered_trace_writes_targets_and_feedback(self) -> None:
        tracker = FeedbackTracker(("右臂",))
        tracker.make_callback("右臂")(FakeFeedbackData([1.0] * 7))
        sample = tracker.latest_joint_samples(("右臂",))["右臂"]
        origin = sample["received_at"] - 0.01
        records = [("主体", 0, "右臂", origin, origin + 0.002,
                    tuple([2.0] * 7), sample)]
        with tempfile.TemporaryDirectory() as directory:
            path = write_playback_trace("wave", origin, records, Path(directory))
            lines = path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 2)
            self.assertIn("target_J1", lines[0])
            self.assertIn("actual_J7", lines[0])
            self.assertIn("主体,0,右臂", lines[1])
            self.assertIn(",2.0,", lines[1])

    def test_21_callback_history_exports_raw_speed_and_status(self) -> None:
        tracker = FeedbackTracker(("右臂",))
        callback = tracker.make_callback("右臂")
        callback(FakeFeedbackData([1.0] * 7))
        callback(FakeFeedbackData([2.0] * 7))
        samples = tracker.feedback_history()
        self.assertEqual([sample["count"] for sample in samples], [1, 2])
        self.assertEqual(samples[0]["joint_speed"], (0.1,) * 7)
        self.assertEqual(samples[0]["robot_speed"], 0.25)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "wave_feedback.csv"
            write_feedback_trace(path, samples[0]["received_at"], samples)
            lines = path.read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 3)
            self.assertIn("speed_raw_J7", lines[0])
            self.assertIn("robot_speed_raw", lines[0])
            self.assertIn(",右臂,2,1,1,0,0.25,", lines[2])


class TestAttachGuard(unittest.TestCase):
    """attach() 返回 0 但未真正绑定时必须立即失败，而不是拖到到位超时。"""

    class _Robot:
        def __init__(self, attached):
            self.attached = attached

        def attach(self):
            return 0

        def get_attached_id(self):
            return self.attached

    class _Controller:
        def check_error(self):
            return ["-5314 Error - Invalid robot ID"]

    def test_attached_ok(self):
        require_attached(self._Controller(), self._Robot(1), "右臂", 1)

    def test_legacy_sdk_without_getter(self):
        class Robot:
            def attach(self):
                return 0

        require_attached(self._Controller(), Robot(), "右臂", 1)


def main():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestOfflinePipeline)
    suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestAttachGuard))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
