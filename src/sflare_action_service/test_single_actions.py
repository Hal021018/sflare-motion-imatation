"""Offline migration regressions. No ROS node, network, SDK, or hardware motion."""
import ast
import math
import threading
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from action_pipeline import ActionRepository, MAX_JOINT_SPEED_DEG_S
from action_program import ProgramRunner, validate_program, walk_steps
from emi_coordinator import DexterousHand, EmiCoordinator


ROOT = Path(__file__).parent
SOURCE = ROOT.parent / "action service" / "01_源码" / "当前版本" / "dual_arm_motion_actions.py"
SINGLE = {"like", "handshake", "forward_wave", "dance", "sway", "heart", "forward",
          "turn_grip", "doubt", "raise_head", "shakehand", "slight_raise", "diagonal_point",
          "chin_rest", "handover", "wave"}
BASE = {"forward_wave", "dance", "sway", "heart", "forward", "turn_grip"}


class FakeChassis:
    def __init__(self):
        self.calls = []
        self.yaw = 0.0
        self.fresh = True
        self.failed = None
        self.is_reached = True
        self.stop_ok = True

    def has_pose(self):
        return self.fresh

    def current_yaw(self):
        return self.yaw

    def set_mode(self, mode):
        self.calls.append(("mode", mode))

    def start_yaw_control(self, cancel, **kwargs):
        self.calls.append(("yaw_start", kwargs))

    def set_target_yaw(self, target, **kwargs):
        self.calls.append(("yaw_target", target, kwargs))
        self.yaw = target

    def reached(self):
        return self.is_reached

    def raise_if_yaw_failed(self, phase):
        if self.failed:
            raise RuntimeError(self.failed)

    def chassis_failure_reason(self):
        return self.failed

    def stop_yaw_control(self):
        self.calls.append(("yaw_stop",))
        return self.stop_ok

    def start_linear_motion(self, cancel, speed, distance):
        self.calls.append(("linear_start", speed, distance))
        return object()

    def stop_linear_motion(self, worker):
        self.calls.append(("linear_stop",))
        return self.stop_ok


class FakeTracker:
    def at_target(self, targets):
        return True

    def describe_target_error(self, targets):
        return "synthetic tracking error"

    def check_faults(self, labels):
        return []


class SingleActionTests(unittest.TestCase):
    def setUp(self):
        self.repo = ActionRepository(ROOT / "actions")

    def runner(self, name, base=None, cancel=None):
        return ProgramRunner(self.repo.get(name), [0]*7, base, cancel or threading.Event())

    def run_fast(self, runner, callback=None):
        clock = [0.0]
        frames, hands = [], []
        def tick(left, right, phase):
            clock[0] += runner.period
            frames.append((list(right), phase))
            if callback:
                callback(right, phase)
        with mock.patch("action_program.time.monotonic", lambda: clock[0]):
            runner.prepare_chassis()
            try:
                runner.run(tick, lambda event: hands.append((event.action, list(runner.current))), FakeTracker())
            finally:
                runner.stop_chassis()
        return frames, hands

    def test_complete_inventory_and_period(self):
        self.assertEqual(set(self.repo.available_actions), SINGLE | {"fist", "stretch", "stretch_ready"})
        for name in SINGLE:
            doc = self.repo.get(name)
            self.assertEqual(doc.period, 0.020)
            self.assertEqual(doc.scope, "right")
            self.assertEqual(doc.keyframes[-1].right, (0.0,)*7)
        # Keep the successful wave on its existing continuous-pass pipeline.
        wave = self.repo.get("wave")
        self.assertFalse(wave.program)
        self.assertEqual(wave.keyframes[-1].time, 11.4)

    def test_real_program_phase_events(self):
        runner = self.runner('like')
        phases = []
        clock = [0.]
        def tick(*args):
            clock[0] += runner.period
        with mock.patch('action_program.time.monotonic', lambda: clock[0]):
            runner.run(tick, lambda event: None, FakeTracker(), notify=phases.append)
        self.assertEqual(phases, ['moving', 'still', 'reset'])

    def test_preparation_wait_does_not_publish_still(self):
        runner = self.runner('diagonal_point')
        events = []
        clock = [0.]
        def tick(*args):
            clock[0] += runner.period
        def notify(phase):
            events.append((phase, runner.frame))
        with mock.patch('action_program.time.monotonic', lambda: clock[0]):
            runner.run(tick, lambda event: None, FakeTracker(), notify=notify)
        self.assertEqual([frame for phase, frame in events if phase == 'still'], [0])

    def test_phase_marker_only_on_presentation_wait(self):
        with self.assertRaises(ValueError):
            validate_program([{'op':'wait', 'seconds':5., 'phase':'moving'}], 1)

    def test_source_coordinate_parity(self):
        if not SOURCE.exists():
            self.skipTest("original reference source is not bundled on this machine")
        tree = ast.parse(SOURCE.read_text(encoding="utf-8-sig"))
        names = {"ZERO", "LIKE_POSE", "RAISE_HEAD_POSE", "SLIGHT_RAISE_POSE", "DIAGONAL_POINT_POSE",
                 "CHIN_REST_POSE", "HANDOVER_POSE", "DOUBT_POSE", "DOUBT_ALT_POSE", "WAVE", "HANDSHAKE",
                 "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P_FINAL", "STAGE_1", "STAGE_2", "STAGE_4"}
        env = {}
        for item in tree.body:
            if isinstance(item, ast.Assign) and isinstance(item.targets[0], ast.Name) and item.targets[0].id in names:
                env[item.targets[0].id] = eval(compile(ast.Expression(item.value), "<pose>", "eval"), {"__builtins__": {}}, env)
        expected = {"wave": env["WAVE"], "handshake": (*env["HANDSHAKE"], env["ZERO"]),
                    "shakehand": (*env["HANDSHAKE"], env["ZERO"]), "sway": (*env["STAGE_1"], env["ZERO"]),
                    "forward": (env["P5"], env["P6"], env["ZERO"]), "forward_wave": env["WAVE"],
                    "turn_grip": (env["P7"], *env["WAVE"][:-1], env["ZERO"]),
                    "dance": (*env["STAGE_1"], *env["STAGE_2"], env["P5"], env["P6"], *env["STAGE_4"]),
                    "doubt": (env["DOUBT_POSE"], *([env["DOUBT_ALT_POSE"], env["DOUBT_POSE"]]*3), env["ZERO"])}
        for name, symbol in (("like", "LIKE_POSE"), ("raise_head", "RAISE_HEAD_POSE"), ("slight_raise", "SLIGHT_RAISE_POSE"),
                             ("diagonal_point", "DIAGONAL_POINT_POSE"), ("chin_rest", "CHIN_REST_POSE"), ("handover", "HANDOVER_POSE")):
            expected[name] = (env[symbol], env["ZERO"])
        expected["heart"] = tuple((p[0]+10, *p[1:]) for p in env["STAGE_2"]) + (env["ZERO"],)
        for name, poses in expected.items():
            self.assertEqual([kf.right for kf in self.repo.get(name).keyframes], [tuple(p[:7]) for p in poses], name)

    def test_chassis_methods_unchanged(self):
        if not SOURCE.exists():
            self.skipTest("original reference source is not bundled")
        original = ast.parse(SOURCE.read_text(encoding="utf-8-sig"))
        migrated = ast.parse((ROOT / "chassis_control.py").read_text(encoding="utf-8"))
        old_class = next(n for n in original.body if isinstance(n, ast.ClassDef) and n.name == "DualArmActionNode")
        new_class = next(n for n in migrated.body if isinstance(n, ast.ClassDef) and n.name == "ChassisController")
        old_methods = {n.name: ast.dump(n, include_attributes=False) for n in old_class.body if isinstance(n, ast.FunctionDef)}
        selected = [n for n in new_class.body if isinstance(n, ast.FunctionDef) and n.name in old_methods and n.name != "__init__"]
        self.assertEqual(len(selected), 15)
        for method in selected:
            self.assertEqual(ast.dump(method, include_attributes=False), old_methods[method.name], method.name)

    def test_every_cached_edge_is_bounded_and_preserves_endpoints(self):
        for name in SINGLE - {"wave"}:
            runner = self.runner(name, FakeChassis() if name in BASE else None)
            for (previous, target, duration), path in runner.paths.items():
                start = runner.initial if previous == -1 else runner.poses[previous]
                self.assertEqual(path[0], start, name)
                for actual, expected in zip(path[-1], runner.poses[target]):
                    self.assertAlmostEqual(actual, expected, places=9, msg=name)
                for point in path:
                    self.assertTrue(all(math.isfinite(q) for q in point))
                    for axis in range(7):
                        self.assertGreaterEqual(point[axis]+1e-8, min(start[axis], runner.poses[target][axis]))
                        self.assertLessEqual(point[axis]-1e-8, max(start[axis], runner.poses[target][axis]))
                self.assertTrue(all(max(abs(a-b) for a, b in zip(x, y)) <= runner.period*MAX_JOINT_SPEED_DEG_S+1e-8
                                    for x, y in zip(path, path[1:])), name)
            last = [s for s in self.repo.get(name).program if s["op"] == "move"][-1]
            self.assertEqual(last["duration"], 4.8, name)

    def test_all_programs_finish_zero_and_stop_base(self):
        for name in SINGLE - {"wave"}:
            base = FakeChassis() if name in BASE else None
            runner = self.runner(name, base)
            frames, hands = self.run_fast(runner)
            self.assertEqual(frames[-1][0], [0.0]*7, name)
            self.assertFalse(runner.base_started)
            if base:
                self.assertEqual(base.calls[-1], ("mode", "idle"), name)
                self.assertIn(("yaw_stop",) if runner.needs_yaw else ("linear_stop",), base.calls, name)
            if name == "like":
                self.assertEqual(hands[0], ("like", [103.0, 0, 0, 0, 0, 0, 0]))
            if name in {"diagonal_point", "chin_rest"}:
                self.assertEqual(hands[0][0], "open")
                self.assertEqual(hands[1][1], list(self.repo.get(name).keyframes[0].right))

    def test_cancel_stops_both_types_of_base_worker(self):
        for name in ("sway", "forward"):
            cancel = threading.Event()
            base = FakeChassis()
            runner = self.runner(name, base, cancel)
            count = [0]
            def stop_mid_action(right, phase):
                count[0] += 1
                if count[0] == 25:
                    cancel.set()
            with self.assertRaisesRegex(RuntimeError, "停止/急停"):
                self.run_fast(runner, stop_mid_action)
            self.assertIn(("yaw_stop",) if name == "sway" else ("linear_stop",), base.calls)
            self.assertEqual(base.calls[-1], ("mode", "idle"))

    def test_missing_odometry_and_bounded_loop_timeout(self):
        base = FakeChassis()
        base.fresh = False
        with self.assertRaisesRegex(RuntimeError, "fresh /odom"):
            self.runner("sway", base)
        base.fresh = True
        base.is_reached = False
        runner = self.runner("turn_grip", base)
        held = []
        steps = [{"op": "loop_until", "max_iterations": 3, "cycles": [[{"op": "wait", "seconds": 0.02}]]}]
        with self.assertRaisesRegex(RuntimeError, "chassis target timeout"):
            runner._execute(steps, lambda *args: held.append(args), lambda event: None, FakeTracker())
        self.assertEqual(len(held), 3)
        base.is_reached = True
        held.clear()
        runner._execute(steps, lambda *args: held.append(args), lambda event: None, FakeTracker())
        self.assertFalse(held)

    def test_original_rotation_bounds_and_dwell_configuration(self):
        dance = list(walk_steps(self.repo.get("dance").program))
        rotation = next(s for s in dance if s["op"] == "yaw_target" and s.get("reference") == "current")
        self.assertEqual((rotation["degrees"], rotation["timeout"], rotation["max_speed"], rotation["travel_degrees"]), (360, 18, 0.6, 540))
        correction = next(s for s in dance if s["op"] == "yaw_final")
        self.assertEqual((correction["degrees"], correction["timeout"], correction["max_speed"], correction["guard_degrees"]), (360, 8, 0.4, 90))
        self.assertEqual([s["max_iterations"] for s in dance if s["op"] == "loop_until"], [34, 50])
        turn = list(walk_steps(self.repo.get("turn_grip").program))
        self.assertEqual(next(s["max_iterations"] for s in turn if s["op"] == "loop_until"), 100)
        # shakehand 保持原样；handshake 属游戏候选，停留已延长到 5s
        self.assertIn({"op": "wait", "seconds": 3}, self.repo.get("shakehand").program)
        self.assertIn({"op": "wait", "seconds": 5.0, "phase": "still"}, self.repo.get("handshake").program)

    def test_schema_rejects_unsafe_instructions(self):
        invalid = [
            [{"op": "move", "frame": 2, "duration": 1}],
            [{"op": "move", "frame": 1, "duration": float("nan")}],
            [{"op": "yaw_final"}, {"op": "move", "frame": 1, "duration": 1}],
            [{"op": "move", "frame": 0, "duration": 1}],
            [{"op": "move", "frame": 1, "duration": 1, "settle_timeout": -1}],
        ]
        for steps in invalid:
            with self.assertRaises(ValueError):
                validate_program(steps, 2)

    def test_hand_gestures_match_original_protocol(self):
        expected = {"half_heart": ((2,3,3,3,3,8),150), "like": ((0,10,10,10,10,0),150),
                    "point": ((4,0,10,10,10,6),150), "chin_rest": ((4,2,2,2,2,6),150)}
        hand = DexterousHand(None, "test")
        with mock.patch.object(hand, "_positions") as send:
            for name, (positions, speed) in expected.items():
                getattr(hand, name)()
                send.assert_called_with(positions, speed=speed)

    def test_fake_sdk_all_program_sessions(self):
        from test_offline import FakeSDKModule, FakeController
        # Synthetic controller limits for protocol/session testing ONLY.
        # Real execution still uses each controller's get_limit unchanged.
        for name in SINGLE - {"wave"}:
            FakeController.instances = []
            FakeController.limit_overrides = {4: (-3, 125)}
            coordinator = EmiCoordinator(sdk_module=FakeSDKModule, logger=lambda message: None)
            coordinator.chassis = FakeChassis() if name in BASE else None
            clock = [0.0]
            def advance(seconds):
                clock[0] += max(seconds, 0.020)
            with tempfile.TemporaryDirectory() as folder, \
                 mock.patch("action_program.time.monotonic", lambda: clock[0]), \
                 mock.patch("emi_coordinator.high_precision_delay", advance), \
                 mock.patch("emi_coordinator.SERVO_STOP_WAIT_SEC", 0.0), \
                 mock.patch("emi_coordinator.TRACE_OUTPUT_DIR", Path(folder)):
                success = coordinator.play_document(self.repo.get(name))
            self.assertTrue(success, (name, coordinator.last_error))
            self.assertTrue(coordinator.safe_to_reuse, name)
            self.assertEqual(len(FakeController.instances), 1, name)
            ctrl = FakeController.instances[0]
            self.assertFalse(ctrl.connected, name)
            self.assertFalse(ctrl.robots[1].power_on, name)
            self.assertEqual(ctrl.robots[1].servo_j_packets[-1][:7], [0.0]*7, name)
        FakeController.limit_overrides = {}

    def test_original_point_outside_controller_limit_is_rejected_not_clamped(self):
        from test_offline import FakeSDKModule, FakeController
        FakeController.instances = []
        FakeController.limit_overrides = {}
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule, logger=lambda message: None)
        self.assertFalse(coordinator.play_document(self.repo.get("chin_rest")))
        self.assertIn("J4", coordinator.last_error)
        robot = FakeController.instances[0].robots[1]
        self.assertFalse(robot.servo_j_packets)
        self.assertFalse(robot.power_on)
        self.assertEqual(self.repo.get("chin_rest").keyframes[0].right[3], 119)

    def test_dance_sequence_failure_recovers_only_when_not_cancelled(self):
        from action_program import ProgramSequenceFailure
        for cancelled in (False, True):
            base = FakeChassis()
            runner = self.runner("dance", base)
            runner.prepare_chassis()
            runner.frame = 12
            runner.current = runner.poses[12]
            if cancelled:
                runner.cancel.set()
            clock = [0.0]
            frames = []
            def tick(left, right, phase):
                clock[0] += 0.020
                frames.append(list(right))
            with mock.patch.object(runner, "_execute", side_effect=ProgramSequenceFailure("synthetic sequencing failure")), \
                 mock.patch("action_program.time.monotonic", lambda: clock[0]):
                with self.assertRaises(ProgramSequenceFailure):
                    runner.run(tick, lambda event: None, FakeTracker())
            if cancelled:
                self.assertFalse(frames)
                self.assertTrue(runner.cancel.is_set())
            else:
                self.assertEqual(frames[-1], [0.0]*7)
            self.assertEqual(base.calls[-1], ("mode", "idle"))

    def test_ros_aliases_match_original_single_arm_commands(self):
        if not SOURCE.exists():
            self.skipTest("original reference source is not bundled")
        original = ast.parse(SOURCE.read_text(encoding="utf-8-sig"))
        node = next(n for n in original.body if isinstance(n, ast.ClassDef) and n.name == "DualArmActionNode")
        init = next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
        assignment = next(n for n in init.body if isinstance(n, ast.AnnAssign) and
                          isinstance(n.target, ast.Attribute) and n.target.attr == "actions")
        old_aliases = ast.literal_eval(assignment.value)
        migrated = ast.parse((ROOT / "dual_arm_fist_stretch.py").read_text(encoding="utf-8"))
        new_aliases = ast.literal_eval(next(n.value for n in migrated.body
                                          if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
                                          and n.target.id == "ACTION_NAMES"))
        for command, action in old_aliases.items():
            if action.startswith("right_"):
                self.assertEqual(new_aliases[command], action[len("right_"):], command)

    def test_base_fault_cancel_is_not_reported_as_user_stop(self):
        from test_offline import FakeSDKModule, FakeController, FakeRobot
        FakeController.instances = []
        FakeController.limit_overrides = {}
        cancel = threading.Event()
        base = FakeChassis()
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule, logger=lambda message: None)
        coordinator.chassis = base
        original_send = FakeRobot.servo_j
        count = [0]
        def faulty_send(robot, *args, **kwargs):
            result = original_send(robot, *args, **kwargs)
            count[0] += 1
            if count[0] == 30:
                base.failed = "/odom feedback stale while turning"
                cancel.set()
            return result
        with tempfile.TemporaryDirectory() as folder, \
             mock.patch.object(FakeRobot, "servo_j", faulty_send), \
             mock.patch("emi_coordinator.high_precision_delay", lambda seconds: None), \
             mock.patch("emi_coordinator.TRACE_OUTPUT_DIR", Path(folder)):
            success = coordinator.play_document(self.repo.get("sway"), cancel)
        self.assertFalse(success)
        self.assertIn("底盘控制失败", coordinator.last_error)
        self.assertNotIn("收到停止/急停请求", coordinator.last_error)
        self.assertTrue(coordinator.safe_to_reuse)
        self.assertIn(("yaw_stop",), base.calls)
        self.assertFalse(FakeController.instances[0].robots[1].power_on)

    def test_failed_base_stop_prohibits_reuse(self):
        from test_offline import FakeSDKModule, FakeController, FakeRobot
        FakeController.instances = []
        FakeController.limit_overrides = {}
        base = FakeChassis()
        base.stop_ok = False
        coordinator = EmiCoordinator(sdk_module=FakeSDKModule, logger=lambda message: None)
        coordinator.chassis = base
        cancel = threading.Event()
        original_send = FakeRobot.servo_j
        count = [0]
        def stop(robot, *args, **kwargs):
            result = original_send(robot, *args, **kwargs)
            # Thumb-opening hold packets precede the linear worker. This test
            # must cancel after that worker starts, not during the hand lead.
            if any(call[0] == "linear_start" for call in base.calls):
                count[0] += 1
            if count[0] == 30:
                cancel.set()
            return result
        with tempfile.TemporaryDirectory() as folder, \
             mock.patch.object(FakeRobot, "servo_j", stop), \
             mock.patch("emi_coordinator.high_precision_delay", lambda seconds: None), \
             mock.patch("emi_coordinator.TRACE_OUTPUT_DIR", Path(folder)):
            self.assertFalse(coordinator.play_document(self.repo.get("forward"), cancel))
        self.assertFalse(coordinator.safe_to_reuse)
        self.assertFalse(FakeController.instances[0].robots[1].power_on)


if __name__ == "__main__":
    unittest.main(verbosity=2)
