"""Test real ROS callback/worker methods using stub message types; no hardware."""
import importlib.util
import os
import sys
import threading
import types
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from action_pipeline import ActionRepository
from action_session import ActionSession, PRIORITY_ZERO_BLOCKING_CODES
REAL_THREAD = threading.Thread


class Message:
    def __init__(self, data=None):
        self.data = data


def load_node_module():
    packages = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.executors", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg")}
    packages["rclpy.executors"].ExternalShutdownException = RuntimeError
    packages["rclpy.node"].Node = object
    packages["rclpy.qos"].DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    packages["rclpy.qos"].QoSProfile = lambda **kwargs: kwargs
    packages["std_msgs.msg"].String = Message
    packages["std_msgs.msg"].Int32 = Message
    spec = importlib.util.spec_from_file_location("_emi_session_test_node", Path(__file__).with_name("dual_arm_fist_stretch.py"))
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, packages):
        spec.loader.exec_module(module)
    return module


class DeferredThread:
    def __init__(self, target, args, **kwargs):
        self.target, self.args = target, args
        self.started = False
        self.joined = False

    def start(self):
        self.started = True

    def join(self, timeout=None):
        self.joined = True

    def is_alive(self):
        return False


class CoordinatorStub:
    def __init__(self, callback, **kwargs):
        self.callback = callback
        self.safe_to_reuse = True
        self.last_error = None
        self.diagnostic_codes = frozenset()
        self.diagnostics_complete = True
        self.chassis = None

    def play_document(self, doc, cancel):
        return self.callback(self, doc, cancel)


class SessionTests(unittest.TestCase):
    def setUp(self):
        # These existing tests exercise the retained EMI backend, including its
        # SDK stubs. New move_j routing/priority-zero tests live in test_movej.
        backend = mock.patch.dict(os.environ, {'PALLAS_GAME_MOTION_MODE': 'emi'})
        backend.start()
        self.addCleanup(backend.stop)
        self.module = load_node_module()
        self.node = self.module.EmiActionNode.__new__(self.module.EmiActionNode)
        self.node.mode = "single"
        self.node._lock = threading.Lock()
        self.node._session = None
        self.node._stopping = False
        self.node._e_stop_pressed = False
        self.node._recover_required = False
        self.node._status_text = "ready"
        self.node.repository = ActionRepository(Path(__file__).with_name("actions"))
        self.states = []
        self.node.publisher = types.SimpleNamespace(publish=lambda msg: self.states.append(msg.data))
        self.logger = mock.Mock()
        self.node.get_logger = lambda: self.logger
        self.node._exit_on_fatal = mock.Mock()
        self.node._monitor_connection = mock.Mock(return_value='connection_failed')
        self.node.chassis = mock.Mock()
        patch = mock.patch.object(self.module.threading, "Thread", DeferredThread)
        patch.start()
        self.addCleanup(patch.stop)

    def command(self, text):
        self.node._on_command(Message(text))

    def estop(self, value):
        self.node._on_e_stop(Message(value))

    def run_session(self, callback):
        session = self.node._session
        with mock.patch.object(self.module, "EmiCoordinator", lambda **kwargs: CoordinatorStub(callback, **kwargs)):
            self.node._run(session)
        return session

    def test_idle_stop_and_recover_do_not_connect_or_move(self):
        with mock.patch.object(self.module, "EmiCoordinator") as sdk:
            self.command("stop")
            self.command("recover")
            sdk.assert_not_called()
        self.assertIsNone(self.node._session)
        self.assertEqual(self.states[-1], "ready")

    def test_stop_has_priority_over_later_gozero(self):
        self.command("wave")
        self.command("stop")
        self.command("gozero")
        self.assertEqual(self.node._session.intent, "stop")
        self.assertTrue(self.node._cancel.is_set())
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            coordinator.last_error = "收到停止/急停请求，中断发包"
            return False
        self.run_session(play)
        self.assertEqual(played, ["wave"])
        self.assertEqual(self.states[-1], "ready")

    def test_duplicate_zero_is_consumed_once_and_keeps_right_scope(self):
        self.node.mode = "dual"
        self.command("wave")
        played = []
        def play(coordinator, doc, cancel):
            played.append((doc.name, doc.scope))
            if doc.name == "wave":
                self.command("gozero")
                self.command("gozero")
                coordinator.last_error = "收到停止/急停请求，中断发包"
                return False
            self.assertFalse(cancel.is_set())
            self.command("gozero")
            self.assertFalse(cancel.is_set())
            return True
        self.run_session(play)
        self.assertEqual(played, [("wave", "right"), ("gozero", "right")])
        self.assertIsNone(self.node._session)

    def test_standalone_zero_uses_host_scope_and_merges_duplicates(self):
        self.node.mode = "dual"
        self.command("gozero")
        session = self.node._session
        self.command("gozero")
        self.assertIs(self.node._session, session)
        self.assertEqual(session.scope, "dual")
        self.assertTrue(session.zeroing)
        self.assertFalse(session.cancel_event.is_set())

    def test_normal_commands_are_not_queued_while_busy(self):
        self.command("wave")
        session = self.node._session
        self.command("handshake")
        self.command("recover")
        self.assertIs(self.node._session, session)
        self.assertEqual(session.action, "wave")
        self.assertEqual(self.states[-1], "busy")

    def test_estop_blocks_zero_and_actions_until_recover(self):
        self.estop(1)
        self.command("wave")
        self.command("gozero")
        self.command("recover")
        self.assertIsNone(self.node._session)
        self.assertEqual(self.states[-1], "physical_alarm")
        self.estop(0)
        self.command("wave")
        self.assertIsNone(self.node._session)
        self.assertEqual(self.states[-1], "recover_required")
        self.command("recover")
        self.assertEqual(self.states[-1], "ready")
        self.node.chassis._publish_chassis_speed.assert_called_once_with(0.0, 0.0)

    def test_early_recover_waits_for_estop_session_cleanup(self):
        self.command("wave")
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            self.estop(1)
            self.estop(0)
            self.command("recover")
            self.assertTrue(self.node._recover_required)
            self.assertEqual(self.states[-1], "recover_required")
            self.command("handshake")
            self.command("gozero")
            coordinator.last_error = "[Error] -20039 after external emergency stop; check_error=-5029"
            return False
        self.run_session(play)
        self.assertEqual(played, ["wave"])
        self.assertFalse(self.node._recover_required)
        self.assertIsNone(self.node._session)
        self.assertEqual(self.states[-1], "ready")

    def test_new_estop_press_revokes_early_recover(self):
        self.command("wave")
        def play(coordinator, doc, cancel):
            self.estop(1)
            self.estop(0)
            self.command("recover")
            self.estop(1)
            coordinator.last_error = "收到停止/急停请求，中断发包"
            return False
        session = self.run_session(play)
        self.assertFalse(session.recover_after_cleanup)
        self.assertTrue(self.node._recover_required)
        self.assertEqual(self.states[-1], "physical_alarm")

    def test_estop_during_completion_cannot_publish_ready(self):
        self.command("wave")
        def play(coordinator, doc, cancel):
            self.estop(1)
            self.estop(0)
            return True
        self.run_session(play)
        self.assertEqual(self.states[-1], "recover_required")
        self.assertNotIn("ready", self.states)

    def test_priority_zero_arriving_during_cleanup_is_not_lost(self):
        self.command("wave")
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            if doc.name == "wave":
                self.command("gozero")
            return True
        self.run_session(play)
        self.assertEqual(played, ["wave", "gozero"])

    def test_stop_can_preempt_consumed_zero(self):
        self.command("wave")
        self.command("gozero")
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            coordinator.last_error = "收到停止/急停请求，中断发包"
            if doc.name == "gozero":
                self.command("stop")
                self.command("gozero")
                self.assertEqual(self.node._session.intent, "stop")
            return False
        self.run_session(play)
        self.assertEqual(played, ["wave", "gozero"])
        self.assertEqual(self.states[-1], "ready")

    def test_controller_faults_block_priority_zero(self):
        for code in PRIORITY_ZERO_BLOCKING_CODES:
            with self.subTest(code=code):
                self.node._recover_required = False
                self.node._fault_alarm_latched = False
                self.command("wave")
                self.command("gozero")
                played = []
                def play(coordinator, doc, cancel):
                    played.append(doc.name)
                    coordinator.diagnostic_codes = frozenset({code})
                    coordinator.last_error = "收到停止/急停请求，中断发包"
                    return False
                self.run_session(play)
                self.assertEqual(played, ["wave"])
                self.assertNotEqual(self.states[-1], "ready")

    def test_missing_diagnostics_blocks_priority_zero(self):
        self.command("wave")
        self.command("gozero")
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            coordinator.diagnostics_complete = False
            return True
        self.run_session(play)
        self.assertEqual(played, ["wave"])
        self.assertEqual(self.states[-1], "failed")

    def test_fault_categories_use_controller_diagnostics(self):
        cases = {-5201: "physical_alarm", -135: "physical_alarm",
                 -5176: "power_not_ready", -161: "power_not_ready",
                 -20236: "power_not_ready", -20035: "connection_failed",
                 -20038: "fatal_network", -20039: "fatal_network",
                 -5031: "limit_recovery_failed", -5032: "limit_recovery_failed",
                 -5306: "invalid_target"}
        for code, expected in cases.items():
            with self.subTest(code=code):
                self.assertEqual(self.node._failure_status("interrupted", {code}), expected)

    def test_historical_estop_is_filtered_only_when_released(self):
        self.assertEqual(self.node._failure_status("-5029"), "failed")
        self.node._e_stop_pressed = None
        self.assertEqual(self.node._failure_status("-5029"), "physical_alarm")

    def test_diagnostic_fault_cannot_be_hidden_by_stop_or_success(self):
        for completed in (True, False):
            for stopping in (True, False):
                self.node._recover_required = False
                self.command("wave")
                if stopping:
                    self.command("stop")
                def play(coordinator, doc, cancel):
                    coordinator.last_error = "收到停止/急停请求，中断发包" if stopping else None
                    coordinator.diagnostic_codes = frozenset({-5306})
                    return completed
                self.run_session(play)
                self.assertEqual(self.states[-1], "invalid_target")

    def test_stale_estop_does_not_block_priority_zero(self):
        self.command("wave")
        self.command("gozero")
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            coordinator.diagnostic_codes = frozenset({-5029})
            coordinator.last_error = "收到停止/急停请求，中断发包" if doc.name == "wave" else None
            return doc.name == "gozero"
        self.run_session(play)
        self.assertEqual(played, ["wave", "gozero"])
        self.assertEqual(self.states[-1], "ready")

    def test_stale_worker_cannot_clear_or_publish_over_new_session(self):
        old = ActionSession("wave", "wave", "right")
        new = ActionSession("handshake", "handshake", "right")
        self.node._session = new
        with self.node._lock:
            self.node._finish_session_locked(old, "ready")
        self.assertIs(self.node._session, new)
        self.assertFalse(self.states)

    def test_shutdown_cancels_without_sdk_from_callback(self):
        self.command("wave")
        session = self.node._session
        self.assertTrue(self.node.shutdown())
        self.assertTrue(session.cancel_event.is_set())
        self.assertEqual(session.intent, "stop")
        self.assertTrue(session.thread.joined)
        before = len(self.states)
        self.command("gozero")
        self.assertEqual(len(self.states), before)

    def test_unknown_estop_cannot_be_acknowledged(self):
        self.node._e_stop_pressed = None
        self.node._recover_required = True
        self.command("recover")
        self.assertTrue(self.node._recover_required)
        self.assertEqual(self.states[-1], "recover_required")

    def test_timer_does_not_republish_handle_busy(self):
        self.node._status_text = "handle_busy"
        self.node._republish_status()
        self.assertFalse(self.states)

    def test_old_worker_never_connects_after_losing_ownership(self):
        old = ActionSession("wave", "wave", "right")
        self.node._session = ActionSession("handshake", "handshake", "right")
        with mock.patch.object(self.module, "EmiCoordinator") as sdk:
            self.node._run(old)
            sdk.assert_not_called()
        self.assertEqual(self.node._session.action, "handshake")
        self.assertFalse(self.states)

    def test_unsafe_cleanup_never_acknowledges_deferred_recover(self):
        self.command("wave")
        def play(coordinator, doc, cancel):
            self.estop(1)
            self.estop(0)
            self.command("recover")
            coordinator.safe_to_reuse = False
            return False
        with mock.patch.object(self.module.os, "_exit", side_effect=SystemExit(1)) as exit_process:
            with self.assertRaises(SystemExit):
                self.run_session(play)
            exit_process.assert_called_once_with(1)
        self.assertTrue(self.node._recover_required)
        self.assertNotIn("ready", self.states)

    def test_real_worker_cleanup_gate_retains_priority_request(self):
        self.command("wave")
        session = self.node._session
        cleaning = threading.Event()
        release = threading.Event()
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            if doc.name == "wave":
                cleaning.set()
                if not release.wait(2.0):
                    raise RuntimeError("test cleanup barrier timeout")
            return True
        with mock.patch.object(self.module, "EmiCoordinator", lambda **kwargs: CoordinatorStub(play, **kwargs)):
            worker = REAL_THREAD(target=self.node._run, args=(session,))
            session.thread = worker
            worker.start()
            try:
                self.assertTrue(cleaning.wait(2.0))
                self.command("handshake")
                self.assertIs(self.node._session, session)
                self.command("gozero")
                self.assertEqual(self.node._session.intent, "gozero")
            finally:
                release.set()
                worker.join(2.0)
            self.assertFalse(worker.is_alive())
        self.assertEqual(played, ["wave", "gozero"])
        self.assertEqual(self.states[-1], "ready")

    def test_fake_sdk_priority_zero_reconnects_only_after_old_cleanup(self):
        from emi_coordinator import EmiCoordinator
        from test_offline import FakeSDKModule, FakeController, FakeRobot
        FakeController.instances = []
        FakeController.initial_pos = [0.0]*7
        FakeController.limit_overrides = {}
        self.addCleanup(setattr, FakeController, "initial_pos", [0.0]*7)
        self.command("wave")
        session = self.node._session
        count = [0]
        original = FakeRobot.servo_j
        def send(robot, *args, **kwargs):
            count[0] += 1
            if len(FakeController.instances) == 2:
                self.assertFalse(FakeController.instances[0].connected)
                self.assertFalse(FakeController.instances[0].robots[1].power_on)
            result = original(robot, *args, **kwargs)
            if count[0] == 25:
                FakeController.initial_pos = list(robot.current_pos)
                self.command("gozero")
            return result
        with tempfile.TemporaryDirectory() as folder, \
             mock.patch.object(self.module, "EmiCoordinator", lambda **kwargs: EmiCoordinator(sdk_module=FakeSDKModule, **kwargs)), \
             mock.patch.object(FakeRobot, "servo_j", send), \
             mock.patch("emi_coordinator.high_precision_delay", lambda seconds: None), \
             mock.patch("emi_coordinator.SERVO_STOP_WAIT_SEC", 0.0), \
             mock.patch("emi_coordinator.TRACE_OUTPUT_DIR", Path(folder)):
            self.node._run(session)
        self.assertEqual(len(FakeController.instances), 2)
        self.assertTrue(all(not ctrl.connected and not ctrl.robots[1].power_on for ctrl in FakeController.instances))
        self.assertEqual(FakeController.instances[-1].robots[1].servo_j_packets[-1][:7], [0.0]*7)
        self.assertEqual(self.states[-1], "ready")


if __name__ == "__main__":
    unittest.main(verbosity=2)
