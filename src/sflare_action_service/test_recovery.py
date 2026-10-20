"""Recovery, ownership and failure tests. All SDK and time are simulated."""
import ast
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest import mock

import limit_recovery as recovery
import emi_coordinator
from controller_guard import CheckedSDK, probe_available, recovery_arm
import test_action_session as session_tests
from test_offline import FakeController, FakeRobot, FakeSDKModule
from action_pipeline import ActionRepository
from action_session import extract_error_codes


class Values:
    def __init__(self, values):
        self.values = values

    def toList(self):
        return list(self.values)


class Robot:
    def __init__(self, position, step=-0.5):
        self.position = list(position)
        self.step = step
        self.calls = []
        self.moving = False
        self.axis = 0

    def get_joint_pos(self, **kwargs):
        return Values(self.position)

    def get_joint_speed(self, **kwargs):
        return [0.0] * 7

    def get_limit(self, axis, **kwargs):
        return [-100.0, 100.0]

    def move_along(self, axis, speed, **kwargs):
        self.calls.append(('jog', axis, speed))
        self.axis = int(axis[1:]) - 1
        self.moving = True
        return 0

    def move_stop_jog(self, **kwargs):
        self.calls.append(('stop',))
        if self.moving:
            self.position[self.axis] += self.step
        self.moving = False
        return 0

    def __getattr__(self, name):
        def method(*args, **kwargs):
            self.calls.append((name, *args))
            return 0
        return method


class Controller:
    def __init__(self, diagnostic='-5031 Robot:1,Axis:4', clear_result=0):
        self.diagnostic = diagnostic
        self.clear_result = clear_result
        self.calls = []

    def check_error(self, **kwargs):
        return [self.diagnostic] if self.diagnostic else []

    def clear_alarm(self, **kwargs):
        self.calls.append('clear')
        return self.clear_result

    def set_power_enable(self, enable, **kwargs):
        self.calls.append(('power', enable))
        return 0


class RecoveryTests(unittest.TestCase):
    def test_negative_joint_decimals_are_not_sdk_fault_codes(self):
        self.assertEqual(extract_error_codes('J1=-66.000 limit=[-65.000,195.000]; -5032'),
                         frozenset({-5032}))
    def engine(self, position=None, diagnostic='-5031 Robot:1,Axis:4', step=-0.5, clear_result=0):
        self.robot = Robot(position or [0, 0, 0, 99, 0, 0, 0], step)
        self.ctrl = Controller(diagnostic, clear_result)
        self.arm = recovery_arm('right', self.ctrl, self.robot, diagnostic)
        self.cancel = threading.Event()
        self.allowed = True
        self.clock = 0.0
        self.engine_instance = recovery.LimitRecovery([self.arm], object(), self.cancel,
                                                      lambda: self.allowed, lambda text: None)
        clock = types.SimpleNamespace(monotonic=lambda: self.clock, sleep=self.advance)
        patch = mock.patch.object(recovery, 'time', clock)
        patch.start()
        self.addCleanup(patch.stop)
        def wait(seconds, phase):
            self.engine_instance._check_interruption(phase)
            self.advance(seconds)
            self.engine_instance._check_interruption(phase)
        self.engine_instance._wait_interruptibly = wait
        return self.engine_instance

    def advance(self, seconds):
        self.clock += seconds

    def jogs(self):
        return [c for c in self.robot.calls if c[0] == 'jog']

    def test_positive_limit_only_moves_inward_and_restores(self):
        self.assertTrue(self.engine().recover_verified_limit())
        self.assertTrue(all(c[2] == -15 for c in self.jogs()))
        self.assertLessEqual(self.robot.position[3], 97)
        self.assertIn(('set_jog_mode', 1, False), self.robot.calls)
        self.assertIn(('set_frame_type', 2), self.robot.calls)

    def test_negative_limit_moves_positive(self):
        engine = self.engine([0, 0, 0, -99, 0, 0, 0], '-5032 Robot:1,Axis:4', .5)
        self.assertTrue(engine.recover_verified_limit())
        self.assertTrue(all(c[2] == 15 for c in self.jogs()))

    def test_actual_overrun_does_not_need_reported_axis(self):
        self.assertTrue(self.engine([0, 0, 0, 101, 0, 0, 0], '-5031').recover_verified_limit())

    def test_far_from_limit_alarm_does_not_authorize_motion(self):
        self.assertFalse(self.engine([0] * 7).recover_verified_limit())
        self.assertFalse(self.jogs())
        self.assertFalse(self.ctrl.calls)

    def test_ambiguous_near_axes_refuse_without_axis(self):
        self.assertFalse(self.engine([99, 0, 0, 99, 0, 0, 0], '-5031').recover_verified_limit())
        self.assertFalse(self.jogs())

    def test_reported_axis_narrows_ambiguous_positions(self):
        self.assertTrue(self.engine([99, 0, 0, 99, 0, 0, 0]).recover_verified_limit())
        self.assertTrue(all(c[1] == 'J4' for c in self.jogs()))

    def test_conflicting_codes_without_overrun_refuse(self):
        self.assertFalse(self.engine(diagnostic='-5031 -5032').recover_verified_limit())
        self.assertFalse(self.jogs())

    def test_nonfinite_feedback_refuses_before_power(self):
        self.assertFalse(self.engine([0, 0, 0, float('nan'), 0, 0, 0]).recover_verified_limit())
        self.assertFalse(self.ctrl.calls)

    def test_wrong_direction_stops_and_fails(self):
        self.assertFalse(self.engine(step=.5).recover_verified_limit())
        self.assertEqual(len(self.jogs()), 1)
        self.assertFalse(self.robot.moving)

    def test_excess_travel_fails_even_if_target_crossed(self):
        self.assertFalse(self.engine(step=-10).recover_verified_limit())
        self.assertFalse(self.robot.moving)

    def test_stall_has_bounded_time_and_stops(self):
        self.assertFalse(self.engine(step=0).recover_verified_limit())
        self.assertLess(self.clock, 15)
        self.assertFalse(self.robot.moving)

    def test_pulse_budget(self):
        engine = self.engine(step=-.01)
        with mock.patch.object(recovery, 'LIMIT_RECOVERY_MAX_PULSES', 2):
            self.assertFalse(engine.recover_verified_limit())
        self.assertEqual(len(self.jogs()), 2)

    def test_time_budget(self):
        engine = self.engine(step=-.01)
        with mock.patch.object(recovery, 'LIMIT_RECOVERY_MAX_DURATION_SEC', .1):
            self.assertFalse(engine.recover_verified_limit())
        self.assertFalse(self.robot.moving)

    def test_multiple_reported_axes_refuse_without_overrun(self):
        engine = self.engine(diagnostic='-5031 Robot:1,Axis:1; Robot:1,Axis:4')
        self.assertFalse(engine.recover_verified_limit())
        self.assertFalse(self.jogs())

    def test_truncated_feedback_refuses(self):
        self.assertFalse(self.engine([99]).recover_verified_limit())
        self.assertFalse(self.ctrl.calls)

    def test_cancel_before_power_does_not_clear_or_move(self):
        engine = self.engine()
        self.cancel.set()
        with self.assertRaises(recovery.ActionCancelled):
            engine.recover_verified_limit()
        self.assertFalse(self.ctrl.calls)

    def test_joint_mode_restore_failure_is_not_success(self):
        engine = self.engine()
        self.robot.set_frame_type = mock.Mock(side_effect=[0, RuntimeError('-5306')])
        self.assertFalse(engine.recover_verified_limit())
        self.assertIn(-5306, engine._last_recovery_codes)

    def test_second_diagnostic_vetoes_new_collision_before_clear(self):
        engine = self.engine()
        self.ctrl.check_error = mock.Mock(side_effect=[['-5031 Robot:1,Axis:4'], ['-5201']])
        self.assertFalse(engine.recover_verified_limit())
        self.assertFalse(self.ctrl.calls)
        self.assertIn(-5201, engine._last_recovery_codes)

    def test_encoder_clear_exception_allowlist(self):
        self.assertTrue(self.engine(clear_result=-5047).recover_verified_limit())

    def test_other_clear_failure_does_not_power_or_jog(self):
        self.assertFalse(self.engine(clear_result=-5306).recover_verified_limit())
        self.assertFalse(self.jogs())
        self.assertNotIn(('power', True), self.ctrl.calls)

    def test_cancel_during_pulse_always_stops(self):
        engine = self.engine()
        wait = engine._wait_interruptibly
        def cancel_pulse(seconds, phase):
            if '脉冲' in phase:
                self.cancel.set()
            wait(seconds, phase)
        engine._wait_interruptibly = cancel_pulse
        with self.assertRaises(recovery.ActionCancelled):
            engine.recover_verified_limit()
        self.assertFalse(self.robot.moving)
        self.assertEqual(len(self.jogs()), 1)

    def test_permission_unknown_or_revoked_prevents_power(self):
        engine = self.engine()
        self.allowed = False
        with self.assertRaises(recovery.ActionCancelled):
            engine.recover_verified_limit()
        self.assertFalse(self.ctrl.calls)

    def test_new_physical_fault_vetoes_clear(self):
        engine = self.engine()
        self.ctrl.diagnostic = '-5201'
        with self.assertRaises(recovery.MotionFailure):
            engine.recover_verified_limit()
        self.assertFalse(self.ctrl.calls)

    def test_source_core_remains_original(self):
        original = Path('C:/Users/blond/Desktop/action service/01_源码/当前版本/dual_arm_motion_actions.py')
        if not original.exists():
            self.skipTest('original reference source is not bundled')
        source = ast.parse(original.read_text(encoding='utf-8-sig'))
        original_cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == 'DualArmActions')
        migrated_cls = next(n for n in ast.parse(Path(recovery.__file__).read_text(encoding='utf-8')).body
                            if isinstance(n, ast.ClassDef) and n.name == 'LimitRecovery')
        for name in ['_limit_candidates', '_reported_limit_axes', '_recover_axis_into_limit']:
            old = next(n for n in original_cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
            new = next(n for n in migrated_cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
            self.assertEqual(ast.dump(old), ast.dump(new))


class ProbeTests(unittest.TestCase):
    def test_unreachable_does_not_construct_sdk(self):
        factory = mock.Mock()
        self.assertEqual(probe_available(factory, [('right', 'ip')], 2094, lambda *args: False),
                         ('connection_failed', True))
        factory.assert_not_called()

    def test_rejected_connect_does_not_poweroff_or_disconnect(self):
        ctrl = mock.Mock()
        ctrl.connect.return_value = -20035
        self.assertEqual(probe_available(lambda: ctrl, [('right', 'ip')], 2094, lambda *args: True),
                         ('handle_busy', True))
        ctrl.set_power_enable.assert_not_called()
        ctrl.disconnect.assert_not_called()


class CoordinatorRecoveryTests(unittest.TestCase):
    def test_complete_recovery_then_poweroff_and_disconnect(self):
        events = []
        class RecoveryRobot(FakeRobot):
            def get_joint_pos(self, **kwargs):
                return Values(self.current_pos)
            def get_joint_speed(self, **kwargs):
                return [0] * 7
            def move_along(self, axis, speed, **kwargs):
                events.append('jog')
                self.current_pos[int(axis[1:]) - 1] -= .5
                return 0
            def move_stop_jog(self, **kwargs):
                events.append('jog_stop')
                return 0
            def set_jog_mode(self, *args):
                return 0
            def set_frame_type(self, *args):
                return 0
            def emi_enable(self, enable):
                events.append('emi_disable')
                return 0
        class RecoveryController(FakeController):
            def add_robot(self, robot_id):
                robot = RecoveryRobot(robot_id)
                robot.current_pos = [0, 0, 0, 96, 0, 0, 0]
                self.robots[robot_id] = robot
                return robot
            def check_error(self, **kwargs):
                events.append('diagnostic')
                return ['-5031 Robot:1,Axis:4']
            def clear_alarm(self, **kwargs):
                events.append('clear')
                return 0
            def set_power_enable(self, enable, *args, **kwargs):
                events.append('power_on' if enable else 'power_off')
                return super().set_power_enable(enable, *args, **kwargs)
            def disconnect(self):
                events.append('disconnect')
                return super().disconnect()
        sdk = types.SimpleNamespace(Controller=RecoveryController, Profile=lambda *args: None)
        coordinator = emi_coordinator.EmiCoordinator(sdk, lambda text: None)
        coordinator.recovery_permitted = lambda: True
        doc = ActionRepository(Path(__file__).with_name('actions')).get('wave')
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(recovery.LimitRecovery, '_wait_interruptibly',
                                                                       lambda engine, seconds, phase: engine._check_interruption(phase)):
            with mock.patch.object(emi_coordinator, 'TRACE_OUTPUT_DIR', Path(folder)):
                self.assertFalse(coordinator.play_document(doc, threading.Event()))
        self.assertEqual(coordinator.recovery_outcome, 'limit_recovered')
        self.assertTrue(coordinator.safe_to_reuse)
        self.assertLess(events.index('power_off'), events.index('clear'))
        self.assertLess(events.index('emi_disable'), events.index('jog'))
        self.assertEqual(events[-2:], ['power_off', 'disconnect'])

    def test_no_recovery_authorization_never_enables_jog(self):
        class DiagnosticController(FakeController):
            def check_error(self, **kwargs):
                return ['-5031 Robot:1,Axis:4']
        coordinator = emi_coordinator.EmiCoordinator(
            types.SimpleNamespace(Controller=DiagnosticController, Profile=lambda *args: None), lambda text: None)
        cancel = threading.Event()
        cancel.set()
        doc = ActionRepository(Path(__file__).with_name('actions')).get('wave')
        with mock.patch.object(recovery.LimitRecovery, 'recover_verified_limit') as recover:
            self.assertFalse(coordinator.play_document(doc, cancel))
            recover.assert_not_called()
        self.assertTrue(coordinator.safe_to_reuse)

    def test_successful_probe_poweroff_before_disconnect_no_robot(self):
        ctrl = mock.Mock()
        ctrl.connect.return_value = ctrl.set_power_enable.return_value = ctrl.disconnect.return_value = 0
        self.assertEqual(probe_available(lambda: ctrl, [('right', 'ip')], 2094, lambda *args: True),
                         ('ready', True))
        self.assertEqual([c[0] for c in ctrl.mock_calls], ['connect', 'set_power_enable', 'disconnect'])

    def test_poweroff_failure_forbids_disconnect(self):
        ctrl = mock.Mock()
        ctrl.connect.return_value = 0
        ctrl.set_power_enable.return_value = -161
        self.assertEqual(probe_available(lambda: ctrl, [('right', 'ip')], 2094, lambda *args: True),
                         ('unsafe_stop', False))
        ctrl.disconnect.assert_not_called()


class MigratedSessionTests(unittest.TestCase):
    setUp = session_tests.SessionTests.setUp
    command = session_tests.SessionTests.command
    estop = session_tests.SessionTests.estop
    run_session = session_tests.SessionTests.run_session
    def test_fatal_network_exits_only_after_coordinator_cleanup(self):
        self.command('wave')
        def play(coordinator, doc, cancel):
            coordinator.last_error = '-20038'
            return False
        self.run_session(play)
        self.node._exit_on_fatal.assert_called_once_with('fatal_network')

    def test_recovery_success_does_not_resume_or_gozero(self):
        self.command('wave')
        self.command('gozero')
        played = []
        def play(coordinator, doc, cancel):
            played.append(doc.name)
            coordinator.diagnostic_codes = frozenset({-5031})
            coordinator.recovery_outcome = 'limit_recovered'
            return False
        self.run_session(play)
        self.assertEqual(played, ['wave'])
        self.assertEqual(self.states[-1], 'limit_recovered')

    def test_recovery_physical_alarm_overrides_limit_success(self):
        self.command('wave')
        def play(coordinator, doc, cancel):
            coordinator.recovery_outcome = 'limit_recovered'
            coordinator.recovery_codes = frozenset({-5201})
            return False
        self.run_session(play)
        self.assertEqual(self.states[-1], 'physical_alarm')

    def test_connection_monitor_blocks_new_action_and_returns_ready(self):
        self.command('wave')
        session = self.node._session
        sdk = types.SimpleNamespace(Controller=mock.Mock())
        coordinator = types.SimpleNamespace(_load_sdk=lambda: sdk)
        def probe(*args):
            self.command('handshake')
            self.command('gozero')
            self.assertEqual(session.action, 'wave')
            self.assertFalse(session.cancel_event.is_set())
            return 'ready', True
        with mock.patch.object(self.module, 'probe_available', side_effect=probe):
            state = self.module.EmiActionNode._monitor_connection(self.node, session, coordinator)
        self.assertEqual(state, 'ready')

    def test_monitor_estop_does_not_publish_ready(self):
        self.command('wave')
        session = self.node._session
        coordinator = types.SimpleNamespace(_load_sdk=lambda: types.SimpleNamespace(Controller=mock.Mock()))
        def probe(*args):
            self.estop(1)
            return 'ready', True
        with mock.patch.object(self.module, 'probe_available', side_effect=probe):
            state = self.module.EmiActionNode._monitor_connection(self.node, session, coordinator)
        self.assertEqual(state, 'physical_alarm')

    def test_timeout_without_estop_diagnostic_is_fatal(self):
        self.command('wave')
        def play(coordinator, doc, cancel):
            self.estop(1)
            self.estop(0)
            self.command('recover')
            coordinator.last_error = '-20039'
            return False
        self.run_session(play)
        self.node._exit_on_fatal.assert_called_once_with('fatal_network')

    def test_mixed_connection_and_collision_does_not_probe(self):
        self.command('wave')
        def play(coordinator, doc, cancel):
            coordinator.last_error = '-20035 -5201'
            return False
        self.run_session(play)
        self.node._monitor_connection.assert_not_called()
        self.assertEqual(self.states[-1], 'physical_alarm')

    def test_fatal_exit_publishes_then_terminates(self):
        with mock.patch.object(self.module.os, '_exit', side_effect=SystemExit(1)) as exit_process:
            with self.assertRaises(SystemExit):
                self.module.EmiActionNode._exit_on_fatal(self.node, 'fatal_network')
        self.assertEqual(self.states[-1], 'fatal_network')
        self.assertTrue(self.node._stopping)
        exit_process.assert_called_once_with(1)


if __name__ == '__main__':
    unittest.main()
