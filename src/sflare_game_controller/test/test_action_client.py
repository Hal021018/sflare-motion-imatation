"""动作服务状态机的离线测试（不依赖 ROS、不连接硬件）。"""

import unittest

from sflare_game_controller.action_protocol import (
    ActionSequencer,
    ActionServiceError,
    LIMIT_RECOVERY_FAILED,
    READY,
    error_code_for,
)


class FakeTransport:
    """按脚本依次推进状态，模拟动作服务的状态话题。"""

    def __init__(self, initial=READY, transitions=()):
        self.current = initial
        self.transitions = list(transitions)
        self.published = []

    def publish(self, command):
        self.published.append(command)

    def status(self):
        return self.current

    def wait_for(self, predicate, timeout):
        while self.transitions:
            value = self.transitions.pop(0)
            self.current = value
            if predicate(value):
                return value
        return None


def build(initial=READY, transitions=()):
    transport = FakeTransport(initial, transitions)
    sequencer = ActionSequencer(transport.publish, transport.wait_for, transport.status)
    return sequencer, transport


class TestStatusMapping(unittest.TestCase):
    def test_terminal_states_map_to_codes(self):
        self.assertEqual(error_code_for('physical_alarm'), 'EMERGENCY_STOP')
        self.assertEqual(error_code_for('recover_required'), 'EMERGENCY_STOP')
        self.assertEqual(error_code_for('power_not_ready'), 'POWER_NOT_READY')
        self.assertEqual(error_code_for('connection_failed'), 'SERVICE_UNAVAILABLE')
        self.assertEqual(error_code_for('limit_recovery_failed'), 'OVERTRAVEL')
        self.assertEqual(error_code_for('invalid_target'), 'ACTION_REJECTED')
        self.assertEqual(error_code_for('fatal_network'), 'ACTION_FAILED')

    def test_transient_states_are_not_terminal(self):
        for state in (READY, 'busy', 'releasing_limit'):
            self.assertIsNone(error_code_for(state), state)


class TestEnsureReady(unittest.TestCase):
    def test_ready_passes(self):
        sequencer, _ = build()
        self.assertEqual(sequencer.ensure_ready(), READY)

    def test_missing_status(self):
        sequencer, _ = build(initial=None)
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.ensure_ready()
        self.assertEqual(ctx.exception.code, 'SERVICE_UNAVAILABLE')

    def test_busy_is_robot_busy(self):
        sequencer, _ = build(initial='busy')
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.ensure_ready()
        self.assertEqual(ctx.exception.code, 'ROBOT_BUSY')

    def test_alarm_is_emergency_stop(self):
        sequencer, _ = build(initial='physical_alarm')
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.ensure_ready()
        self.assertEqual(ctx.exception.code, 'EMERGENCY_STOP')


class TestPlay(unittest.TestCase):
    def test_normal_flow(self):
        sequencer, transport = build(transitions=['busy', READY])
        sequencer.play('like', 5.0, 30.0)
        self.assertEqual(transport.published, ['like'])

    def test_not_started(self):
        sequencer, _ = build(transitions=[])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.play('like', 5.0, 30.0)
        self.assertEqual(ctx.exception.code, 'ACTION_NOT_STARTED')

    def test_rejected_at_start(self):
        sequencer, _ = build(transitions=['unknown_command'])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.play('nope', 5.0, 30.0)
        self.assertEqual(ctx.exception.code, 'ACTION_REJECTED')

    def test_fault_during_action(self):
        sequencer, _ = build(transitions=['busy', 'physical_alarm'])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.play('like', 5.0, 30.0)
        self.assertEqual(ctx.exception.code, 'EMERGENCY_STOP')

    def test_action_timeout(self):
        sequencer, _ = build(transitions=['busy'])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.play('like', 5.0, 30.0)
        self.assertEqual(ctx.exception.code, 'ACTION_TIMEOUT')

    def test_limit_failure_maps_to_overtravel(self):
        sequencer, _ = build(transitions=['busy', LIMIT_RECOVERY_FAILED])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.play('like', 5.0, 30.0)
        self.assertEqual(ctx.exception.code, 'OVERTRAVEL')

    def test_refuses_when_busy(self):
        sequencer, transport = build(initial='busy')
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.play('like', 5.0, 30.0)
        self.assertEqual(ctx.exception.code, 'ROBOT_BUSY')
        self.assertEqual(transport.published, [])


class TestRecoverLimits(unittest.TestCase):
    def test_normal_flow(self):
        sequencer, transport = build(transitions=['releasing_limit', READY])
        sequencer.recover_limits(60.0)
        self.assertEqual(transport.published, ['recover_limit'])

    def test_failure(self):
        sequencer, _ = build(transitions=['releasing_limit', LIMIT_RECOVERY_FAILED])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.recover_limits(60.0)
        self.assertEqual(ctx.exception.code, 'OVERTRAVEL')

    def test_not_started(self):
        sequencer, _ = build(transitions=[])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.recover_limits(60.0)
        self.assertEqual(ctx.exception.code, 'ACTION_NOT_STARTED')

    def test_alarm_during_recovery(self):
        sequencer, _ = build(transitions=['releasing_limit', 'physical_alarm'])
        with self.assertRaises(ActionServiceError) as ctx:
            sequencer.recover_limits(60.0)
        self.assertEqual(ctx.exception.code, 'EMERGENCY_STOP')


class TestStop(unittest.TestCase):
    def test_stop_publishes_command(self):
        sequencer, transport = build()
        sequencer.stop()
        self.assertEqual(transport.published, ['stop'])


if __name__ == '__main__':
    unittest.main()
