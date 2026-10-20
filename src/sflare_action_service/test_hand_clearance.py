"""Hardware-free protocol and ordering tests for the shared hand gestures."""
import threading
import tempfile
import time
from pathlib import Path
import unittest
from unittest import mock
from emi_coordinator import DexterousHand


class HandClearanceTests(unittest.TestCase):
    def test_open_thumb_before_four_fingers_without_reclosing(self):
        events = []
        hand = DexterousHand(None, 'test')
        hand._send = lambda command, data: events.append((command, data))
        hand.wait = lambda seconds: events.append(('wait', seconds))
        hand.open()
        self.assertEqual([e[0] for e in events], [0x4C, 0x4C, 'wait', 0x50])
        self.assertEqual(events[0][1], bytes((0, 0, 0, 120)))
        self.assertEqual(events[1][1][0], 5)
        self.assertEqual(events[2][1], .5)
        self.assertEqual([int.from_bytes(events[-1][1][i:i+2], 'little')
                          for i in range(0, 15, 3)], [0]*5)

    def test_both_fist_apis_reduce_thumb_only(self):
        hand = DexterousHand(None, 'test')
        with mock.patch.object(hand, '_positions') as send:
            for method in [hand.safe_fist, hand.hold_stretch_grip]:
                method()
                self.assertEqual(send.call_args.args[0], (7, 10, 10, 10, 10, 5.5))

    def test_cancel_during_lead_never_opens_four_fingers(self):
        hand = DexterousHand(None, 'test')
        hand.wait = mock.Mock(side_effect=RuntimeError('cancelled'))
        with mock.patch.object(hand, '_send') as send:
            with self.assertRaises(RuntimeError):
                hand.open()
            self.assertEqual([c.args[0] for c in send.call_args_list], [0x4C, 0x4C])

    def test_thumb_send_failure_does_not_continue(self):
        hand = DexterousHand(None, 'test')
        with mock.patch.object(hand, '_send', side_effect=RuntimeError('serial error')) as send:
            with self.assertRaises(RuntimeError):
                hand.open()
            self.assertEqual(send.call_count, 1)

    def test_single_finger_packet_checksum(self):
        controller = mock.Mock()
        controller.com_send.return_value = 0
        hand = DexterousHand(controller, 'test')
        hand._finger_position(0, 0)
        packet = bytes.fromhex(controller.com_send.call_args.args[1])
        self.assertEqual(packet[:6], bytes((0x55, 0xAA, 2, 1, 0x4C, 4)))
        checksum = 0
        for value in packet[2:-1]:
            checksum ^= value
        self.assertEqual(packet[-1], checksum)

    def test_live_hand_transition_holds_emi_target(self):
        from test_offline import FakeSDKModule, FakeController, FakeRobot
        from action_pipeline import ActionRepository
        from emi_coordinator import EmiCoordinator
        events = []
        original_send = FakeRobot.servo_j
        original_com = FakeController.com_send
        def servo(robot, q, *args, **kwargs):
            events.append(('servo', tuple(q)))
            return original_send(robot, q, *args, **kwargs)
        def com(ctrl, port, data):
            frame = bytes.fromhex(data)
            events.append(('command', frame[4]))
            return original_com(ctrl, port, data)
        FakeController.instances = []
        FakeController.limit_overrides = {}
        coordinator = EmiCoordinator(FakeSDKModule, lambda text: None)
        with tempfile.TemporaryDirectory() as folder, \
             mock.patch.object(FakeRobot, 'servo_j', servo), \
             mock.patch.object(FakeController, 'com_send', com), \
             mock.patch('emi_coordinator.high_precision_delay', lambda seconds: time.sleep(.001)), \
             mock.patch('emi_coordinator.SERVO_STOP_WAIT_SEC', 0), \
             mock.patch('emi_coordinator.TRACE_OUTPUT_DIR', Path(folder)):
            self.assertTrue(coordinator.play_document(ActionRepository(Path(__file__).with_name('actions')).get('wave')))
        groups = []
        for index, event in enumerate(events):
            if event == ('command', 0x4C):
                tail = next((i for i in range(index+1, len(events)) if events[i] == ('command', 0x50)), None)
                if tail is not None:
                    points = [e[1] for e in events[index+1:tail] if e[0] == 'servo']
                    if points:
                        groups.append(points)
        self.assertTrue(groups, 'Runtime thumb transition must continue EMI sending')
        self.assertTrue(all(all(p == group[0] for p in group) for group in groups))

    def test_cancel_during_cleanup_still_closes_com_and_confirms_poweroff(self):
        from test_offline import FakeSDKModule, FakeController
        from action_pipeline import ActionRepository
        from emi_coordinator import EmiCoordinator
        cancel = threading.Event()
        calls = []
        def open_hand(hand):
            calls.append('open')
            if len(calls) == 3:  # startup, runtime event, final cleanup
                cancel.set()
                raise RuntimeError('收到停止/急停请求，中断灵巧手动作')
        FakeController.instances = []
        FakeController.limit_overrides = {}
        coordinator = EmiCoordinator(FakeSDKModule, lambda text: None)
        with tempfile.TemporaryDirectory() as folder, \
             mock.patch.object(DexterousHand, 'open', open_hand), \
             mock.patch.object(FakeController, 'com_close', return_value=0) as close, \
             mock.patch('emi_coordinator.high_precision_delay', lambda seconds: None), \
             mock.patch('emi_coordinator.SERVO_STOP_WAIT_SEC', 0), \
             mock.patch('emi_coordinator.TRACE_OUTPUT_DIR', Path(folder)):
            coordinator.play_document(ActionRepository(Path(__file__).with_name('actions')).get('wave'), cancel)
        self.assertTrue(cancel.is_set())
        self.assertEqual(len(calls), 3)
        self.assertTrue(coordinator.safe_to_reuse)
        self.assertGreaterEqual(close.call_count, 2)
        self.assertFalse(FakeController.instances[0].robots[1].power_on)


if __name__ == '__main__':
    unittest.main()
