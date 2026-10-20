import json
import time
from types import SimpleNamespace
from test_imitate_node import topic_node


def prepare(node):
    controls = []
    node._control = lambda command, ticket=None, **kw: controls.append((command, ticket, kw))
    node._topic_gate.acquire()
    node._execute_topic('arm_demo', {'act_name': 'like'})
    now = [time.monotonic()+.1]
    event(node, 'started')
    return controls, now


def event(node, kind, **kwargs):
    node._on_teleop_event(SimpleNamespace(data=json.dumps(dict(
        event=kind, round_id=node._round_id, session_id='real-session-1', **kwargs))))


def finish_pause(node):
    ticket = node._capture.ticket
    event(node, 'paused', request_id=ticket.request_id)
    node._capture_worker.join(timeout=3)
    assert not node._capture_worker.is_alive()
    return ticket


def test_button_early_samples_once_then_requests_reset(topic_node):
    node, results = topic_node
    controls, now = prepare(node)
    now[0] += 12
    event(node, 'button')
    assert controls[-1][0] == 'pause_for_sample'
    assert not any(kind == 'score' for kind, _ in results)
    ticket = finish_pause(node)
    assert ticket.reason == 'button'
    assert sum(kind == 'score' for kind, _ in results) == 1
    assert controls[-1][0] == 'reset_after_sample'
    event(node, 'button')
    now[0] += 100
    node._capture_timer()
    assert sum(command == 'reset_after_sample' for command, _, _ in controls) == 1


def test_watchdog_never_auto_samples_without_button(topic_node):
    node, results = topic_node
    controls, now = prepare(node)
    for _ in range(1000):node._capture_timer()
    assert node._capture.state == 'running'
    assert controls == [('arm_round', None, {'capture_trigger': 'button'})]
    assert not any(kind == 'score' for kind, _ in results)


def test_no_pause_ack_no_sample_or_reset(topic_node):
    node, results = topic_node
    controls, now = prepare(node)
    event(node, 'button')
    node._capture_wait_deadline = time.monotonic()-1
    node._capture_timer()
    assert node._capture.state == 'failed'
    assert not any(kind == 'score' for kind, _ in results)
    assert not any(command == 'reset_after_sample' for command, _, _ in controls)
    assert controls[-1][0] == 'stop_without_reset'


def test_sample_failure_cannot_reset(topic_node):
    node, results = topic_node
    controls, now = prepare(node)
    event(node, 'button')
    node._invoke = lambda name: dict(success='0', message='encoder failed', data={})
    finish_pause(node)
    assert node._capture.state == 'failed'
    assert not any(command == 'reset_after_sample' for command, _, _ in controls)


def test_estop_invalidates_late_pause_ack(topic_node):
    node, results = topic_node
    controls, now = prepare(node)
    event(node, 'button')
    ticket = node._capture.ticket
    event(node, 'emergency_stop')
    event(node, 'paused', request_id=ticket.request_id)
    assert node._capture_worker is None
    assert not any(command == 'reset_after_sample' for command, _, _ in controls)


def test_direct_estop_clears_reference_without_driver_event(topic_node):
    node, results = topic_node
    controls, now = prepare(node)
    event(node, 'button')
    ticket = node._capture.ticket
    node._on_capture_estop(SimpleNamespace(data=1))
    assert node._reference_positions is None
    event(node, 'paused', request_id=ticket.request_id)
    assert node._capture_worker is None
    assert not any(command == 'reset_after_sample' for command, _, _ in controls)
