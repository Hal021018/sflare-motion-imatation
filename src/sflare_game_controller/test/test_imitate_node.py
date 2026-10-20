import json
import pytest
from types import SimpleNamespace


@pytest.fixture
def topic_node():
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('sflare_game_interfaces.msg')
    from sflare_game_controller.imitate_node import ImitateNode
    from test_moveaction import FakeActionClient
    rclpy.init()
    node = ImitateNode()
    node.action_client = FakeActionClient()
    results = []
    node._result = lambda kind, data: results.append((kind, data))
    yield node, results
    node.destroy_node()
    rclpy.shutdown()


def test_topic_only_inventory(topic_node):
    node, results = topic_node
    assert node.command_service is None
    node._on_topic_command(SimpleNamespace(data='{"cmd":"acts","args":{}}'))
    assert len(results[0][1]['list']) == 6
    assert '敬礼' not in results[0][1]['list']


def test_check_shape(topic_node):
    node, results = topic_node
    node._topic_gate.acquire()
    node._execute_topic('arm_check', {})
    assert results == [('arm_check', {'msg': ''})]
    assert not node._topic_gate.locked()


def test_demo_uses_actual_name(topic_node):
    node, results = topic_node
    node._topic_gate.acquire()
    node._execute_topic('arm_demo', {'act_name': '点赞'})
    assert node.action_client.commands == ['like']
    assert results == [('arm_demo', {'state': 'start'}), ('arm_demo', {'state': 'end'})]
    # No fabricated power state/hold/reset on command acceptance or completion.


def test_phases_require_matching_action(topic_node):
    node, results = topic_node
    node._active_demo = 'like'
    for action, state in [('wave', 'still'), ('like', 'moving'), ('like', 'still'), ('like', 'reset')]:
        node._on_action_event(SimpleNamespace(data=json.dumps(
            {'action': action, 'type': 'phase', 'state': state})))
    assert results == [('arm_demo', {'state': s}) for s in ['moving', 'still', 'reset']]


def test_rejected_command_does_not_claim_end(topic_node):
    node, results = topic_node
    node.action_client.play_error = ('ACTION_TIMEOUT', 'timeout')
    node._topic_gate.acquire()
    node._execute_topic('arm_demo', {'act_name': 'like'})
    assert results[-1][0] == 'error'
    assert ('arm_demo', {'state': 'end'}) not in results
