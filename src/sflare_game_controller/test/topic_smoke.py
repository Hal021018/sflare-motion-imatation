"""Explicit ROS Topic smoke test; fake SDK and fake action service only, domain17."""
import json
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from interface_smoke_test import FakeActionService
from sflare_game_controller.imitate_node import ImitateNode


def main():
    rclpy.init()
    node, peer, action = ImitateNode(), Node('topic_smoke_client'), FakeActionService()
    assert node.command_service is None
    results = []
    peer.create_subscription(String, '/sflare_os/game/imitate/result',
                             lambda m: results.append(json.loads(m.data)), 50)
    command = peer.create_publisher(String, '/sflare_os/game/imitate/cmd', 20)
    events = peer.create_publisher(String, '/dual_arm_xcdl_action_events', 50)
    executor = MultiThreadedExecutor(num_threads=4)
    for item in (node, peer, action):
        executor.add_node(item)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()

    def wait(predicate):
        deadline = time.monotonic()+8
        while not predicate():
            if time.monotonic() >= deadline:
                raise AssertionError('Topic timeout: '+repr(results))
            time.sleep(.02)

    def send(cmd, args=None):
        message = String()
        message.data = json.dumps({'cmd': cmd, 'args': args or {}})
        command.publish(message)

    def event(kind, state):
        message = String()
        message.data = json.dumps({'action': 'like', 'type': kind, 'state': state})
        events.publish(message)

    try:
        wait(lambda: command.get_subscription_count() > 0 and node.action_client.status() == 'ready')
        time.sleep(.3)
        send('acts')
        wait(lambda: any(r['type'] == 'acts' for r in results))
        assert len(next(r['data']['list'] for r in results if r['type'] == 'acts')) == 6
        send('arm_check')
        wait(lambda: any(r['type'] == 'arm_check' for r in results))
        assert next(r['data'] for r in results if r['type'] == 'arm_check') == {'msg': ''}
        send('arm_demo', {'act_name': '点赞'})
        wait(lambda: node._active_demo == 'like')
        for phase in ('moving', 'still', 'reset'):
            event('phase', phase)
        event('power', 'on')
        event('power', 'off')
        wait(lambda: any(r == {'type': 'arm_demo', 'data': {'state': 'end'}} for r in results))
        phases = [r['data']['state'] for r in results if r['type'] == 'arm_demo']
        assert phases == ['start', 'moving', 'still', 'reset', 'end'], phases
        wait(lambda: len([r for r in results if r['type'] == 'arm_power']) == 2)
        # Explicit internal sample serialization test, NOT a B-key integration test.
        sample = json.loads(node._handle_calculate_score('calculate_score'))
        assert sample['success'] == '1', sample
        node.publish_sampled_score(sample['data'])
        wait(lambda: any(r['type'] == 'score' for r in results))
        score = next(r['data'] for r in results if r['type'] == 'score')
        assert score['j1'] == {'target': 103., 'actual': 0.}
        assert set(score) == {'score', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7'}
        send('right_init')
        wait(lambda: any(r['type'] == 'error' for r in results))
        print(json.dumps({'success': True, 'phases': phases, 'score_shape': score,
                          'b_key_connected': False, 'hardware_motion': False}), flush=True)
    finally:
        executor.shutdown()
        worker.join(timeout=3)
        for item in (node, peer, action):
            item.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
