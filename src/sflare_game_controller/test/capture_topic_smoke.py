"""Explicit domain17 B-key-only fake-driver test; never moves hardware."""
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
    game, peer, action = ImitateNode(), Node('capture_fake_driver'), FakeActionService()
    events = peer.create_publisher(String, '/sflare_os/game/imitate/teleop_event', 20)
    commands = peer.create_publisher(String, '/sflare_os/game/imitate/cmd', 20)
    controls, results = [], []
    def publish(publisher, data):
        message = String(); message.data = json.dumps(data); publisher.publish(message)
    def control_callback(message):
        data = json.loads(message.data); controls.append(data)
        if data['command'] in {'pause_for_sample', 'reset_after_sample'}:
            # Fake driver echoes the exact round/session/request ticket, as required.
            event = dict(round_id=data['round_id'], session_id=data['session_id'],
                         request_id=data['request_id'])
            if data['command'] == 'pause_for_sample':
                event['event'] = 'paused'
            else:
                event.update(event='reset_done', success=True, power_confirmed='off')
            publish(events, event)
    peer.create_subscription(String, '/sflare_os/game/imitate/teleop_control', control_callback, 20)
    peer.create_subscription(String, '/sflare_os/game/imitate/result',
                             lambda m: results.append(json.loads(m.data)), 50)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (game, peer, action):executor.add_node(node)
    worker = threading.Thread(target=executor.spin, daemon=True);worker.start()
    def wait(predicate):
        deadline = time.monotonic()+10
        while not predicate():
            if time.monotonic() >= deadline:raise AssertionError((controls, results))
            time.sleep(.01)
    try:
        wait(lambda: commands.get_subscription_count() > 0 and game.action_client.status() == 'ready')
        time.sleep(.3)
        for index, reason in enumerate(['button', 'button'], 1):
            publish(commands, {'cmd': 'arm_demo', 'args': {'act_name': 'like'}})
            wait(lambda: sum(r == {'type': 'arm_demo', 'data': {'state': 'end'}} for r in results) == index)
            # A demonstration ending is NOT a teleop start.
            assert len([c for c in controls if c['command'] == 'pause_for_sample']) == index-1
            event = dict(round_id=game._round_id, session_id='physical-session-'+str(index))
            publish(events, dict(event, event='started'))
            wait(lambda: game._capture.state == 'running')
            for _ in range(100):game._capture_timer()
            assert len([c for c in controls if c['command'] == 'pause_for_sample']) == index-1
            publish(events, dict(event, event='button'))
            wait(lambda: game._capture.state == 'done')
            wait(lambda: len([r for r in results if r['type'] == 'score']) == index)
            publishes = [c for c in controls if c['command'] == 'reset_after_sample']
            assert len(publishes) == index and publishes[-1]['reason'] == reason
            # Late B cannot overwrite the captured result or request another reset.
            publish(events, dict(event, event='button'))
            time.sleep(.1)
            assert len([c for c in controls if c['command'] == 'reset_after_sample']) == index
        print(json.dumps({'success': True, 'rounds': 2, 'reasons': ['button', 'button'],
                          'scores': 2, 'reset_permissions': 2, 'hardware_motion': False,
                          'automatic_capture': False}), flush=True)
    finally:
        executor.shutdown();worker.join(timeout=3)
        for node in (game, peer, action):node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':main()
