"""Explicit supervised real-motion acceptance. Run only when the robot area is safe.

Does not change SDK, controller configuration, action files, or ROS parameters.
Stops the sequence on the first error and verifies power-off between actions.
"""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('actions', nargs='+', choices=[
        'like', 'handshake', 'handover', 'diagonal_point', 'raise_head', 'doubt'])
    args = parser.parse_args()
    import rclpy
    import PallasSDK as sdk
    from std_msgs.msg import String, Int32
    from rclpy.qos import QoSProfile, DurabilityPolicy
    rclpy.init()
    node = rclpy.create_node('old_sdk_topic_acceptance')
    events, states, estops, summary = [], [], [], []
    def receive(message):
        data = json.loads(message.data)
        events.append(data)
        print('EVENT', round(time.monotonic(), 3), json.dumps(data, ensure_ascii=False), flush=True)
    node.create_subscription(String, '/sflare_os/game/imitate/result', receive, 50)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/dual_arm_xcdl_action_status', lambda m: states.append(m.data), qos)
    node.create_subscription(Int32, '/e_stop', lambda m: estops.append(m.data), qos)
    publisher = node.create_publisher(String, '/sflare_os/game/imitate/cmd', 20)
    stop = node.create_publisher(String, '/dual_arm_xcdl_action', 10)

    def hardware_check():
        controller = sdk.Controller()
        assert controller.connect('192.168.3.100', 2094) == 0
        controller.set_data_unit(1)
        robot = controller.add_robot(1)
        try:
            version = controller.get_sdk_version()
            assert 'PallasSDK_V2.0.5355;' in version, version
            status, power = controller.get_status(), robot.get_state()
            joints, errors = list(robot.get_joint_pos())[:7], controller.check_error()
            snapshot = dict(sdk=version, status=status, power=power, joints=joints, errors=errors)
            print('HARDWARE', json.dumps(snapshot), flush=True)
            assert status[0] == 0 and power[0] == 0 and not errors, snapshot
            assert len(joints) == 7 and all(map(math.isfinite, joints))
            return snapshot
        finally:
            controller.disconnect()

    try:
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert publisher.get_subscription_count() == 1
        hardware_check()
        for name in args.actions:
            assert states and states[-1] == 'ready' and estops and estops[-1] == 0
            events.clear()
            started = time.monotonic()
            print('BEGIN', name, flush=True)
            message = String()
            message.data = json.dumps({'cmd': 'arm_demo', 'args': {'act_name': name}})
            publisher.publish(message)
            deadline = time.monotonic()+70
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if estops[-1] != 0:
                    raise RuntimeError('Physical emergency stop')
                if any(e['type'] == 'error' for e in events):
                    raise RuntimeError('Action error: '+repr(events))
                if any(e == {'type': 'arm_demo', 'data': {'state': 'end'}} for e in events):
                    break
            else:
                raise RuntimeError('Action timeout: '+name)
            phases = [e['data']['state'] for e in events if e['type'] == 'arm_demo']
            power_events = [e['data']['state'] for e in events if e['type'] == 'arm_power']
            assert all(p in phases for p in ['start', 'moving', 'still', 'reset', 'end']), phases
            assert power_events == ['on', 'off'], power_events
            hardware = hardware_check()
            result = dict(action=name, elapsed_sec=round(time.monotonic()-started, 3),
                          phases=phases, power_events=power_events, hardware_after=hardware)
            summary.append(result)
            print('PASS', json.dumps(result), flush=True)
            deadline = time.monotonic()+1
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
        print('FINAL_SUMMARY', json.dumps(summary), flush=True)
    except Exception:
        message = String()
        message.data = 'stop'
        stop.publish(message)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.1)
        print('STOP_SENT; PARTIAL_SUMMARY', json.dumps(summary), flush=True)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
