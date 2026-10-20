"""Topic-only public interface. B-key service hook is intentionally not fabricated."""
import json
import threading
import time
import uuid
from types import SimpleNamespace

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32
from rclpy.qos import QoSProfile, DurabilityPolicy

from .game_node import GameNode
from .imitate_protocol import decode_request, score_payload
from .capture_window import CaptureWindow


class ImitateNode(GameNode):
    def __init__(self):
        super().__init__(enable_service=False)
        self._topic_gate = threading.Lock()
        self._topic_worker = None
        self._active_demo = None
        self._last_phase = None
        self._closing = False
        self._capture = CaptureWindow()
        self._capture_flow_lock = threading.RLock()
        self._capture_worker = None
        self._round_id = None
        self._capture_wait_deadline = None
        self.teleop_control = self.create_publisher(
            String, '/sflare_os/game/imitate/teleop_control', 20)
        self.result_publisher = self.create_publisher(String, '/sflare_os/game/imitate/result', 50)
        self.create_subscription(String, '/sflare_os/game/imitate/cmd', self._on_topic_command,
                                 20, callback_group=self._callback_group)
        self.create_subscription(String, '/dual_arm_xcdl_action_events', self._on_action_event,
                                 50, callback_group=self._callback_group)
        self.create_subscription(String, '/sflare_os/game/imitate/teleop_event',
                                 self._on_teleop_event, 20, callback_group=self._callback_group)
        self.create_subscription(Int32, '/e_stop', self._on_capture_estop,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                 callback_group=self._callback_group)
        self.create_timer(.05, self._capture_timer, callback_group=self._callback_group)
        self.get_logger().info('Topic接口就绪；仅B键触发采样，等待底层真实遥操事件')

    def _control(self, command, ticket=None, **extra):
        data = {'command': command, 'round_id': self._round_id, **extra}
        if ticket is not None:
            data.update(session_id=ticket.session_id, request_id=ticket.request_id,
                        reason=ticket.reason)
        message = String()
        message.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        self.teleop_control.publish(message)

    def _on_capture_estop(self, message):
        if message.data != 1:
            return
        with self._capture_flow_lock:
            if self._round_id is not None:
                self._clear_reference()
                self._fail_capture('实体急停，本轮计时、评分与自动归位许可失效')

    def _on_teleop_event(self, message):
        with self._capture_flow_lock:
            self._handle_teleop_event(message)

    def _handle_teleop_event(self, message):
        try:
            event = json.loads(message.data)
            if self._closing or self._round_id is None or event.get('round_id') != self._round_id:
                return
            kind, session = event.get('event'), event.get('session_id')
            if kind == 'started':
                with self._state_lock:
                    if self._reference_positions is None:
                        raise ValueError('没有有效的本轮动作标准')
                    generation = self._reference_generation
                if self._capture.start(session, generation):
                    self._set_state('teleoperating')
            elif kind == 'button':
                self._request_capture('button', session)
            elif kind == 'paused':
                # Must mean the driver stopped sending teleop commands, not just accepted pause.
                ticket = self._capture.begin_sample(session, event.get('request_id'))
                if ticket is not None:
                    self._capture_wait_deadline = time.monotonic()+15.
                    self._capture_worker = threading.Thread(target=self._sample_teleop,
                                                            args=(ticket,), daemon=True)
                    self._capture_worker.start()
            elif kind == 'reset_done':
                ticket = self._capture.ticket
                if ticket is None or (session, event.get('request_id')) != (
                        ticket.session_id, ticket.request_id):
                    return
                if event.get('success') is not True:
                    self._fail_capture('原驱动归位失败')
                elif self._capture.done(session, event.get('request_id')):
                    self._set_state('idle')
                    self._capture_wait_deadline = None
                    if event.get('power_confirmed') == 'off':
                        self._result('arm_power', {'state': 'off'})
            elif kind in {'cancelled', 'emergency_stop'}:
                if session == self._capture.session_id:
                    self._fail_capture('遥操取消或急停，本轮不再自动归位')
        except (ValueError, TypeError, KeyError, RuntimeError) as exc:
            self._error('teleop', 'TELEOP_EVENT_INVALID', str(exc))

    def _request_capture(self, reason, session=None):
        with self._capture_flow_lock:
            ticket = self._capture.claim(reason, session)
            if ticket is not None:
                self._capture_wait_deadline = time.monotonic()+5.
                self._control('pause_for_sample', ticket)

    def _capture_timer(self):
        with self._capture_flow_lock:
            if self._closing:
                return
            deadline = self._capture_wait_deadline
            if deadline is not None and time.monotonic() >= deadline:
                self._fail_capture('暂停、采样或归位确认超时；禁止继续自动归位')

    def _fail_capture(self, message, ticket=None):
        with self._capture_flow_lock:
            if ticket is not None and self._capture.ticket != ticket:
                return
            self._capture.cancel(failed=True)
            self._capture_wait_deadline = None
            self._control('stop_without_reset', self._capture.ticket)
            self._set_state('error')
            self._error('teleop', 'CAPTURE_FAILED', message)

    def _sample_teleop(self, ticket):
        if not self._topic_gate.acquire(blocking=False):
            self._fail_capture('采样时游戏节点忙', ticket)
            return
        try:
            result = self._invoke('calculate_score')
            with self._capture_flow_lock:
                with self._state_lock:
                    valid = ticket.generation == self._reference_generation
                if not valid or not self._capture.matches(ticket, 'sampling'):
                    return
                if result['success'] != '1':
                    self._fail_capture('采样评分失败: '+result['message'], ticket)
                    return
                if not self._capture.complete(ticket, result['data']):
                    return
                # Sample is immutable before any reset permission is published.
                self.publish_sampled_score(result['data'])
                self._capture_wait_deadline = time.monotonic()+45.
                self._control('reset_after_sample', ticket, sampled=True, allow_reset=True)
                self._set_state('resetting')
        except Exception as exc:
            self._fail_capture('采样异常: '+str(exc), ticket)
        finally:
            self._topic_gate.release()

    def _result(self, kind, data):
        message = String()
        message.data = json.dumps({'type': kind, 'data': data}, ensure_ascii=False, allow_nan=False)
        self.result_publisher.publish(message)

    def _error(self, command, code, message):
        self._result('error', {'cmd': command, 'code': code, 'msg': message})

    def _on_topic_command(self, message):
        try:
            command, arguments = decode_request(message.data)
        except (ValueError, TypeError) as exc:
            self._error('', 'INVALID_REQUEST', str(exc))
            return
        if self._closing:
            return
        if command == 'arm_demo' and self._capture.state in {
                'running', 'awaiting_pause', 'sampling', 'reset_pending'}:
            self._error(command, 'ROBOT_BUSY', '遥操或采样归位尚未结束')
            return
        if command == 'acts':
            self._result('acts', {'list': [a['display_name'] for a in self._actions.values()]})
            return
        if not self._topic_gate.acquire(blocking=False):
            self._error(command, 'ROBOT_BUSY', '已有操作，命令不排队')
            return
        self._topic_worker = threading.Thread(target=self._execute_topic,
                                              args=(command, arguments), daemon=True)
        self._topic_worker.start()

    def _invoke(self, request_type):
        # Reuse tested business functions, not a ROS service call/server.
        response = self.command_callback(SimpleNamespace(request_type=request_type),
                                         SimpleNamespace(result=''))
        return json.loads(response.result)

    def _execute_topic(self, command, arguments):
        try:
            if command == 'arm_check':
                result = self._invoke('self_check')
                data = result['data']
                msg = ''
                if result['success'] != '1':
                    msg = result['message']
                elif data['emergency_stop_status'] != '0':
                    msg = '急停未释放'
                elif data['overtravel_status'] != '0':
                    msg = '关节超限'
                self._result('arm_check', {'msg': msg})
                return
            requested = arguments['act_name'].strip()
            matches = [name for name, spec in self._actions.items()
                       if requested in {name, spec['display_name']}]
            if len(matches) != 1:
                self._error(command, 'UNKNOWN_ACTION', 'act_name不在实际动作列表中')
                return
            self._active_demo = matches[0]
            self._capture.cancel()
            self._round_id = None
            self._last_phase = 'start'
            self._result('arm_demo', {'state': 'start'})
            result = self._invoke('moveaction:'+matches[0])
            if result['success'] == '1':
                self._round_id = uuid.uuid4().hex
                self._control('arm_round', capture_trigger='button')
                self._result('arm_demo', {'state': 'end'})
            else:
                self._error(command, result['code'], result['message'])
        except Exception as exc:
            self._error(command, 'OPERATION_FAILED', str(exc))
        finally:
            self._active_demo = None
            self._last_phase = None
            self._topic_gate.release()

    def _on_action_event(self, message):
        try:
            event = json.loads(message.data)
            if event.get('type') == 'power' and event.get('state') in {'on', 'off'}:
                self._result('arm_power', {'state': event['state']})
            elif (self._active_demo is not None and event.get('action') == self._active_demo
                  and event.get('type') == 'phase'
                  and event.get('state') in {'moving', 'still', 'reset'}):
                phase = event['state']
                if phase != self._last_phase:
                    self._last_phase = phase
                    self._result('arm_demo', {'state': phase})
        except (ValueError, TypeError, AttributeError) as exc:
            self.get_logger().warning('忽略非法动作事件: '+str(exc))

    def publish_sampled_score(self, result):
        """Publish the captured angles, never the post-reset position."""
        self._result('score', score_payload(result))

    def destroy_node(self):
        self._closing = True
        self._capture.cancel()
        self._capture_wait_deadline = None
        self._control('stop_without_reset', self._capture.ticket)
        capture_worker = self._capture_worker
        if capture_worker is not None and capture_worker.is_alive():
            capture_worker.join(timeout=15.)
            if capture_worker.is_alive():
                raise RuntimeError('采样线程尚未退出，禁止销毁节点')
        worker = self._topic_worker
        if worker is not None and worker.is_alive():
            self.action_client.stop()
            worker.join(timeout=10.)
            if worker.is_alive():
                raise RuntimeError('操作线程尚未退出，禁止销毁ROS节点')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImitateNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
