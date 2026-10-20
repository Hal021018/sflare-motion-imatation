"""在已运行的假SDK游戏节点上验证公共ROS话题与服务接口。

本脚本由开发者显式运行，不会自动发送硬件运动命令。
测试节点通过专用String话题发布定位存活消息；被测节点必须配置
use_fake_sdk=true。脚本自带一个假动作服务，用于应答 moveaction /
release_limit，不需要真实动作服务进程。
"""

import json
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from sflare_game_interfaces.msg import GameState
from sflare_game_interfaces.srv import GameCommand
from sflare_game_controller.pose_scoring import calculate_score


SERVICE_NAME = '/sflare_os/game/command'
STATE_TOPIC = '/sflare_os/game/state'
PRESENCE_TOPIC = '/sflare_os/interface_test_presence'
ACTION_TOPIC = '/dual_arm_xcdl_action'
ACTION_STATUS_TOPIC = '/dual_arm_xcdl_action_status'

# 与 config/actions.yaml 中 6 个动作的 command 对应
ACTION_COMMANDS = {
    'diagonal_point', 'handshake', 'handover', 'like', 'raise_head', 'doubt',
}


class FakeActionService(Node):
    """最小动作服务替身：busy → ready，support recover_limit。"""

    def __init__(self) -> None:
        super().__init__('fake_action_service')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, ACTION_STATUS_TOPIC, qos)
        self.create_subscription(String, ACTION_TOPIC, self._on_command, 10)
        self.received = []
        self.status = 'ready'
        self._finish_timer = None
        self._publish_status()
        self.create_timer(0.2, self._publish_status)

    def _message(self, text: str) -> String:
        message = String()
        message.data = text
        return message

    def _publish_status(self) -> None:
        self._publisher.publish(self._message(self.status))

    def _set_status(self, status: str) -> None:
        self.status = status
        self._publish_status()

    def _on_command(self, message: String) -> None:
        command = message.data.strip()
        self.received.append(command)
        if command == 'stop':
            self._set_status('ready')
            return
        if command == 'recover_limit':
            self._set_status('releasing_limit')
            self._schedule_finish('ready', 0.5)
            return
        self._set_status('busy')
        self._schedule_finish('ready', 1.0)

    def _schedule_finish(self, status: str, delay: float) -> None:
        if self._finish_timer is not None:
            self._finish_timer.cancel()
            self.destroy_timer(self._finish_timer)
        self._finish_timer = self.create_timer(delay, lambda: self._finish(status))

    def _finish(self, status: str) -> None:
        # 只取消，不在回调内销毁自身定时器（rclpy 会 abort）
        if self._finish_timer is not None:
            self._finish_timer.cancel()
            self._finish_timer = None
        self._set_status(status)


class InterfaceSmokeTest(Node):
    """发布测试定位消息、订阅状态并调用统一游戏服务。"""

    def __init__(self) -> None:
        """初始化测试节点与所有ROS通信端点。"""
        super().__init__('sflare_game_interface_smoke_test')
        self._state_message = None
        self._presence_publisher = self.create_publisher(
            String, PRESENCE_TOPIC, 10)
        self._state_subscription = self.create_subscription(
            GameState, STATE_TOPIC, self._state_callback, 10)
        self._service_client = self.create_client(
            GameCommand, SERVICE_NAME)
        self._presence_timer = self.create_timer(
            0.2, self._publish_presence)

    def _publish_presence(self) -> None:
        """发布测试定位存活数据。

        用于验证定位状态从有效变为过期，运动接口不依赖该状态。
        """
        message = String()
        message.data = 'interface-smoke-test'
        self._presence_publisher.publish(message)

    def _state_callback(self, message: GameState) -> None:
        """缓存最新状态话题消息，供状态接口断言使用。"""
        self._state_message = message

    def spin_until(
        self, predicate, timeout_sec: float, description: str,
    ) -> None:
        """调度ROS回调直至条件满足或超时。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f'等待超时：{description}')
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def call(self, request_type: str) -> dict:
        """调用统一命令服务并解析JSON响应。

        服务无响应或响应为空时抛出异常，避免将超时误报为通过。
        """
        request = GameCommand.Request()
        request.request_type = request_type
        future = self._service_client.call_async(request)
        self.spin_until(
            lambda: future.done(), 60.0,
            f'{SERVICE_NAME} request_type={request_type}',
        )
        response = future.result()
        if response is None:
            raise RuntimeError(f'{request_type}服务调用没有响应对象')
        return json.loads(response.result)


def expect(condition: bool, message: str) -> None:
    """检查接口返回是否符合预期。"""
    if not condition:
        raise AssertionError(message)


def main() -> int:
    """依次验证状态话题、现有服务请求及已移除命令的错误返回。"""
    rclpy.init()
    fake_service = FakeActionService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(fake_service)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    test_node = InterfaceSmokeTest()
    results = []
    try:
        test_node.spin_until(
            lambda: test_node._service_client.wait_for_service(
                timeout_sec=0.0),
            15.0,
            f'服务可用：{SERVICE_NAME}',
        )
        test_node.spin_until(
            lambda: (test_node._state_message is not None
                     and test_node._state_message.positioning_status == '1'),
            10.0,
            f'状态消息及定位有效：{STATE_TOPIC}',
        )
        state = test_node._state_message
        expect(state.positioning_status == '1',
               f'定位状态应为1，实际为{state.positioning_status}')
        expect(state.current_state != '', 'current_state不应为空')
        results.append(('state_topic', {
            'positioning_status': state.positioning_status,
            'emergency_stop_status': state.emergency_stop_status,
            'overtravel_status': state.overtravel_status,
            'current_state': state.current_state,
        }))

        response = test_node.call('self_check')
        expect(response['success'] == '1', f'self_check失败：{response}')
        expect(response['code'] == 'OK', f'self_check code异常：{response}')
        for field in (
                'emergency_stop_status', 'overtravel_status', 'positions',
                'lower_limits', 'upper_limits', 'action_service_status'):
            expect(field in response['data'],
                   f'self_check缺少字段{field}：{response}')
        expect(response['data']['action_service_status'] == 'ready',
               f'动作服务状态应为ready：{response}')
        results.append(('self_check', response))

        response = test_node.call('release_limit')
        expect(response['success'] == '1',
               f'release_limit失败：{response}')
        expect(response['code'] == 'NOT_REQUIRED',
               f'假SDK正常限位应返回NOT_REQUIRED：{response}')
        results.append(('release_limit', response))

        response = test_node.call('calculate_score')
        expect(response['code'] == 'REFERENCE_NOT_READY',
               f'未建立标准时应拒绝评分：{response}')
        results.append(('score_without_reference', response))

        # 停止定位数据并等待状态过期，验证无定位消息时仍允许关节运动。
        test_node._presence_timer.cancel()
        test_node.spin_until(
            lambda: test_node._state_message.positioning_status == '0',
            10.0,
            '定位数据过期，状态变为0',
        )
        results.append(('positioning_offline', {'positioning_status': '0'}))

        response = test_node.call('moveaction')
        expect(response['success'] == '1', f'moveaction失败：{response}')
        expect(response['code'] == 'OK', f'moveaction code异常：{response}')
        name = response['data']['action_name']
        expect(name in ACTION_COMMANDS, f'动作名不在动作表：{response}')
        standard = response['data']['standard_deg']
        expect(len(standard) == 7, f'标准姿态维度异常：{response}')
        expect(response['data']['reference_positions'] == standard,
               f'标准应与动作表一致：{response}')
        expect(fake_service.received[-1] == name,
               f'动作服务未收到动作名：{fake_service.received}')
        results.append(('moveaction', response))

        # 假控制器七轴恒为 0，评分应与“标准 vs 0”的离线计算一致。
        expected = calculate_score(
            standard, [0.0] * 7, reference_xyz=[0.]*3, current_xyz=[0.]*3,
            lower_limits=[-180.]*7, upper_limits=[180.]*7,
            scoring=response['data']['scoring_parameters'])['score']
        response = test_node.call('calculate_score')
        expect(response['success'] == '1', f'评分失败：{response}')
        expect(response['data']['score'] == expected,
               f'实机评分应为{expected}：{response}')
        results.append(('calculate_score', response))

        response = test_node.call('stop')
        expect(response['success'] == '1', f'stop失败：{response}')
        expect(response['code'] == 'OK', f'stop code异常：{response}')
        # stop 是异步发布，等假动作服务真正收到再断言
        test_node.spin_until(
            lambda: bool(fake_service.received)
            and fake_service.received[-1] == 'stop',
            5.0, '动作服务收到 stop')
        expect(fake_service.received[-1] == 'stop',
               f'动作服务未收到stop：{fake_service.received}')
        results.append(('stop', response))

        response = test_node.call('calculate_score')
        expect(response['code'] == 'REFERENCE_NOT_READY',
               f'停止后标准应失效：{response}')
        results.append(('score_after_stop', response))

        response = test_node.call('moveaction:no_such_action')
        expect(response['success'] == '0',
               f'未知动作应失败：{response}')
        expect(response['code'] == 'UNKNOWN_ACTION',
               f'未知动作错误码异常：{response}')
        results.append(('unknown_action', response))

        response = test_node.call('unsupported_test_request')
        expect(response['success'] == '0',
               f'未知请求应失败：{response}')
        expect(response['code'] == 'UNKNOWN_REQUEST',
               f'未知请求错误码异常：{response}')
        results.append(('unknown_request', response))

        # 旧客户端发来已删除命令时，只验证拒绝结果，不存在采样操作。
        response = test_node.call('start_scoring')
        expect(response['success'] == '0',
               f'已移除的start_scoring请求应失败：{response}')
        expect(response['code'] == 'UNKNOWN_REQUEST',
               f'start_scoring应返回UNKNOWN_REQUEST：{response}')
        results.append(('removed_start_scoring', response))

        print(json.dumps({
            'success': True,
            'results': results,
        }, ensure_ascii=False, indent=2))
        return 0
    except Exception as error:
        print(json.dumps({
            'success': False,
            'error': str(error),
            'results': results,
        }, ensure_ascii=False, indent=2), file=sys.stderr)
        return 1
    finally:
        # 先停执行器再销毁节点，避免回调线程访问已销毁对象
        executor.shutdown()
        thread.join(timeout=2.0)
        test_node.destroy_node()
        fake_service.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
