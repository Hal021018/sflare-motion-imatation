"""Sflare小游戏统一ROS 2状态与命令节点。

节点发布 `/sflare_os/game/state`，并通过 `/sflare_os/game/command` 同步返回命令
最终结果。耗时服务运行在多线程执行器中，状态定时器和定位话题订阅不会因机械臂
运动而停顿；硬件改变操作仍由互斥锁保证一次只执行一个。
"""

import math
import os
import random
import threading
import time
from collections import deque

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message

from sflare_game_interfaces.msg import GameState
from sflare_game_interfaces.srv import GameCommand

from .action_client import ActionServiceClient, ActionServiceError
from .controller import FakeGameController, PallasGameController
from .protocol import make_result
from .pose_scoring import JointStateInvalid, calculate_score, validate_scoring


class GameNode(Node):
    """实现统一游戏服务、状态发布和机械臂安全状态机。"""

    def __init__(self, enable_service=False) -> None:
        super().__init__('sflare_game_node')
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._operation_lock = threading.Lock()
        self._current_state = 'idle'
        self._emergency_stop_status = '-1'
        self._overtravel_status = '-1'
        self._last_monitored_message = 0.0
        self._recent_actions = deque(maxlen=2)
        self._reference_positions = None
        self._reference_action_index = None
        self._reference_action_name = None
        self._reference_generation = 0

        self._declare_parameters()
        self._scoring_arm = self.get_parameter('scoring_arm').value
        if self._scoring_arm not in ('right', 'left'):
            raise ValueError('scoring_arm必须为right或left')
        use_fake_sdk = self.get_parameter(
            'use_fake_sdk').get_parameter_value().bool_value
        if not use_fake_sdk and self._scoring_arm != 'right':
            raise ValueError('当前真机适配器仅支持右臂评分，不能将robot1标注为左臂')
        controller_ip = self.get_parameter(
            'controller_ip').get_parameter_value().string_value
        self.controller = (
            FakeGameController()
            if use_fake_sdk
            else PallasGameController(controller_ip)
        )

        # 动作服务：动作表 + 状态机客户端
        self._actions = self._load_actions()
        self._action_order = {name: index + 1
                              for index, name in enumerate(sorted(self._actions))}
        self._start_timeout = self.get_parameter(
            'start_timeout_sec').get_parameter_value().double_value
        self._timeout_margin = self.get_parameter(
            'timeout_margin_sec').get_parameter_value().double_value
        self._recover_timeout = self.get_parameter(
            'recover_timeout_sec').get_parameter_value().double_value
        self._auto_recover_limit = self.get_parameter(
            'auto_recover_limit').get_parameter_value().bool_value
        for label, value in (('start_timeout_sec', self._start_timeout),
                             ('timeout_margin_sec', self._timeout_margin),
                             ('recover_timeout_sec', self._recover_timeout)):
            if not 0 < value < float('inf'):
                raise ValueError(f'{label} 必须为正有限数')
        self.action_client = ActionServiceClient(
            self,
            self.get_parameter('action_topic').get_parameter_value().string_value,
            self.get_parameter(
                'action_status_topic').get_parameter_value().string_value,
            callback_group=self._callback_group,
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_publisher = self.create_publisher(
            GameState, '/sflare_os/game/state', state_qos)
        self.command_service = self.create_service(
            GameCommand,
            '/sflare_os/game/command',
            self.command_callback,
            callback_group=self._callback_group,
        ) if enable_service else None
        publish_hz = self.get_parameter(
            'state_publish_hz').get_parameter_value().double_value
        self.state_timer = self.create_timer(
            1.0 / publish_hz,
            self.publish_state,
            callback_group=self._callback_group,
        )
        self._create_monitored_subscription()
        self.get_logger().info('Sflare game core ready: /sflare_os/game/state')

    def _declare_parameters(self) -> None:
        """声明控制器、定位监控、动作服务与评分参数。"""
        self.declare_parameter('use_fake_sdk', True)
        self.declare_parameter('controller_ip', '192.168.3.100')
        self.declare_parameter('monitored_topic', '')
        self.declare_parameter('monitored_topic_type', '')
        self.declare_parameter('monitored_topic_timeout_sec', 1.0)
        self.declare_parameter('state_publish_hz', 5.0)
        # 动作服务
        self.declare_parameter('action_topic', '/dual_arm_xcdl_action')
        self.declare_parameter('action_status_topic', '/dual_arm_xcdl_action_status')
        self.declare_parameter('action_config', '')
        self.declare_parameter('start_timeout_sec', 5.0)
        self.declare_parameter('timeout_margin_sec', 10.0)
        self.declare_parameter('recover_timeout_sec', 90.0)
        self.declare_parameter('auto_recover_limit', True)
        # 评分：七轴角度直接来自只读自检（SDK）；本机无真实臂关节话题
        self.declare_parameter('scoring_arm', 'right')

    def _action_config_path(self) -> str:
        """返回动作表路径：参数 > 安装目录 > 源码目录（便于本地测试）。"""
        configured = self.get_parameter(
            'action_config').get_parameter_value().string_value
        if configured:
            return configured
        try:
            share = get_package_share_directory('sflare_game_controller')
            candidate = os.path.join(share, 'config', 'actions.yaml')
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
        # 未安装或旧安装缺文件时回退到包目录
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'actions.yaml')

    def _load_actions(self) -> dict:
        """加载动作表，校验每个动作的标准姿态必须是 7 个有限数值。"""
        path = self._action_config_path()
        with open(path, encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        raw = document.get('actions') if isinstance(document, dict) else None
        if not raw:
            raise ValueError(f'动作表为空或格式错误: {path}')
        actions = {}
        for name, item in raw.items():
            angles = [float(value) for value in item['standard_deg']]
            if len(angles) != 7 or not all(map(math.isfinite, angles)):
                raise ValueError(f'动作 {name} 的 standard_deg 必须是 7 个有限数值')
            actions[name] = {
                'command': str(item.get('command', name)),
                'display_name': str(item.get('display_name', name)),
                'standard_deg': angles,
                'hold_sec': float(item.get('hold_sec', 0.0)),
                'estimated_duration_sec': float(item['estimated_duration_sec']),
                'scoring': validate_scoring(item['scoring']),
            }
        return actions

    def _clear_reference(self) -> int:
        """开始新动作或停止时清空本轮标准，返回防止旧操作写回的代号。"""
        with self._state_lock:
            self._reference_generation += 1
            self._reference_positions = None
            self._reference_action_index = None
            self._reference_action_name = None
            return self._reference_generation

    def _create_monitored_subscription(self) -> None:
        """按参数动态订阅任意ROS消息，只记录数据到达时间。"""
        topic = self.get_parameter(
            'monitored_topic').get_parameter_value().string_value
        type_name = self.get_parameter(
            'monitored_topic_type').get_parameter_value().string_value
        self.monitored_subscription = None
        if not topic or not type_name:
            self.get_logger().warning(
                'monitored_topic未配置，positioning_status将保持0')
            return
        try:
            message_type = get_message(type_name)
            self.monitored_subscription = self.create_subscription(
                message_type,
                topic,
                self.monitored_message_callback,
                10,
                callback_group=self._callback_group,
            )
        except Exception as error:
            self._set_state('error')
            self.get_logger().error(
                f'创建监控话题订阅失败：{error}')

    def monitored_message_callback(self, _message) -> None:
        """记录最近一次定位类话题数据到达的单调时间。"""
        with self._state_lock:
            self._last_monitored_message = time.monotonic()

    def _positioning_status(self) -> str:
        """根据最后数据时间和1秒超时计算定位数据存活状态。"""
        timeout = self.get_parameter(
            'monitored_topic_timeout_sec').get_parameter_value().double_value
        with self._state_lock:
            last_message = self._last_monitored_message
        if last_message <= 0.0:
            return '0'
        return '1' if time.monotonic() - last_message <= timeout else '0'

    def _set_state(self, state: str) -> None:
        """线程安全地更新当前业务状态。"""
        with self._state_lock:
            self._current_state = state

    def publish_state(self) -> None:
        """周期发布四字段游戏状态，供rosbridge实时显示。"""
        with self._state_lock:
            emergency = self._emergency_stop_status
            overtravel = self._overtravel_status
            current = self._current_state
        message = GameState()
        message.positioning_status = self._positioning_status()
        message.emergency_stop_status = emergency
        message.overtravel_status = overtravel
        message.current_state = current
        self.state_publisher.publish(message)

    def _update_safety_cache(self, result: dict) -> None:
        """将一次成功自检结果写入状态话题缓存。"""
        with self._state_lock:
            self._emergency_stop_status = result['emergency_stop_status']
            self._overtravel_status = result['overtravel_status']

    def _self_check(self, reference=None) -> dict:
        """执行硬件自检，并附带定位数据与动作服务状态。"""
        self.action_client.ensure_ready()
        result = (self.controller.self_check() if reference is None
                  else self.controller.self_check(reference=reference))
        self._update_safety_cache(result)
        return {
            'positioning_status': self._positioning_status(),
            'action_service_status': self.action_client.status() or '-1',
            **result,
        }

    def command_callback(
        self,
        request: GameCommand.Request,
        response: GameCommand.Response,
    ) -> GameCommand.Response:
        """分派统一字符串请求；``moveaction:name`` 的冒号后为动作名。"""
        full = request.request_type.strip().lower()
        base, _, argument = full.partition(':')
        base = base.strip()
        argument = argument.strip()
        if base == 'stop':
            response.result = self._handle_stop(full)
            return response

        supported = {
            'self_check',
            'release_limit',
            'moveaction',
            'calculate_score',
        }
        if base not in supported:
            response.result = make_result(
                False, full, 'UNKNOWN_REQUEST', '不支持的request_type')
            return response

        if not self._operation_lock.acquire(blocking=False):
            response.result = make_result(
                False, full, 'ROBOT_BUSY', '已有硬件操作正在执行')
            return response

        try:
            response.result = self._dispatch(base, argument, full)
        except InterruptedError as error:
            self._set_state('idle')
            response.result = make_result(
                False, full, 'OPERATION_CANCELLED', str(error))
        except JointStateInvalid as error:
            self._set_state('error')
            response.result = make_result(
                False, full, 'JOINT_STATE_INVALID', str(error))
        except ActionServiceError as error:
            self._set_state('error')
            response.result = make_result(False, full, error.code, str(error))
        except Exception as error:
            self._set_state('error')
            self.get_logger().error(f'{full}执行失败：{error}')
            response.result = make_result(
                False, full, 'OPERATION_FAILED', str(error))
        finally:
            self._operation_lock.release()
        return response

    def _dispatch(self, base: str, argument: str, full: str) -> str:
        """调用对应业务方法，未知请求返回固定错误响应。"""
        if base == 'self_check':
            self._set_state('self_checking')
            result = self._self_check()
            self._set_state('idle')
            return make_result(True, full, 'OK', '自检完成', result)
        if base == 'release_limit':
            return self._handle_release_limit(full)
        if base == 'moveaction':
            return self._handle_moveaction(full, argument)
        if base == 'calculate_score':
            return self._handle_calculate_score(full)
        # 正常入口已过滤未知请求；保留兜底以防后续直接调用时误执行动作。
        return make_result(
            False, full, 'UNKNOWN_REQUEST', '不支持的request_type')

    def _handle_release_limit(self, request_type: str) -> str:
        """代理到动作服务的 ``recover_limit``；无超限时保持原 NOT_REQUIRED 语义。"""
        try:
            self.action_client.ensure_ready()
        except ActionServiceError as error:
            self._set_state('error')
            return make_result(False, request_type, error.code, str(error))

        self._set_state('self_checking')
        initial = self._self_check()
        if initial['emergency_stop_status'] != '0':
            self._set_state('error')
            return make_result(
                False, request_type, 'EMERGENCY_STOP',
                '急停状态异常，禁止解除限位', initial)
        if initial['overtravel_status'] == '0':
            self._set_state('idle')
            return make_result(
                True, request_type, 'NOT_REQUIRED',
                '当前没有超限轴，无需解除限位', initial)

        self._set_state('releasing_limit')
        try:
            self.action_client.recover_limits(self._recover_timeout)
        except ActionServiceError as error:
            self._set_state('error')
            return make_result(False, request_type, error.code, str(error))

        result = self._self_check()
        if result['overtravel_status'] != '0':
            self._set_state('error')
            return make_result(
                False, request_type, 'OVERTRAVEL',
                '解除限位后复检仍存在超限轴', result)
        self._set_state('idle')
        return make_result(
            True, request_type, 'OK', '解除限位并复检完成', result)

    def _handle_moveaction(self, request_type: str, argument: str = '') -> str:
        """调用动作服务播放指定/随机动作，并把动作表里的标准姿态记为本轮标准。

        - 服务必须处于 ready；运动中/故障态直接拒绝。
        - 自检发现超限且 ``auto_recover_limit`` 为真时，先发 ``recover_limit``，
          复检通过才继续；否则返回 OVERTRAVEL。
        - 标准姿态取 ``actions.yaml`` 的 ``standard_deg``（= 动作最后一个非零关键帧）。
        """
        generation = self._clear_reference()
        try:
            self.action_client.ensure_ready()
        except ActionServiceError as error:
            self._set_state('error')
            return make_result(False, request_type, error.code, str(error))

        self._set_state('self_checking')
        safety = self._self_check()
        if safety['emergency_stop_status'] != '0':
            self._set_state('error')
            return make_result(
                False, request_type, 'EMERGENCY_STOP',
                '急停状态异常，禁止运动', safety)
        if safety['overtravel_status'] != '0':
            if not self._auto_recover_limit:
                self._set_state('error')
                return make_result(
                    False, request_type, 'OVERTRAVEL',
                    '存在超限轴，请先解除限位', safety)
            self._set_state('releasing_limit')
            self.get_logger().info('检测到超限轴，先执行按需脱限')
            try:
                self.action_client.recover_limits(self._recover_timeout)
            except ActionServiceError as error:
                self._set_state('error')
                return make_result(
                    False, request_type, error.code,
                    f'自动解除限位失败：{error}', safety)
            safety = self._self_check()
            if safety['overtravel_status'] != '0':
                self._set_state('error')
                return make_result(
                    False, request_type, 'OVERTRAVEL',
                    '自动解除限位后仍存在超限轴', safety)

        name = argument.strip()
        if name:
            if name not in self._actions:
                self._set_state('idle')
                return make_result(
                    False, request_type, 'UNKNOWN_ACTION',
                    f'动作表中没有 {name}')
        else:
            candidates = [key for key in self._actions
                          if key not in self._recent_actions]
            name = random.choice(candidates or list(self._actions))
            self._recent_actions.append(name)

        spec = self._actions[name]
        duration = spec['estimated_duration_sec'] + self._timeout_margin
        with self._state_lock:
            if self._reference_generation != generation:
                raise InterruptedError('准备动作期间已收到停止，不再发送动作')
        self._set_state('moving')
        try:
            self.action_client.play(spec['command'], self._start_timeout, duration)
        except ActionServiceError as error:
            if error.code in {'ACTION_TIMEOUT', 'ACTION_NOT_STARTED'}:
                self.action_client.stop()
            self._set_state('error')
            return make_result(
                False, request_type, error.code, str(error), {'action_name': name})

        with self._state_lock:
            if self._reference_generation != generation:
                return make_result(
                    False, request_type, 'OPERATION_CANCELLED',
                    '本轮动作已被停止，标准姿态不再有效',
                    {'action_name': name})
            self._reference_positions = list(spec['standard_deg'])
            self._reference_action_index = self._action_order[name]
            self._reference_action_name = name
        self._set_state('idle')
        return make_result(
            True, request_type, 'OK', f'{spec["display_name"]} 完成', {
                'action_name': name,
                'action_command': spec['command'],
                'display_name': spec['display_name'],
                'action_index': self._action_order[name],
                'standard_deg': list(spec['standard_deg']),
                'reference_positions': list(spec['standard_deg']),
                'standard_source': 'yaml',
                'hold_sec': spec['hold_sec'],
                'estimated_duration_sec': spec['estimated_duration_sec'],
                'service_status': 'ready',
                'scoring_arm': self._scoring_arm,
                'scoring_version': 'spatial_joint_v2',
                'scoring_parameters': spec['scoring'],
            })

    def _handle_calculate_score(self, request_type: str) -> str:
        """对本轮实测标准与请求后的新七轴快照评分；不执行任何运动。"""
        with self._state_lock:
            reference = self._reference_positions
            action_index = self._reference_action_index
            action_name = self._reference_action_name
            generation = self._reference_generation
        if reference is None:
            return make_result(
                False, request_type, 'REFERENCE_NOT_READY',
                '本轮没有标准姿态，请先完成moveaction')
        self._set_state('self_checking')
        safety = self._self_check(reference=reference)
        if safety['emergency_stop_status'] != '0':
            self._set_state('error')
            return make_result(False, request_type, 'EMERGENCY_STOP',
                               '急停状态异常，不能评分', safety)
        if safety['overtravel_status'] != '0':
            self._set_state('error')
            return make_result(False, request_type, 'OVERTRAVEL',
                               '存在超限或未知限位状态，不能评分', safety)
        self._set_state('calculating_score')
        # 本机 /arm_joint_states 是占位全零，真实角度只能从只读自检取
        current = [float(value) for value in safety['positions']]
        result = calculate_score(
            reference, current, reference_xyz=safety['reference_xyz_mm'],
            current_xyz=safety['current_xyz_mm'], lower_limits=safety['lower_limits'],
            upper_limits=safety['upper_limits'], scoring=self._actions[action_name]['scoring'])
        result.update(action_name=action_name, cartesian_source=safety['cartesian_source'])
        with self._state_lock:
            cancelled = self._reference_generation != generation
        if cancelled:
            return make_result(
                False, request_type, 'OPERATION_CANCELLED',
                '评分期间本轮被停止，结果已丢弃')
        result.update(action_index=action_index, scoring_arm=self._scoring_arm)
        self._set_state('idle')
        return make_result(True, request_type, 'OK', '姿态评分完成', result)

    def _handle_stop(self, request_type: str) -> str:
        """向动作服务发停止请求；不等业务锁，立即清空本轮标准。"""
        self._clear_reference()
        self._set_state('stopping')
        try:
            self.action_client.stop()
            return make_result(
                True, request_type, 'OK', '停止请求已发送')
        except Exception as error:
            self._set_state('error')
            return make_result(
                False, request_type, 'STOP_FAILED', str(error))
        finally:
            with self._state_lock:
                failed = self._current_state == 'error'
            if not failed:
                self._set_state('idle')

    def destroy_node(self) -> None:
        """节点关闭前断开可能存在的只读 SDK 连接。"""
        try:
            self.controller.disconnect()
        except Exception as error:
            self.get_logger().warning(f'节点退出清理失败：{error}')
        super().destroy_node()


def main(args=None) -> None:
    """使用四线程执行器运行节点，保证长服务期间状态仍可调度。"""
    rclpy.init(args=args)
    node = GameNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
