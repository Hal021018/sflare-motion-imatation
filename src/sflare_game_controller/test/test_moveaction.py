"""moveaction / release_limit / stop 的节点级测试（假 SDK + 假动作服务）。

不连接真实机械臂，也不依赖动作服务进程。
"""

import json

import pytest


class FakeActionClient:
    """记录命令并按需失败/回调的假动作服务客户端。"""

    def __init__(self):
        self.status_value = 'ready'
        self.commands = []
        self.play_error = None
        self.recover_error = None
        self.on_play = None
        self.on_recover = None

    def status(self):
        return self.status_value

    def ensure_ready(self):
        from sflare_game_controller.action_client import (
            ActionServiceError, error_code_for)
        if self.status_value == 'ready':
            return 'ready'
        if self.status_value in ('busy', 'releasing_limit'):
            code = 'ROBOT_BUSY'
        else:
            code = error_code_for(self.status_value) or 'SERVICE_UNAVAILABLE'
        raise ActionServiceError(code, f'status={self.status_value}')

    def play(self, name, start_timeout, total_timeout):
        self.commands.append(name)
        if self.on_play is not None:
            self.on_play()
        if self.play_error is not None:
            from sflare_game_controller.action_client import ActionServiceError
            raise ActionServiceError(*self.play_error)
        return 'ready'

    def recover_limits(self, timeout, start_timeout=5.0):
        self.commands.append('recover_limit')
        if self.on_recover is not None:
            self.on_recover()
        if self.recover_error is not None:
            from sflare_game_controller.action_client import ActionServiceError
            raise ActionServiceError(*self.recover_error)
        return 'ready'

    def stop(self):
        self.commands.append('stop')


@pytest.fixture
def game_node():
    """创建假 SDK 游戏节点，并把动作客户端替换为假实现。"""
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('sflare_game_interfaces.srv')
    from sflare_game_controller.game_node import GameNode
    from sflare_game_controller.controller import FakeGameController

    rclpy.init()
    node = GameNode()
    try:
        assert isinstance(node.controller, FakeGameController)
        node.action_client = FakeActionClient()
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def call(node, request_type, argument=''):
    return json.loads(node._handle_moveaction(request_type, argument))


# ---------------------------------------------------------------- moveaction
def test_random_action_uses_yaml_standard(game_node) -> None:
    """随机动作成功后，标准姿态来自 actions.yaml 且已写入本轮标准。"""
    result = call(game_node, 'moveaction')
    assert result['success'] == '1', result
    name = result['data']['action_name']
    spec = game_node._actions[name]
    assert result['data']['standard_deg'] == spec['standard_deg']
    assert result['data']['standard_source'] == 'yaml'
    assert result['data']['action_command'] == spec['command']
    assert game_node._reference_positions == spec['standard_deg']
    assert game_node.action_client.commands == [spec['command']]


def test_explicit_action_name(game_node) -> None:
    """moveaction:like 只播放指定动作。"""
    result = call(game_node, 'moveaction:like', 'like')
    assert result['success'] == '1', result
    assert result['data']['action_name'] == 'like'
    assert game_node.action_client.commands == ['like']


def test_unknown_action_name_rejected(game_node) -> None:
    result = call(game_node, 'moveaction:nope', 'nope')
    assert result['code'] == 'UNKNOWN_ACTION'
    assert game_node.action_client.commands == []


def test_recent_actions_do_not_repeat(game_node) -> None:
    """连续两次随机动作不应重复。"""
    first = call(game_node, 'moveaction')['data']['action_name']
    second = call(game_node, 'moveaction')['data']['action_name']
    assert first != second


def test_busy_service_rejected(game_node) -> None:
    game_node.action_client.status_value = 'busy'
    result = call(game_node, 'moveaction')
    assert result['code'] == 'ROBOT_BUSY'
    assert game_node.action_client.commands == []


def test_alarm_service_rejected(game_node) -> None:
    game_node.action_client.status_value = 'physical_alarm'
    result = call(game_node, 'moveaction')
    assert result['code'] == 'EMERGENCY_STOP'


def test_missing_status_rejected(game_node) -> None:
    game_node.action_client.status_value = None
    result = call(game_node, 'moveaction')
    assert result['code'] == 'SERVICE_UNAVAILABLE'


def test_emergency_stop_blocks_motion(game_node) -> None:
    game_node.controller.emergency_stop = True
    result = call(game_node, 'moveaction')
    assert result['code'] == 'EMERGENCY_STOP'
    assert game_node.action_client.commands == []


def test_auto_recover_then_play(game_node) -> None:
    """自检发现超限 → 先发 recover_limit，复检通过后继续播放。"""
    game_node.controller.positions[0] = 999.0

    def fix():
        game_node.controller.positions[0] = 0.0

    game_node.action_client.on_recover = fix
    result = call(game_node, 'moveaction')
    assert result['success'] == '1', result
    assert game_node.action_client.commands[0] == 'recover_limit'
    assert len(game_node.action_client.commands) == 2


def test_auto_recover_failure_blocks(game_node) -> None:
    game_node.controller.positions[0] = 999.0
    game_node.action_client.recover_error = ('OVERTRAVEL', 'still over')
    result = call(game_node, 'moveaction')
    assert result['code'] == 'OVERTRAVEL'
    assert game_node.action_client.commands == ['recover_limit']


def test_auto_recover_disabled(game_node) -> None:
    game_node._auto_recover_limit = False
    game_node.controller.positions[0] = 999.0
    result = call(game_node, 'moveaction')
    assert result['code'] == 'OVERTRAVEL'
    assert game_node.action_client.commands == []


def test_play_failure_maps_code(game_node) -> None:
    game_node.action_client.play_error = ('ACTION_TIMEOUT', 'too slow')
    result = call(game_node, 'moveaction')
    assert result['code'] == 'ACTION_TIMEOUT'
    assert game_node._reference_positions is None


def test_stop_during_play_cancels_round(game_node) -> None:
    """动作执行期间收到 stop → 本轮标准作废。"""
    game_node.action_client.on_play = game_node._clear_reference
    result = call(game_node, 'moveaction')
    assert result['code'] == 'OPERATION_CANCELLED'
    assert game_node._reference_positions is None


# ------------------------------------------------------------- release_limit
def test_release_limit_not_required(game_node) -> None:
    result = json.loads(game_node._handle_release_limit('release_limit'))
    assert result['code'] == 'NOT_REQUIRED'
    assert game_node.action_client.commands == []


def test_release_limit_proxies_to_service(game_node) -> None:
    game_node.controller.positions[0] = 999.0
    game_node.action_client.on_recover = (
        lambda: game_node.controller.positions.__setitem__(0, 0.0))
    result = json.loads(game_node._handle_release_limit('release_limit'))
    assert result['code'] == 'OK', result
    assert game_node.action_client.commands == ['recover_limit']


def test_release_limit_failure(game_node) -> None:
    game_node.controller.positions[0] = 999.0
    game_node.action_client.recover_error = ('OVERTRAVEL', 'no luck')
    result = json.loads(game_node._handle_release_limit('release_limit'))
    assert result['code'] == 'OVERTRAVEL'


def test_release_limit_busy_service(game_node) -> None:
    game_node.action_client.status_value = 'busy'
    result = json.loads(game_node._handle_release_limit('release_limit'))
    assert result['code'] == 'ROBOT_BUSY'


# --------------------------------------------------------------- stop / score
def test_stop_clears_reference_and_publishes(game_node) -> None:
    call(game_node, 'moveaction:like', 'like')
    assert game_node._reference_positions is not None
    result = json.loads(game_node._handle_stop('stop'))
    assert result['success'] == '1'
    assert game_node.action_client.commands[-1] == 'stop'
    assert game_node._reference_positions is None


def test_score_requires_reference(game_node) -> None:
    result = json.loads(game_node._handle_calculate_score('calculate_score'))
    assert result['code'] == 'REFERENCE_NOT_READY'


def test_score_uses_standard_from_yaml(game_node) -> None:
    call(game_node, 'moveaction:like', 'like')
    standard = list(game_node._actions['like']['standard_deg'])
    game_node.controller.positions = [value + 3.0 for value in standard]
    result = json.loads(game_node._handle_calculate_score('calculate_score'))
    assert result['success'] == '1'
    assert result['data']['score'] == 100
    assert result['data']['reference_positions'] == standard


def test_self_check_reports_service_status(game_node) -> None:
    result = game_node._self_check()
    assert result['action_service_status'] == 'ready'
