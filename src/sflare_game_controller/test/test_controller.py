"""只读自检适配器与假控制器的契约测试。"""

import pytest
from types import SimpleNamespace

from sflare_game_controller.controller import (
    FakeGameController,
    PallasGameController,
    SafetyStateError,
)


def test_emergency_stop_mapping() -> None:
    """状态数组第0项只接受协议定义的0和1。"""
    assert PallasGameController._parse_emergency_stop([0, 8]) is False
    assert PallasGameController._parse_emergency_stop([1, 8]) is True
    with pytest.raises(SafetyStateError):
        PallasGameController._parse_emergency_stop([])
    with pytest.raises(SafetyStateError):
        PallasGameController._parse_emergency_stop([2])


def test_limit_order_is_normalised() -> None:
    """正负限位无论SDK以何种顺序返回都得到一致边界。"""
    assert PallasGameController._normalise_limit([90, -80]) == (-80, 90)
    assert PallasGameController._normalise_limit([-80, 90]) == (-80, 90)


def test_real_controller_has_no_motion_api() -> None:
    """游戏进程不再直接驱动机械臂：运动/Jog 接口必须不存在。"""
    for name in ('movej', 'release_limits', 'abort'):
        assert not hasattr(PallasGameController, name), name


def test_fake_self_check_clean() -> None:
    controller = FakeGameController()
    result = controller.self_check()
    assert result['emergency_stop_status'] == '0'
    assert result['overtravel_status'] == '0'
    assert result['overtravel_axes'] == []
    assert result['positions'] == [0.0] * 7


def test_fake_self_check_detects_overtravel() -> None:
    """超出上下限的轴必须被逐轴报告。"""
    controller = FakeGameController(axis_count=3)
    controller.positions = [-200.0, 0.0, 210.0]
    result = controller.self_check()
    assert result['overtravel_status'] == '1'
    assert result['overtravel_axes'] == [1, 3]


def test_fake_self_check_detects_emergency_stop() -> None:
    controller = FakeGameController()
    controller.emergency_stop = True
    assert controller.self_check()['emergency_stop_status'] == '1'


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_real_feedback_must_be_finite(bad):
    controller = PallasGameController('unused')
    controller.ctrl = SimpleNamespace(get_status=lambda: [0])
    controller.robot = SimpleNamespace(get_axis_number=lambda: 7,
                                      get_joint_pos=lambda: [bad] + [0.] * 6)
    with pytest.raises(SafetyStateError):
        controller._collect_safety_state()
