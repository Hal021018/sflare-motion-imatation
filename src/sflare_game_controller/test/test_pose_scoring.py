"""七轴名称解析、弧度换算和评分文档分段边界的纯Python测试。"""

import math
from types import SimpleNamespace

import pytest

from sflare_game_controller.pose_scoring import (
    JOINT_SUFFIXES,
    JointStateInvalid,
    calculate_score,
    read_joint_angles,
)


def make_message(right, left=None):
    """构造左右臂14轴消息，使用与实际话题相同的名称和弧度单位。"""
    if left is None:
        left = [90.0] * 7
    names = [f'{arm}_arm_{suffix}' for arm in ('left', 'right')
             for suffix in JOINT_SUFFIXES]
    return SimpleNamespace(
        name=names,
        position=[math.radians(value) for value in left + list(right)],
    )


def test_read_right_arm_by_name() -> None:
    """打乱14轴顺序后，仍只读取右臂并恢复固定七轴顺序。"""
    angles = [0, -5, 15, 30, 90, -45, 180]
    message = make_message(angles)
    message.name.reverse()
    message.position.reverse()
    assert read_joint_angles(message) == angles
    assert read_joint_angles(message, 'left') == [90.0] * 7


@pytest.mark.parametrize('fault', ['missing', 'duplicate', 'length', 'nan'])
def test_invalid_joint_message(fault) -> None:
    """缺轴、重名、数组不匹配和非有限值不能生成七轴姿态。"""
    message = make_message([0.0] * 7)
    if fault == 'missing':
        message.name[-1] = 'unrelated_joint'
    elif fault == 'duplicate':
        message.name[-1] = message.name[-2]
    elif fault == 'length':
        message.position.pop()
    else:
        message.position[-1] = float('nan')
    with pytest.raises(JointStateInvalid):
        read_joint_angles(message)


CONFIG = dict(position_share=.7, position_tolerance_mm=50., position_zero_mm=300.,
              joint_tolerance_deg=5., joint_weights=[24, 24, 18, 18, 10, 3, 3])


def score(reference=None, current=None, **kwargs):
    options = dict(reference_xyz=[0.]*3, current_xyz=[0.]*3,
                   lower_limits=[-180.]*7, upper_limits=[180.]*7, scoring=CONFIG)
    options.update(kwargs)
    return calculate_score(reference or [0.]*7, current or [0.]*7, **options)


@pytest.mark.parametrize('distance,expected', [(0, 100), (50, 100), (175, 65), (300, 30), (1000, 30)])
def test_position_distance(distance, expected):
    assert score(current_xyz=[distance, 0, 0])['score'] == expected


def test_directional_limit_normalization():
    reference = [100.]+[0.]*6
    positive = score(reference, [130.]+[0.]*6)
    negative = score(reference, [70.]+[0.]*6)
    assert positive['normalized_joint_errors'][0] > negative['normalized_joint_errors'][0]
    assert positive['raw_score'] < negative['raw_score']


def test_same_xyz_different_configuration():
    assert score(current=[45.]+[0.]*6)['raw_score'] < score()['raw_score']


def test_action_specific_weight():
    config = dict(CONFIG, joint_weights=[90, 1, 1, 1, 1, 1, 5])
    assert score(current=[30.]+[0.]*6, scoring=config)['raw_score'] < score(current=[30.]+[0.]*6)['raw_score']


@pytest.mark.parametrize('angle', [-180, -30.000001, -30, -5, 0, 5, 30, 30.000001, 180])
def test_score_bounded_and_negative_penalties(angle):
    result = score(current=[angle]*7, current_xyz=[300, 0, 0])
    assert 0 <= result['score'] <= 100
    assert result['total_penalty'] <= 0
    assert all(p <= 0 for p in result['joint_penalties'])
    assert result['raw_score'] == pytest.approx(100 + result['total_penalty'])


def test_missing_cartesian_rejected():
    with pytest.raises(ValueError):
        score(current_xyz=[float('nan'), 0, 0])


def test_no_thirty_degree_cliff():
    assert abs(score(current=[30.]*7)['raw_score']-score(current=[30.000001]*7)['raw_score']) < .001


def test_all_penalties_saturate_at_zero():
    assert score(current=[180.]*7, current_xyz=[1000., 0., 0.])['score'] == 0


def test_invalid_limits_rejected():
    with pytest.raises(ValueError):
        score(lower_limits=[10.]*7, upper_limits=[0.]*7)


@pytest.mark.parametrize('weights', [[0.]*7, [-1.]*7, [float('nan')]*7])
def test_invalid_weights_rejected(weights):
    with pytest.raises(ValueError):
        score(scoring=dict(CONFIG, joint_weights=weights))


@pytest.mark.parametrize('invalid', [
    [], [0] * 6, [0] * 8, [float('nan')] * 7, [float('inf')] * 7,
    ['bad'] * 7, [True] * 7, None,
])
def test_invalid_score_input(invalid) -> None:
    """输入不足七轴或包含无效角度时明确失败，不输出伪分数。"""
    with pytest.raises(ValueError):
        calculate_score(invalid, [0.0] * 7, reference_xyz=[0]*3, current_xyz=[0]*3,
                        lower_limits=[-180]*7, upper_limits=[180]*7, scoring=CONFIG)
    with pytest.raises(ValueError):
        calculate_score([0.0] * 7, invalid, reference_xyz=[0]*3, current_xyz=[0]*3,
                        lower_limits=[-180]*7, upper_limits=[180]*7, scoring=CONFIG)
