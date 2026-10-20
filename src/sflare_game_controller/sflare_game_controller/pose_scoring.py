"""纯函数评分：SDK同坐标系XYZ为主，动作专属、方向归一化关节罚分为辅。

保留JointState解析工具，但当前真机评分不使用占位全零话题。
本模块不执行运动、ROS或SDK调用。
"""

import math


JOINT_SUFFIXES = (
    'J1_joint', 'J2_joint', 'J3_joint', 'J4_joint', 'J5_joint',
    'J6_wrist_1_joint', 'J6_wrist_2_joint',
)


class JointStateInvalid(ValueError):
    """关节消息缺轴、名称重复、数组不匹配或角度无效。"""


class JointStateTimeout(TimeoutError):
    """等待本次请求之后的新关节消息超时。"""


def read_joint_angles(message, arm: str = 'right') -> list[float]:
    """方法1：从JointState按名称读取七轴，返回角度制数组。

    message需包含等长的name与position，position单位为弧度；arm仅允许
    right或left。顺序固定为J1到J5、J6_wrist_1、J6_wrist_2，与消息排列
    无关。数据异常抛出JointStateInvalid，不返回缺轴或上次缓存的角度。
    """
    if arm not in ('right', 'left'):
        raise JointStateInvalid('arm必须为right或left')
    try:
        names = list(message.name)
        positions = list(message.position)
        if len(names) != len(positions):
            raise JointStateInvalid('name与position数量不一致')
        degrees = []
        for suffix in JOINT_SUFFIXES:
            name = f'{arm}_arm_{suffix}'
            if names.count(name) != 1:
                raise JointStateInvalid(f'关节{name}缺失或重复')
            radians = float(positions[names.index(name)])
            if not math.isfinite(radians):
                raise JointStateInvalid(f'关节{name}角度不是有限数值')
            # 消除弧度换算的二进制尾差，避免理论15度被解析为15.000000000002。
            angle = math.degrees(radians)
            if not math.isfinite(angle):
                raise JointStateInvalid(f'关节{name}角度换算溢出')
            degrees.append(round(angle, 9))
        return degrees
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        if isinstance(error, JointStateInvalid):
            raise
        raise JointStateInvalid(f'关节消息格式错误：{error}') from error


def finite_vector(values, count, label):
    try:
        if len(values) != count or any(isinstance(v, bool) for v in values):
            raise ValueError(label + '维度或类型错误')
        result = [float(v) for v in values]
        if not all(map(math.isfinite, result)):
            raise ValueError(label + '包含NaN/Inf')
        return result
    except (TypeError, OverflowError) as error:
        raise ValueError(label + '必须为有限数值数组') from error


def validate_scoring(config):
    """所有动作显式配置评分参数；无缺省降级到旧公式。"""
    result = dict(config)
    weights = finite_vector(result['joint_weights'], 7, '关节权重')
    if min(weights) < 0 or not math.isfinite(sum(weights)) or sum(weights) <= 0:
        raise ValueError('权重必须非负且总和大于0')
    result['joint_weights'] = [w / sum(weights) for w in weights]
    for key in ('position_share', 'position_tolerance_mm', 'position_zero_mm',
                'joint_tolerance_deg'):
        value = result[key]
        if isinstance(value, bool) or not math.isfinite(float(value)):
            raise ValueError(key + '必须是有限数值')
        result[key] = float(value)
    if not 0 < result['position_share'] < 1:
        raise ValueError('position_share必须在0和1之间')
    if not 0 <= result['position_tolerance_mm'] < result['position_zero_mm']:
        raise ValueError('位置距离阈值顺序错误')
    if result['joint_tolerance_deg'] < 0:
        raise ValueError('关节容差不能为负')
    return result


def calculate_score(reference, current, *, reference_xyz, current_xyz,
                    lower_limits, upper_limits, scoring) -> dict:
    """v2: 负罚分归一化。XYZ为同一SDK坐标系的毫米坐标，角度为度。

    最终分=100+位置负罚分+七轴负罚分，限制至[0,100]，最后统一向下取整。
    无有效XYZ或限位就拒绝评分，不伪造空间分或降级为纯角度评分。
    """
    reference = finite_vector(reference, 7, '标准角度')
    current = finite_vector(current, 7, '当前角度')
    target = finite_vector(reference_xyz, 3, '标准XYZ')
    actual = finite_vector(current_xyz, 3, '当前XYZ')
    lower = finite_vector(lower_limits, 7, '下限')
    upper = finite_vector(upper_limits, 7, '上限')
    config = validate_scoring(scoring)
    weights = config['joint_weights']
    tolerance = config['joint_tolerance_deg']
    signed = [q-r for q, r in zip(current, reference)]
    ratios = []
    for r, q, lo, hi, delta in zip(reference, current, lower, upper, signed):
        if not lo < hi or not lo <= r <= hi or not lo <= q <= hi:
            raise ValueError('关节或标准姿态超出有效限位')
        span = hi-r if delta >= 0 else r-lo
        # A direction narrower than the configured deadband has no tolerated travel.
        deadband = min(tolerance, span)
        excess = max(0., abs(delta)-deadband)
        ratios.append(min(1., excess/(span-deadband)) if span > deadband else 0.)
    distance = math.dist(target, actual)
    if not math.isfinite(distance):
        raise ValueError('空间距离计算溢出')
    spatial_error = min(1., max(0., (distance-config['position_tolerance_mm']) /
                                      (config['position_zero_mm']-config['position_tolerance_mm'])))
    position_max = 100.*config['position_share']
    joint_max = 100.-position_max
    position_penalty = -position_max*spatial_error
    penalties = [-joint_max*w*e for w, e in zip(weights, ratios)]
    raw = max(0., min(100., 100.+position_penalty+sum(penalties)))
    return {
        'scoring_version': 'spatial_joint_v2',
        'score': int(math.floor(round(raw, 10))), 'raw_score': raw,
        'reference_positions': reference, 'current_positions': current,
        'reference_xyz_mm': target, 'current_xyz_mm': actual,
        'position_error_mm': distance,
        'target_region_reached': distance <= config['position_tolerance_mm'],
        'position_score': position_max+position_penalty,
        'joint_score': joint_max+sum(penalties),
        'position_penalty': position_penalty, 'joint_penalties': penalties,
        'total_penalty': position_penalty+sum(penalties),
        'errors': list(map(abs, signed)), 'signed_errors': signed,
        'normalized_joint_errors': ratios, 'joint_weights': weights,
        'joint_scores': [joint_max*w+p for w, p in zip(weights, penalties)],
        'scoring_parameters': config,
    }
