"""Public Topic protocol; no ROS or hardware operations."""
import json
import math


def decode_request(raw):
    request = json.loads(raw)
    if not isinstance(request, dict) or not isinstance(request.get('cmd'), str):
        raise ValueError('请求必须包含字符串cmd与对象args')
    command = request['cmd'].strip()
    arguments = request.get('args', {})
    if not isinstance(arguments, dict):
        raise ValueError('args必须为对象')
    if command not in {'acts', 'arm_check', 'arm_demo'}:
        raise ValueError('不支持的cmd: '+command)
    if command == 'arm_demo':
        if not isinstance(arguments.get('act_name'), str) or not arguments['act_name'].strip():
            raise ValueError('arm_demo需要非空act_name')
    return command, arguments


def score_payload(result):
    """Degrees, J1..J7; never substitute command values for measured angles."""
    target, actual = result['reference_positions'], result['current_positions']
    if len(target) != 7 or len(actual) != 7:
        raise ValueError('评分必须包含7轴目标和实际角度')
    if not all(math.isfinite(float(v)) for v in list(target)+list(actual)):
        raise ValueError('评分角度无效')
    data = {'score': result['score']}
    data.update({'j'+str(i+1): {'target': float(t), 'actual': float(a)}
                 for i, (t, a) in enumerate(zip(target, actual))})
    return data
