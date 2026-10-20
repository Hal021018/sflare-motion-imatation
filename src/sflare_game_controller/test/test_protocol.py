"""统一服务JSON协议测试。"""

import json

from sflare_game_controller.protocol import make_result


def test_make_result_keeps_fixed_envelope() -> None:
    """成功标志、请求类型和数据必须稳定地出现在响应中。"""
    result = json.loads(make_result(
        True,
        'self_check',
        'OK',
        '自检正常',
        {'emergency_stop_status': '0'},
    ))
    assert result == {
        'success': '1',
        'request_type': 'self_check',
        'code': 'OK',
        'message': '自检正常',
        'data': {'emergency_stop_status': '0'},
    }


def test_make_result_outputs_empty_data_object() -> None:
    """无业务数据时仍应输出对象，避免上位增加空值分支。"""
    result = json.loads(make_result(
        False, 'moveaction', 'ROBOT_BUSY', '机械臂忙'))
    assert result['success'] == '0'
    assert result['data'] == {}
