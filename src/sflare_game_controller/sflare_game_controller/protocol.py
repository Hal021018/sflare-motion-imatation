"""统一服务的 JSON 响应协议。

所有服务结果都经过本模块编码，确保 rosbridge 上位机收到稳定、可解析且包含明确
错误码的字符串。简单状态使用字符串，数值统计和数组保留 JSON 原生类型。
"""

import json
from typing import Any


def make_result(
    success: bool,
    request_type: str,
    code: str,
    message: str,
    data: dict[str, Any] | None = None,
) -> str:
    """生成固定结构的服务响应 JSON。

    参数:
        success: 请求是否完整执行成功。
        request_type: 已规范化的小写请求类型。
        code: 供上位状态机判断的稳定错误码。
        message: 面向日志和界面的中文说明。
        data: 不同请求的结构化结果；无数据时输出空对象。
    """
    payload = {
        'success': '1' if success else '0',
        'request_type': request_type,
        'code': code,
        'message': message,
        'data': data or {},
    }
    return json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
