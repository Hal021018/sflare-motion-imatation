"""通过WebSocket发送一次moveaction并打印返回结果，依赖websocket-client。"""

import argparse
import json
import time

import websocket


def moveaction_once(url: str, timeout: float) -> dict:
    """连接url，发送一次请求并等待timeout秒，返回JSON结果或抛出异常。"""
    connection = websocket.create_connection(url, timeout=timeout)
    try:
        request_id = 'moveaction_once'
        connection.send(json.dumps({
            'op': 'call_service',
            'service': '/sflare_os/game/command',
            'type': 'sflare_game_interfaces/srv/GameCommand',
            'args': {'request_type': 'moveaction'},
            'id': request_id,
        }))
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('等待运动结果超时')
            connection.settimeout(remaining)
            reply = json.loads(connection.recv())
            # rosbridge可能同时发送状态通知，只处理本次服务请求的响应。
            if reply.get('id') != request_id:
                continue
            if reply.get('op') == 'status' and reply.get('level') == 'error':
                raise RuntimeError(str(reply))
            if reply.get('op') != 'service_response':
                continue
            if not reply.get('result'):
                raise RuntimeError(str(reply.get('values')))
            return json.loads(reply['values']['result'])
    finally:
        connection.close()


def main() -> int:
    """从命令行读取地址；成功返回0，拒绝或通信失败返回非零。"""
    parser = argparse.ArgumentParser(description='请求一次moveaction')
    parser.add_argument('url', help='例如 ws://机器人地址:9090')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('timeout必须大于0')
    try:
        result = moveaction_once(args.url, args.timeout)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get('success') == '1' else 1
    except Exception as error:
        print(f'请求失败：{error}')
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
