"""Windows端rosbridge接口冒烟测试，不需要安装ROS 2。

默认只订阅状态并调用只读自检及错误返回验证；只有显式选择真实动作测试、
并在终端输入确认词后，才会请求解除限位、机械臂运动与停止服务。
"""

import argparse
import json
import time
import uuid

import websocket


SERVICE = '/sflare_os/game/command'
STATE_TOPIC = '/sflare_os/game/state'


class RosbridgeClient:
    """封装最少的rosbridge订阅、服务请求与响应关联逻辑。"""

    def __init__(self, url: str, timeout: float) -> None:
        """连接WebSocket；连接失败时由websocket库抛出异常。"""
        self.ws = websocket.create_connection(url, timeout=timeout)
        self.timeout = timeout
        self.state = None
        self.ws.send(json.dumps({
            'op': 'subscribe', 'topic': STATE_TOPIC,
            'type': 'sflare_game_interfaces/msg/GameState',
        }))

    def _receive(self, request_id: str, deadline: float) -> dict | None:
        """接收消息并缓存状态话题，直到对应服务响应或超时。"""
        while time.monotonic() < deadline:
            try:
                message = json.loads(self.ws.recv())
            except websocket.WebSocketTimeoutException:
                continue
            if message.get('op') == 'publish':
                self.state = message.get('msg')
            if message.get('id') == request_id:
                return message
        return None

    def call(self, request_type: str) -> dict:
        """调用统一游戏服务并解析其JSON字符串响应。"""
        request_id = uuid.uuid4().hex
        self.ws.send(json.dumps({
            'op': 'call_service',
            'service': SERVICE,
            'type': 'sflare_game_interfaces/srv/GameCommand',
            'args': {'request_type': request_type},
            'id': request_id,
        }))
        reply = self._receive(
            request_id, time.monotonic() + self.timeout)
        if reply is None:
            raise TimeoutError(f'{request_type}服务等待超时')
        if not reply.get('result'):
            raise RuntimeError(f'{request_type}调用失败：{reply}')
        raw_result = reply.get('values', {}).get('result', '{}')
        return json.loads(raw_result)

    def close(self) -> None:
        """关闭WebSocket连接，避免脚本退出后遗留网络会话。"""
        self.ws.close()


def main() -> int:
    """执行默认只读测试；真实动作必须由命令行选项和交互确认双重授权。"""
    parser = argparse.ArgumentParser(description='Sflare rosbridge接口测试')
    parser.add_argument(
        '--url', default='ws://172.168.102.48:9090',
        help='测试机rosbridge WebSocket地址')
    parser.add_argument(
        '--with-motion', action='store_true',
        help='额外测试解除限位、moveaction和stop（会影响真实机械臂）')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument(
        '--score', action='store_true',
        help='读取当前右臂姿态评分，需先完成本轮moveaction建立标准')
    args = parser.parse_args()

    client = RosbridgeClient(args.url, args.timeout)
    try:
        send_stop = False
        # 默认仅调用只读自检和未知请求，方便直接查看接口返回。
        for request_type in ('self_check', 'unsupported_test_request'):
            result = client.call(request_type)
            print(f'{request_type}: {json.dumps(result, ensure_ascii=False)}')
        print('state_topic:', json.dumps(client.state, ensure_ascii=False))

        if args.with_motion:
            confirmation = input(
                '将通过真实SDK调用release_limit和moveaction。'
                '确认机械臂周围安全后输入 MOVE：')
            if confirmation != 'MOVE':
                print('真实动作测试已取消。')
                return 0
            release = client.call('release_limit')
            print('release_limit:',
                  json.dumps(release, ensure_ascii=False))
            if release.get('success') == '1':
                move = client.call('moveaction')
                print('moveaction:', json.dumps(move, ensure_ascii=False))
                send_stop = True
            else:
                print('release_limit未成功，跳过moveaction和stop。')
        if args.score:
            result = client.call('calculate_score')
            print('calculate_score:', json.dumps(result, ensure_ascii=False))
        if send_stop:
            print('stop:', json.dumps(client.call('stop'), ensure_ascii=False))
    finally:
        client.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
