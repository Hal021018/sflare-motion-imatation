"""动作服务的 rclpy 客户端：只提供 publish / wait_for / status 三个原语。

所有状态判定在 :mod:`action_protocol` 里，本模块不包含业务逻辑。

状态话题必须用 ``TRANSIENT_LOCAL`` 订阅，否则刚连上时拿不到服务端的最后一条状态。
"""

from __future__ import annotations

from typing import Callable, Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .action_protocol import (  # noqa: F401  对外重新导出，调用方只依赖本模块
    BUSY,
    LIMIT_RECOVERY_FAILED,
    READY,
    RELEASING_LIMIT,
    STATUS_ERROR_CODES,
    ActionSequencer,
    ActionServiceError,
    error_code_for,
    StatusMailbox,
)


class ActionServiceClient(ActionSequencer):
    """rclpy 实现：状态用 TRANSIENT_LOCAL 订阅，命令可靠发布。"""

    def __init__(self, node: Node, command_topic: str, status_topic: str,
                 callback_group=None) -> None:
        self._node = node
        self._mailbox = StatusMailbox()
        self._status_topic = status_topic

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscription = node.create_subscription(
            String, status_topic, self._on_status, status_qos,
            callback_group=callback_group)
        self._publisher = node.create_publisher(String, command_topic, 10)
        super().__init__(self._publish_impl, self._wait_for_impl, self._status_impl)

    # ---- 注入给状态机的原语 ---------------------------------------------
    def _on_status(self, message: String) -> None:
        value = message.data.strip().lower()
        previous = self._mailbox.status()
        self._mailbox.put(value)
        if value != previous:
            self._node.get_logger().info(f'动作服务状态: {value}')

    def _status_impl(self) -> Optional[str]:
        if self._node.count_publishers(self._status_topic) == 0:
            return None
        return self._mailbox.status()

    def _publish_impl(self, command: str) -> None:
        message = String()
        message.data = command
        self._mailbox.begin()
        self._publisher.publish(message)

    def _wait_for_impl(
        self, predicate: Callable[[str], bool], timeout: float,
    ) -> Optional[str]:
        """依次消费本次发布后的状态，保留瞬时 busy→ready。"""
        return self._mailbox.wait(predicate, timeout)
