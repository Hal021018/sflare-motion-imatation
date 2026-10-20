"""One physical teleoperation session -> one B-key pre-reset sample.

No SDK, ROS, timers or hardware calls. The driver must confirm paused before sampling.
"""
import copy
from dataclasses import dataclass
import threading
import uuid


@dataclass(frozen=True)
class CaptureTicket:
    session_id: str
    request_id: str
    generation: int
    reason: str


class CaptureWindow:
    def __init__(self):
        self.lock = threading.RLock()
        self.session_id = None
        self.state = 'idle'
        self.generation = None
        self.ticket = None
        self.result = None
        self.seen_sessions = set()

    def start(self, session_id, generation):
        if not isinstance(session_id, str) or not session_id.strip():
            raise ValueError('必须提供实际遥操会话唯一标识')
        with self.lock:
            if session_id in self.seen_sessions:
                return False
            if self.state not in {'idle', 'done', 'cancelled', 'failed'}:
                raise RuntimeError('上一遥操会话尚未结束')
            self.seen_sessions.add(session_id)
            self.session_id, self.generation = session_id, generation
            self.state, self.ticket, self.result = 'running', None, None
            return True

    def claim(self, reason, session_id=None):
        if reason != 'button':
            raise ValueError('无效采样触发')
        with self.lock:
            if self.state != 'running':
                return None
            if session_id is not None and session_id != self.session_id:
                return None
            self.ticket = CaptureTicket(self.session_id, uuid.uuid4().hex,
                                        self.generation, reason)
            self.state = 'awaiting_pause'
            return self.ticket

    def begin_sample(self, session_id, request_id):
        with self.lock:
            if (self.state != 'awaiting_pause' or self.ticket is None
                    or session_id != self.ticket.session_id
                    or request_id != self.ticket.request_id):
                return None
            self.state = 'sampling'
            return self.ticket

    def complete(self, ticket, result):
        with self.lock:
            if self.state != 'sampling' or ticket != self.ticket:
                return False
            self.result = copy.deepcopy(result)
            self.state = 'reset_pending'
            return True

    def done(self, session_id, request_id):
        with self.lock:
            if (self.state != 'reset_pending' or self.ticket is None
                    or session_id != self.ticket.session_id
                    or request_id != self.ticket.request_id):
                return False
            self.state = 'done'
            return True

    def cancel(self, failed=False):
        with self.lock:
            self.state = 'failed' if failed else 'cancelled'

    def matches(self, ticket, state):
        with self.lock:
            return self.ticket == ticket and self.state == state
