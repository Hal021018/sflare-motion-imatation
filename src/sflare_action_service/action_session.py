"""Original action-session request state. No ROS, SDK, or motion code."""
import re
import threading
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class ActionSession:
    """Only the action worker owns SDK calls; callbacks set request state."""

    command: str
    action: str
    scope: str = "dual"
    cancel_event: threading.Event = field(default_factory=threading.Event)
    intent: str = "none"
    zeroing: bool = False
    thread: Optional[threading.Thread] = None
    e_stop_interrupted: bool = False
    recover_after_cleanup: bool = False
    monitoring_connection: bool = False


# Same fault gate as the original priority-gozero path. Not a new recovery policy.
PRIORITY_ZERO_BLOCKING_CODES = frozenset({
    -5201, -135, -20038, -20039, -5031, -5032,
    -5176, -161, -20236, -20035, -5306,
})


def extract_error_codes(value):
    """Read structured SDK codes as well as textual diagnostic codes."""
    codes = set()
    code = getattr(value, "code", None)
    try:
        if code is not None:
            codes.add(int(code))
    except (TypeError, ValueError):
        pass
    get_id = getattr(value, "get_id", None)
    if callable(get_id):
        try:
            codes.add(int(get_id()))
        except Exception:
            pass
    # Negative decimal joint values in a preflight diagnostic are not SDK codes.
    codes.update(int(item) for item in re.findall(r"(?<![\d.])-\d{2,6}(?![\d.])", str(value)))
    return frozenset(codes)
