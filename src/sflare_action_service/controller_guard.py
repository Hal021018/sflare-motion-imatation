"""Controller availability probing, never power-on or robot motion."""
import socket
import time
from types import SimpleNamespace


class CheckedSDK:
    """Recovery SDK setters must not silently ignore nonzero return codes."""
    READS = frozenset({'get_joint_pos', 'get_joint_speed', 'get_limit', 'check_error'})

    def __init__(self, target):
        self.target = target

    def __getattr__(self, name):
        method = getattr(self.target, name)
        if name in self.READS:
            return method
        def checked(*args, **kwargs):
            result = method(*args, **kwargs)
            if result != 0:
                raise RuntimeError(f'{name} 返回失败: {result!r}')
            return result
        return checked


def recovery_arm(label, controller, robot, diagnostic):
    return SimpleNamespace(label=label, controller=CheckedSDK(controller),
                           robot=CheckedSDK(robot), diagnostic=diagnostic)


def tcp_reachable(ip, port):
    try:
        with socket.create_connection((ip, port), timeout=1.0):
            return True
    except OSError:
        return False


def probe_available(controller_factory, targets, port, reachable=tcp_reachable):
    """Return (state, safe_cleanup). Never touch a rejected SDK session.

    -20035 is ambiguous in the deployed SDK. A separate bounded TCP check
    distinguishes network unavailability from a *possible* external owner.
    Only a successful SDK connect establishes availability.
    """
    from action_session import extract_error_codes
    for label, ip in targets:
        if not reachable(ip, port):
            return 'connection_failed', True
        controller = controller_factory()
        connected = False
        result = 'ready'
        try:
            code = controller.connect(ip, port)
            if code != 0:
                raise RuntimeError(f'{label} connect: {code!r}')
            connected = True
        except Exception as exc:
            codes = extract_error_codes(exc)
            result = 'handle_busy' if -20035 in codes else 'connection_failed'
        finally:
            if connected:
                off = False
                for attempt in range(3):
                    try:
                        if controller.set_power_enable(False, 1, timeout=3000) == 0:
                            off = True
                            break
                    except Exception:
                        pass
                    if attempt < 2:
                        time.sleep(0.5)
                if not off:
                    return 'unsafe_stop', False
                try:
                    if controller.disconnect() != 0:
                        return 'unsafe_stop', False
                except Exception:
                    return 'unsafe_stop', False
        if result != 'ready':
            return result, True
    return 'ready', True
