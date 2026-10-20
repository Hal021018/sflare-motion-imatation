"""Original controller fault categories, without SDK calls or recovery motion."""
from action_session import extract_error_codes

PHYSICAL_ALARM_CODES = frozenset({-5201, -135})
E_STOP_DIAGNOSTIC_CODES = frozenset({-5029})
POWER_NOT_READY_CODES = frozenset({-5176, -161, -20236})
NETWORK_FATAL_CODES = frozenset({-20038, -20039})
LIMIT_CODES = frozenset({-5031, -5032})
INVALID_TARGET_CODES = frozenset({-5306})


def active_fault_codes(codes, e_stop_pressed):
    """The released physical E-stop makes retained -5029 historical."""
    return frozenset(codes) - (E_STOP_DIAGNOSTIC_CODES if e_stop_pressed is False else frozenset())


def failure_status(error, diagnostic_codes=(), e_stop_pressed=None):
    codes = active_fault_codes(extract_error_codes(error) | frozenset(diagnostic_codes), e_stop_pressed)
    if codes & (PHYSICAL_ALARM_CODES | E_STOP_DIAGNOSTIC_CODES):
        return "physical_alarm"
    if codes & POWER_NOT_READY_CODES:
        return "power_not_ready"
    # New SDK logs prove -20035 also denotes TCP connection failure.
    # Do not claim pendant ownership or launch an ownership probe from this code alone.
    if -20035 in codes:
        return "connection_failed"
    if codes & LIMIT_CODES:
        return "limit_recovery_failed"
    if codes & NETWORK_FATAL_CODES:
        return "fatal_network"
    if codes & INVALID_TARGET_CODES:
        return "invalid_target"
    return "failed"
