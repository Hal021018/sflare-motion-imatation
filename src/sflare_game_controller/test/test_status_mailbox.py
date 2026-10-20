from sflare_game_controller.action_protocol import StatusMailbox


def test_fast_transitions_are_not_overwritten():
    mailbox = StatusMailbox()
    mailbox.put('ready')
    mailbox.begin()
    mailbox.put('releasing_limit')
    mailbox.put('ready')
    assert mailbox.wait(lambda s: s != 'ready', 0) == 'releasing_limit'
    assert mailbox.wait(lambda s: s == 'ready', 0) == 'ready'


def test_previous_ready_is_not_completion():
    mailbox = StatusMailbox()
    mailbox.put('ready')
    mailbox.begin()
    assert mailbox.wait(lambda s: s == 'ready', 0) is None


def test_cached_state_expires():
    now = [0.]
    mailbox = StatusMailbox(clock=lambda: now[0])
    mailbox.put('ready')
    assert mailbox.status() == 'ready'
    now[0] = 4.
    assert mailbox.status() is None
