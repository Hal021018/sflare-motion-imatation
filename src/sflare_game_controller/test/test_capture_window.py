import threading
import pytest
from sflare_game_controller.capture_window import CaptureWindow


def make():
    now = [100.]
    return CaptureWindow(), now


def test_no_timeout_trigger_exists():
    window, now = make()
    now[0] = 500.
    window.start('actual-start-1', 4)
    with pytest.raises(ValueError):window.claim('timeout')
    assert window.state == 'running'
    assert not hasattr(window, 'deadline')


def test_button_samples_only_once():
    window, now = make()
    window.start('a', 1)
    now[0] += 12
    ticket = window.claim('button', 'a')
    assert ticket.reason == 'button'
    now[0] += 50
    assert window.claim('button', 'a') is None


def test_concurrent_buttons_only_one_claim():
    window, now = make()
    window.start('a', 1)
    now[0] += 30
    results = []
    barrier = threading.Barrier(20)
    def run(reason):
        barrier.wait()
        results.append(window.claim(reason, 'a'))
    workers = [threading.Thread(target=run, args=('button',))
               for i in range(20)]
    for worker in workers: worker.start()
    for worker in workers: worker.join()
    assert sum(item is not None for item in results) == 1


def test_pause_ack_required_and_duplicates_ignored():
    window, now = make()
    window.start('a', 1)
    ticket = window.claim('button', 'a')
    assert not window.complete(ticket, {'score': 99})
    assert window.begin_sample('a', 'stale') is None
    assert window.begin_sample('a', ticket.request_id) == ticket
    assert window.begin_sample('a', ticket.request_id) is None
    result = {'angles': [1, 2, 3]}
    assert window.complete(ticket, result)
    result['angles'][0] = 999
    assert window.result['angles'][0] == 1
    assert not window.complete(ticket, result)
    assert window.done('a', ticket.request_id)
    assert not window.start('a', 1)


def test_new_round_or_estop_invalidates_late_sample():
    window, _ = make()
    window.start('old', 1)
    old = window.claim('button', 'old')
    window.begin_sample('old', old.request_id)
    window.cancel()
    assert not window.complete(old, {'score': 100})
    window.start('new', 2)
    assert window.claim('button', 'old') is None
    assert window.begin_sample('old', old.request_id) is None


def test_duplicate_start_does_not_restart_session():
    window, now = make()
    window.start('a', 1)
    now[0] += 20
    assert not window.start('a', 1)
    assert window.state == 'running'


def test_button_before_start_ignored():
    window, _ = make()
    assert window.claim('button', 'a') is None


def test_overlapping_session_rejected():
    window, _ = make()
    window.start('a', 1)
    with pytest.raises(RuntimeError):window.start('b', 2)
