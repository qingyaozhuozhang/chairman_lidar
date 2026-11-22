from concurrent.futures import Future
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

PATH = Path(__file__).parents[2] / 'src/function/framework/core/actions.py'
spec = importlib.util.spec_from_file_location('actions', PATH)
actions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(actions)


def done(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True

    def __init__(self):
        self.cancelled = False
        self.result = Future()

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancelled = True
        if not self.result.done():
            self.result.set_result('cancelled')
        return done('accepted')


def test_active_action_settles_before_next_task(monkeypatch):
    monkeypatch.setattr(actions.rclpy, 'ok', lambda: True)
    handle = Handle()
    owner = SimpleNamespace(cancel_current_task=False)
    tracker = actions.TrackedActionClient(owner, SimpleNamespace(send_goal_async=lambda _: done(handle)))
    tracker.send_goal_async(None)
    tracker.settle()
    assert handle.cancelled
    assert not tracker.pending


def test_late_acceptance_cancelled_and_blocks_until_settled(monkeypatch):
    monkeypatch.setattr(actions.rclpy, 'ok', lambda: True)
    acceptance = Future()
    tracker = actions.TrackedActionClient(SimpleNamespace(cancel_current_task=False), SimpleNamespace(send_goal_async=lambda _: acceptance), timeout=0.02)
    tracker.send_goal_async(None)
    with pytest.raises(TimeoutError):
        tracker.settle()
    assert tracker.pending
    handle = Handle()
    acceptance.set_result(handle)
    assert handle.cancelled
    tracker.settle()
    assert not tracker.pending


def test_missing_action_server_wait_can_be_cancelled(monkeypatch):
    monkeypatch.setattr(actions.rclpy, 'ok', lambda: True)
    owner = SimpleNamespace(cancel_current_task=False)

    def unavailable(timeout_sec):
        owner.cancel_current_task = True
        return False

    tracker = actions.TrackedActionClient(owner, SimpleNamespace(wait_for_server=unavailable))
    with pytest.raises(RuntimeError, match='取消'):
        tracker.wait_for_server()
