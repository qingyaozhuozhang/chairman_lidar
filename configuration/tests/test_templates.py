import importlib.util
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).parents[2] / 'docs/templates'


def load_template(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / name / 'task.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_route_template_stops_at_first_failed_point():
    visited = []

    def go_to_point(point, mode=None):
        visited.append((point, mode))
        return point != 7

    ctx = SimpleNamespace(config={'points': [1, 7, 8], 'mode': 4}, cancelled=False, go_to_point=go_to_point)
    assert load_template('point_sequence').run(ctx, None) is False
    assert visited == [(1, 4), (7, 4)]


def test_route_template_uses_point_default_mode_and_obeys_cancellation():
    visited = []
    ctx = SimpleNamespace(config={'points': [1, 7]}, cancelled=False)

    def go_to_point(point, mode=None):
        visited.append((point, mode))
        ctx.cancelled = True
        return True

    ctx.go_to_point = go_to_point
    assert load_template('point_sequence').run(ctx, None) is False
    assert visited == [(1, None)]


def test_special_template_propagates_interruptible_wait_result():
    durations = []

    def wait(seconds):
        durations.append(seconds)
        return False

    ctx = SimpleNamespace(config={'wait_seconds': 0.75}, cancelled=False, log=lambda _: None, wait=wait)
    assert load_template('on_demand').run(ctx, None) is False
    assert durations == [0.75]
