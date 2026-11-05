import importlib.util
from pathlib import Path

import pytest

PATH = Path(__file__).parents[2] / 'src/function/framework/core/parameters.py'
spec = importlib.util.spec_from_file_location('transaction', PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ParameterTransaction = module.ParameterTransaction


class Backend:
    def __init__(self):
        self.values = {'controller_server': {'speed': 1.0}, 'velocity_smoother': {'max': [1.0, 2.0]}}
        self.reject = None
        self.pending = False

    def get(self, node, names):
        return {n: self.values[node][n] for n in names}

    def set(self, node, values):
        if self.reject == node:
            raise RuntimeError('rejected')
        self.values[node].update(values)

    def settle(self):
        if self.pending:
            raise TimeoutError('request still pending')


def test_multiple_modes_restore_live_baseline_not_hardcoded_defaults():
    b = Backend()
    b.values['controller_server']['speed'] = 0.45
    t = ParameterTransaction(b)
    t.begin({'controller_server': ['speed'], 'velocity_smoother': ['max']})
    t.apply({'controller_server': {'speed': 3.0}, 'velocity_smoother': {'max': [3.0, 3.0]}})
    t.apply({'controller_server': {'speed': 0.5}})
    t.restore()
    assert b.values == {'controller_server': {'speed': 0.45}, 'velocity_smoother': {'max': [1.0, 2.0]}}
    assert not t.active


def test_partial_application_is_rolled_back_and_failure_does_not_leak():
    b = Backend()
    t = ParameterTransaction(b)
    t.begin({'controller_server': ['speed'], 'velocity_smoother': ['max']})
    b.reject = 'velocity_smoother'
    with pytest.raises(RuntimeError):
        t.apply({'controller_server': {'speed': 3.0}, 'velocity_smoother': {'max': [3.0, 3.0]}})
    b.reject = None
    t.restore()
    assert b.values['controller_server']['speed'] == 1.0


def test_failed_restoration_blocks_next_task_until_recovery():
    b = Backend()
    t = ParameterTransaction(b)
    t.begin({'controller_server': ['speed']})
    t.apply({'controller_server': {'speed': 3.0}})
    b.reject = 'controller_server'
    with pytest.raises(RuntimeError):
        t.restore()
    with pytest.raises(RuntimeError):
        t.begin({'controller_server': ['speed']})
    b.reject = None
    t.restore()
    assert not t.active
    assert b.values['controller_server']['speed'] == 1.0


def test_late_request_must_finish_before_restoration():
    b = Backend()
    t = ParameterTransaction(b)
    t.begin({'controller_server': ['speed']})
    t.apply({'controller_server': {'speed': 3.0}})
    b.pending = True
    with pytest.raises(TimeoutError):
        t.restore()
    assert t.active
    b.pending = False
    t.restore()
    assert b.values['controller_server']['speed'] == 1.0


def test_snapshot_failure_never_writes_anything():
    b = Backend()
    t = ParameterTransaction(b)
    with pytest.raises(KeyError):
        t.begin({'controller_server': ['missing']})
    assert not t.active
    assert b.values['controller_server']['speed'] == 1.0


def test_unsnapshotted_parameter_cannot_be_changed():
    b = Backend()
    t = ParameterTransaction(b)
    t.begin({'controller_server': ['speed']})
    with pytest.raises(ValueError):
        t.apply({'controller_server': {'typo': 3.0}})
