import importlib.util
from concurrent.futures import Future
from pathlib import Path
import threading
from types import SimpleNamespace
import pytest


spec = importlib.util.spec_from_file_location('function_menu', Path(__file__).parents[2] / 'src/function/framework/sum.py')
menu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(menu)
POINTS = {1: [0, 0, 0, 1, '测试目标', 1]}
TASKS = {-9: {'name': 'offset', 'description': '偏置移动', 'uses_offset': True}}
MODES = {1: {'name': '慢速'}}


def test_menu_waits_for_completion_then_allows_next_selection():
    submitted = []
    future = Future()
    state = {'reads': 0}
    output = []
    active = threading.Event()

    def read(prompt):
        state['reads'] += 1
        if state['reads'] == 1:
            return '1'
        assert future.done(), '不能在当前请求完成前读取下一次选择'
        assert not active.is_set()
        return 'q'

    def submit(number, offset):
        submitted.append((number, offset))
        assert active.is_set()
        threading.Timer(0.08, lambda: future.set_result(SimpleNamespace(success=True, message='arrived'))).start()
        return future

    menu.menu_loop(POINTS, TASKS, MODES, submit, threading.Event(), active, read, output.append)
    assert submitted == [(1, (0, 0, 0))]
    assert output.count('\nchairman_navigation 功能菜单') == 2
    assert output.index('完成：arrived') < len(output) - 1 - output[::-1].index('\nchairman_navigation 功能菜单')
    assert '完成：arrived' in output


def test_offset_prompt_and_failure_return_to_menu():
    answers = iter(['-9', '0.2 -0.1 0.0', '1', 'q'])
    calls = []
    output = []

    def submit(number, offset):
        calls.append((number, offset))
        future = Future()
        future.set_result(SimpleNamespace(success=False, message='cancelled'))
        return future

    menu.menu_loop(POINTS, TASKS, MODES, submit, threading.Event(), threading.Event(), lambda _: next(answers), output.append)
    assert calls == [(-9, (0.2, -0.1, 0.0)), (1, (0, 0, 0))]
    assert output.count('未完成：cancelled') == 2
    assert output.count('\nchairman_navigation 功能菜单') == 3


def test_unknown_id_and_invalid_offset_do_not_submit():
    answers = iter(['99', '-9', 'nan 0 0', 'q'])
    output = []
    menu.menu_loop(POINTS, TASKS, MODES, lambda *_: (_ for _ in ()).throw(AssertionError('unexpected request')),
                   threading.Event(), threading.Event(), lambda _: next(answers), output.append)
    assert any('没有功能 99' in line for line in output)
    assert '偏置必须是有限数值' in output


def test_stopping_during_task_exits_without_reading_next_selection():
    stopped = threading.Event()
    active = threading.Event()

    def submit(*_):
        stopped.set()
        return Future()

    menu.menu_loop(POINTS, TASKS, MODES, submit, stopped, active, lambda _: '1', lambda _: None)
    assert not active.is_set()


def test_menu_can_be_shown_on_request_without_submitting():
    answers = iter(['', 'm', 'M', 'q'])
    output = []
    menu.menu_loop(POINTS, TASKS, MODES, lambda *_: pytest.fail('unexpected submit'),
                   threading.Event(), threading.Event(), lambda _: next(answers), output.append)
    assert output.count('\nchairman_navigation 功能菜单') == 3
