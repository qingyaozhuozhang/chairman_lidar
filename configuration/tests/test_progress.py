import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace

import pytest


spec = importlib.util.spec_from_file_location(
    'task_progress', Path(__file__).parents[2] / 'src/function/framework/sum.py')
progress = importlib.util.module_from_spec(spec)
spec.loader.exec_module(progress)


class Terminal(io.StringIO):
    def isatty(self):
        return True


def reporter(tty=False):
    stream = Terminal() if tty else io.StringIO()
    messages = []
    logger = SimpleNamespace(info=messages.append, warn=messages.append, error=messages.append)
    now = [0.0]
    state = progress.TaskProgress(logger, stream, lambda: now[0])
    state.begin('编号 0：原点（模式 3）')
    state.stage('模式3 1/2 提前转正')
    return state, stream, messages, now


def test_nonterminal_keeps_boundaries_without_periodic_log_spam():
    state, stream, messages, now = reporter()
    for i in range(100):
        now[0] = i * 0.3
        state.update(f'距提前点 {i}m')
        state.render()
    assert stream.getvalue() == ''
    assert len(messages) == 1
    state.end_stage('第一段动作已结束')
    assert len(messages) == 1
    state.finish(True)
    assert len(messages) == 2
    assert messages[0].startswith('[任务开始]')
    assert messages[1].startswith('[任务完成]')


def test_tty_updates_one_line_throttles_and_clears_before_logs(monkeypatch):
    state, stream, messages, now = reporter(tty=True)
    monkeypatch.setattr(progress.os, 'get_terminal_size', lambda *_: SimpleNamespace(columns=42))
    stream.fileno = lambda: 1
    state.update('距提前点 0.300m | 朝向误差 10°')
    state.render()
    first = stream.getvalue()
    now[0] = 0.1
    state.render()
    assert stream.getvalue() == first
    now[0] = 0.3
    state.render()
    assert stream.getvalue().count('\r\x1b[2K') == 2
    assert '\n' not in stream.getvalue()
    for line in stream.getvalue().split('\r\x1b[2K'):
        width = sum(2 if progress.unicodedata.east_asian_width(c) in 'WF' else 1 for c in line)
        assert width < 42
    progress.ProgressLogger(state).warn('模拟 Nav2 告警')
    assert not state.visible
    assert messages[-1] == '模拟 Nav2 告警'
    assert stream.getvalue().endswith('\r\x1b[2K')
    state.finish(True)
    terminal = stream.getvalue()
    now[0] = 1.0
    state.render()
    assert stream.getvalue() == terminal


@pytest.mark.parametrize('success,cancelled,cleanup,state_label', [
    (True, False, '', '任务完成'),
    (False, False, '', '任务失败'),
    (False, True, '', '任务中止'),
    (True, False, '恢复超时', '任务失败'),
    (False, True, '动作未结束', '任务失败'),
])
def test_terminal_state_distinguishes_cancel_failure_and_failed_cleanup(success, cancelled, cleanup, state_label):
    state, _, messages, _ = reporter()
    if not success and not cancelled:
        state.fail('ABORTED：Nav2 执行失败（状态码 6）')
    result = state.finish(success, cancelled, cleanup)
    assert result.startswith(f'[{state_label}]')
    assert '编号 0：原点' in result and '结束位置：模式3 1/2' in result
    assert messages[-1] == result
    if cleanup:
        assert '尚未确认' in result
        assert '临时参数已恢复' not in result
    elif not success and not cancelled:
        assert 'ABORTED' in result


def test_real_ros_logger_supports_mixed_severities():
    from rclpy.logging import get_logger
    state = progress.TaskProgress(get_logger('task_progress_test'), io.StringIO())
    state.begin('测试日志')
    state.log('warn', '测试告警')
    state.log('error', '测试错误')
    state.finish(False, cancelled=True)
