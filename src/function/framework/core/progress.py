"""任务/阶段边界写入 ROS 日志；交互终端中的运行状态只刷新同一行。"""
import os
import sys
import threading
import time
import unicodedata


def fit_terminal(text, columns):
    """按显示宽度截断，避免中文状态条换行后刷屏。"""
    result, width = [], 0
    for char in text.replace('\n', ' ').replace('\r', ' '):
        size = 0 if unicodedata.combining(char) else (2 if unicodedata.east_asian_width(char) in 'WF' else 1)
        if width + size > columns:
            break
        result.append(char)
        width += size
    return ''.join(result)


class TaskProgress:
    def __init__(self, logger, stream=None, clock=time.monotonic):
        self.logger = logger
        self.stream = sys.stderr if stream is None else stream
        self.clock = clock
        self.lock = threading.RLock()
        self.running = False
        self.visible = False
        self.stage_name = '准备执行'
        self.detail = ''
        self.failure = ''
        self.last_draw = float('-inf')

    def clear(self):
        with self.lock:
            if self.visible:
                self.stream.write('\r\033[2K')
                self.stream.flush()
                self.visible = False

    def log(self, method, *args, **kwargs):
        with self.lock:
            self.clear()
            # rclpy 按调用位置缓存 severity，不同等级必须使用不同调用位置。
            if method == 'info':
                return self.logger.info(*args, **kwargs)
            if method in ('warn', 'warning'):
                return self.logger.warn(*args, **kwargs)
            if method == 'error':
                return self.logger.error(*args, **kwargs)
            if method == 'fatal':
                return self.logger.fatal(*args, **kwargs)
            raise ValueError(f'不支持的日志等级：{method}')

    def begin(self, label):
        with self.lock:
            self.label = label
            self.started = self.clock()
            self.running = True
            self.stage_name = '功能执行'
            self.detail = '正在准备功能'
            self.failure = ''
            self.last_draw = float('-inf')
            self.log('info', f'[任务开始] {label}')

    def stage(self, name, detail=''):
        with self.lock:
            self.stage_name = name
            self.detail = detail
            self.log('info', f'[阶段开始] {name}' + (f' | {detail}' if detail else ''))

    def end_stage(self, result):
        with self.lock:
            self.detail = result
            self.log('info', f'[阶段结束] {self.stage_name} | {result}')

    def update(self, detail):
        with self.lock:
            self.detail = detail

    def fail(self, reason):
        with self.lock:
            self.failure = reason

    def finish(self, success, cancelled=False, cleanup_error=''):
        """只能在停止动作和恢复参数的尝试结束后调用。"""
        with self.lock:
            if cleanup_error:
                state = '任务失败'
                reason = f'停止/参数恢复尚未确认：{cleanup_error}；请先恢复后再执行下一任务'
                if self.failure:
                    reason = self.failure + '；' + reason
            elif cancelled:
                state, reason = '任务中止', '收到中止请求；动作已结束，已发送零速度，临时参数已恢复'
            elif success:
                state, reason = '任务完成', '执行成功；动作已结束，已发送零速度，临时参数已恢复'
            else:
                state = '任务失败'
                reason = (self.failure or '功能返回失败，请查看上方告警') + '；已发送零速度，临时参数已恢复'
            self.running = False
            elapsed = self.clock() - self.started
            message = f'[{state}] {self.label} | 结束位置：{self.stage_name} | {reason} | 用时 {elapsed:.1f}s'
            self.log('info' if state == '任务完成' else 'warn', message)
            return message

    def render(self):
        with self.lock:
            now = self.clock()
            if not self.running or not self.stream.isatty() or now - self.last_draw < 0.2:
                return
            # 流动条仅表示仍在执行，不把耗时或直线距离冒充任务完成百分比。
            step = int((now - self.started) * 5) % 12
            bar = ''.join('=' if (i - step) % 12 < 3 else ' ' for i in range(12))
            text = f'[{bar}] {self.stage_name} | {self.detail} | {now - self.started:.1f}s'
            try:
                columns = os.get_terminal_size(self.stream.fileno()).columns
            except (OSError, ValueError, AttributeError):
                columns = 100
            self.stream.write('\r\033[2K' + fit_terminal(text, max(0, columns - 1)))
            self.stream.flush()
            self.visible = True
            self.last_draw = now


class ProgressLogger:
    """普通告警打印前也清除状态条，保留原 ROS 日志等级和文件记录。"""
    def __init__(self, progress):
        self.progress = progress

    def __getattr__(self, name):
        if name in ('info', 'warn', 'warning', 'error', 'fatal'):
            return lambda *args, **kwargs: self.progress.log(name, *args, **kwargs)
        return getattr(self.progress.logger, name)
