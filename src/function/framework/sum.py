"""功能入口：菜单交互、任务状态显示及 ROS 进程退出管理。"""
import math
import os
import signal
import sys
import threading
import time
import unicodedata


def show_menu(points, tasks, modes, output=print):
    output('\nchairman_navigation 功能菜单')
    output('定点导航：')
    for number, point in points.items():
        if number not in tasks:
            output(f'  {number:>3}: {point[4]}（模式 {point[5]}：{modes[point[5]]["name"]}）')
    output('特殊功能：')
    for number, task in tasks.items():
        output(f'  {number:>3}: {task.get("description", task.get("name", number))}')
    output('输入功能编号；执行时 Ctrl+C 中止当前任务；输入 m 重看菜单；菜单中 q 或 Ctrl+C 退出。')


def menu_loop(points, tasks, modes, submit, stopped, active, read=input, output=print, progress=None):
    """同步等待本次请求完成，避免提前接收下一次选择。"""
    known_ids = set(points) | set(tasks)
    show_menu(points, tasks, modes, output)
    while not stopped.is_set():
        try:
            line = read('选择功能编号> ').strip()
            if line.lower() == 'q':
                return
            if line.lower() == 'm':
                show_menu(points, tasks, modes, output)
                continue
            if not line:
                continue
            target = int(line)
            if target not in known_ids:
                raise ValueError(f'没有功能 {target}，请按菜单选择')
            offset = (0.0, 0.0, 0.0)
            if tasks.get(target, {}).get('uses_offset', False):
                fields = read('输入偏置 x y z（单位 m，空格分隔）> ').split()
                if len(fields) != 3:
                    raise ValueError('偏置需要三个数，例如 0.2 0.0 0.0')
                offset = tuple(float(v) for v in fields)
                if not all(math.isfinite(v) for v in offset):
                    raise ValueError('偏置必须是有限数值')
            active.set()
            future = submit(target, offset)
            if progress is None:
                output(f'[任务已提交] 编号 {target}，等待执行结果；Ctrl+C 中止')
            while not future.done():
                if progress is not None:
                    progress.render()
                if stopped.wait(0.05):
                    return
            result = future.result()
            if progress is not None:
                progress.clear()
            if progress is None:
                output(f'{"完成" if result.success else "未完成"}：{result.message}')
            show_menu(points, tasks, modes, output)
        except EOFError:
            return
        except (ValueError, RuntimeError) as exc:
            output(str(exc))
        finally:
            if progress is not None:
                progress.clear()
            active.clear()


def fit_terminal(text, columns):
    """按终端显示宽度截断文本，避免中文字符导致状态行换行。"""
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


    def end_stage(self, result):
        with self.lock:
            self.detail = result


    def update(self, detail):
        with self.lock:
            self.detail = detail

    def fail(self, reason):
        with self.lock:
            self.failure = reason

    def finish(self, success, cancelled=False, cleanup_error=''):
        """汇总任务结果；调用前须完成动作停止与参数恢复的尝试。"""
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
                reason = (self.failure or '功能返回失败') + '；已发送零速度，临时参数已恢复'
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
            # 状态条表示任务正在执行，不表示完成百分比。
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
    """输出日志前清除状态行，保留 ROS 日志等级及文件记录。"""
    def __init__(self, progress):
        self.progress = progress

    def __getattr__(self, name):
        if name in ('info', 'warn', 'warning', 'error', 'fatal'):
            return lambda *args, **kwargs: self.progress.log(name, *args, **kwargs)
        return getattr(self.progress.logger, name)


def run_console(menu=None, args=None):
    """启动公共 ROS 接口；传入菜单回调进入交互，不传则仅提供服务。"""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from custom_msg.srv import SetNavTarget
    from std_msgs.msg import Empty
    from chairman_tasks.task import TaskServer
    # 退出恢复期间保持 ROS 上下文可用。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = MultiThreadedExecutor(num_threads=6)
    client_node = None
    server = None
    worker = None
    stopped = threading.Event()
    active = threading.Event()
    previous_handlers = {}
    cancel_requested = threading.Event()
    try:
        server = TaskServer()
        executor.add_node(server)
        client_node = Node('chairman_function_console')
        client = client_node.create_client(SetNavTarget, '/set_nav_target')
        executor.add_node(client_node)

        def handle_signal(signum, frame):
            # 运行中 Ctrl+C 只取消当前请求，收尾后返回菜单。
            if signum == signal.SIGINT and active.is_set() and menu is not None:
                cancel_requested.set()
                return
            stopped.set()
            cancel_requested.set()
            if menu is not None:
                raise KeyboardInterrupt

        def process_cancel():
            # 等任务取得执行锁后再递交取消，避免初始化重置取消标记。
            if cancel_requested.is_set() and server.task_lock.locked() and not server.cancel_current_task:
                server.stop_callback(Empty())

        client_node.create_timer(0.05, process_cancel)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, handle_signal)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()

        def submit(target, offset):
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('/set_nav_target 尚未就绪')
            request = SetNavTarget.Request(target=target)
            request.kfs_offset.x, request.kfs_offset.y, request.kfs_offset.z = offset
            future = client.call_async(request)
            future.add_done_callback(lambda _: cancel_requested.clear())
            return future

        if menu is None:
            while not stopped.wait(0.1):
                pass
        else:
            menu(server.PRESET_GOALS, server.task_configs, server.mode_configs, submit, stopped, active,
                 progress=server.task_progress)
    except KeyboardInterrupt:
        pass
    finally:
        # 收尾期间忽略重复信号，保持 ROS 回调可完成取消和参数恢复。
        for sig in previous_handlers:
            signal.signal(sig, signal.SIG_IGN)
        if server is not None:
            server.task_progress.clear()
            server.stop_callback(None)
            if server.task_lock.acquire(timeout=15.0):
                try:
                    server.restore_parameters()
                except Exception as exc:
                    print(f'退出前参数恢复失败：{exc}；重新启动 Nav2 可重载磁盘基准参数')
                finally:
                    server.task_lock.release()
            else:
                print('任务未及时结束；请重新启动 Nav2 以重载磁盘基准参数')
        executor.shutdown(timeout_sec=3.0)
        if worker is not None:
            worker.join(timeout=1.0)
        if server is not None:
            server.destroy_node()
        if client_node is not None:
            client_node.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


def main(args=None):
    run_console(menu=menu_loop, args=args)


def server_main(args=None):
    run_console(args=args)


if __name__ == '__main__':
    main()
