"""公共运行接口：ROS 连接、任务提交、取消、参数恢复及退出清理。"""
import signal
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from custom_msg.srv import SetNavTarget
from std_msgs.msg import Empty

from rclpy.node import Node
from std_srvs.srv import Trigger

from .node_setup import NodeSetupMixin
from .defaults import DefaultsMixin
from .runtime_parameters import RuntimeParametersMixin
from .tracking import TrackingMixin
from .motion import MotionMixin
from chairman_tasks.special.stair_forward.controller import StairMixin
from chairman_tasks.fixed_point.task import FixedPointMixin
from chairman_tasks.fixed_point.modes.dynamic import DynamicProfileMixin
from chairman_tasks.fixed_point.modes.pre_align import PreAlignMixin
from .registry import dispatch, validate_task_modules
from .actions import TrackedActionClient
from .progress import TaskProgress, ProgressLogger


class FunctionRuntime(NodeSetupMixin, DefaultsMixin, RuntimeParametersMixin,
                    TrackingMixin, MotionMixin, StairMixin, FixedPointMixin, DynamicProfileMixin,
                    PreAlignMixin, Node):
    def __init__(self):
        super().__init__()
        self.task_progress = TaskProgress(super().get_logger())
        self.progress_logger = ProgressLogger(self.task_progress)
        self.initialize_transactions()
        self._nav_client = TrackedActionClient(self, self._nav_client)
        self.task_lock = threading.Lock()
        self.restore_service = self.create_service(
            Trigger, '/restore_navigation_parameters', self.restore_callback,
            callback_group=self.reentrant_group)

    def get_logger(self):
        return getattr(self, 'progress_logger', None) or super().get_logger()

    def restore_callback(self, request, response):
        if not self.task_lock.acquire(blocking=False):
            response.success = False
            response.message = '当前任务尚未结束，请先停止任务'
            return response
        try:
            self._nav_client.settle()
            self.restore_temporary_parameters()
            response.success = True
            response.message = '原参数已恢复'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        finally:
            self.task_lock.release()
        return response

    def srv_callback(self, request, response):
        if not self.task_lock.acquire(blocking=False):
            response.success = False
            response.message = '已有任务执行中，请等待结束或先急停'
            self.get_logger().warn(f'[任务拒绝] 编号 {request.target} | {response.message}')
            return response
        task = self.task_configs.get(request.target)
        if task is not None:
            name = task.get('description', task.get('name', str(request.target)))
        elif request.target in self.PRESET_GOALS:
            point = self.PRESET_GOALS[request.target]
            name = f'{point[4]}（模式 {point[5]}）'
        else:
            name = '未知功能'
        self.cancel_current_task = False
        self.task_progress.begin(f'编号 {request.target}：{name}')
        cleanup_error = ''
        try:
            if self.parameter_transaction.active or self._nav_client.pending:
                raise RuntimeError('上一任务参数未恢复，请调用 /restore_navigation_parameters')
            self.stop_nav_cmd_tracking('新任务：清除旧速度')
            if self.in_stair_mode:
                self.stop_stair_mode()
            self.task_progress.update('正在执行功能，等待结束条件')
            response.success = bool(dispatch(self, request)) and not self.cancel_current_task
        except Exception as exc:
            response.success = False
            self.task_progress.fail(str(exc))
        finally:
            self.task_progress.update('正在确认动作停止并恢复临时参数')
            self.manual_control_active = False
            self.manual_control_reason = ''
            try:
                self.stop_nav_cmd_tracking('任务收尾：速度清零')
                self.stop_stair_mode()
                self.publish_manual_zero_speed('任务收尾：手写控制速度清零')
                self._nav_client.settle()
                self.restore_temporary_parameters()
            except Exception as exc:
                response.success = False
                cleanup_error = str(exc)
            finally:
                response.success = response.success and not self.cancel_current_task
                try:
                    response.message = self.task_progress.finish(
                        response.success, self.cancel_current_task, cleanup_error)
                finally:
                    self.task_lock.release()
        return response


def run_runtime(menu=None, args=None):
    """启动公共 ROS 接口；传入菜单回调进入交互，不传则仅提供服务。"""
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
        server = FunctionRuntime()
        validate_task_modules(server.task_configs)
        executor.add_node(server)
        client_node = Node('chairman_function_console')
        client = client_node.create_client(SetNavTarget, '/set_nav_target')
        executor.add_node(client_node)

        def handle_signal(signum, frame):
            # Ctrl+C during execution cancels this request and returns to the menu.
            if signum == signal.SIGINT and active.is_set() and menu is not None:
                cancel_requested.set()
                return
            stopped.set()
            cancel_requested.set()
            if menu is not None:
                raise KeyboardInterrupt

        def process_cancel():
            # Retry until the service takes the request; its initial reset must
            # not swallow a Ctrl+C received immediately after call_async().
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
        # Ignore repeated signals while ROS callbacks complete cancellation/rollback.
        for sig in previous_handlers:
            signal.signal(sig, signal.SIG_IGN)
        if server is not None:
            server.task_progress.clear()
            server.cancel_current_task = True
            server.stop_nav_cmd_tracking('退出功能框架')
            server.stop_stair_mode()
            if server.task_lock.acquire(timeout=15.0):
                try:
                    server._nav_client.settle()
                    server.restore_temporary_parameters()
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
