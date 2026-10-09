"""按需任务调度：接收请求、执行功能、停止动作并恢复临时参数。

TaskServer 管理请求互斥与收尾；TaskContext 提供任务组合接口。"""
import threading
import time
from importlib import import_module

from framework.core.communication import CommunicationNode
from framework.sum import TaskProgress, ProgressLogger
from chairman_tasks.config import load_configuration
from chairman_tasks.fixed_point import task as navigation
from chairman_tasks.special.stair_forward import task as stair


class TaskServer(CommunicationNode):
    def __init__(self):
        super().__init__(self.srv_callback, self.stop_callback, self.restore_callback)
        self.task_progress = TaskProgress(super().get_logger())
        self.progress_logger = ProgressLogger(self.task_progress)
        self.task_lock = threading.Lock()
        load_configuration(self)
        validate_task_modules(self.task_configs)
        self.manual_control_reason = ''
        stair.stop_stair_mode(self)
        self.stair_last_time = self.get_clock().now()
        self.stair_timer = self.create_timer(self.dt, lambda: stair.stair_timer_callback(self),
                                             callback_group=self.reentrant_group)


    def get_logger(self):
        return getattr(self, 'progress_logger', None) or super().get_logger()

    def restore_callback(self, request, response):
        if not self.task_lock.acquire(blocking=False):
            response.success = False
            response.message = '当前任务尚未结束，请先停止任务'
            return response
        try:
            self.restore_parameters()
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
                stair.stop_stair_mode(self)
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
                stair.stop_stair_mode(self)
                self.publish_manual_zero_speed('任务收尾：手写控制速度清零')
                self.restore_parameters()
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

    def stop_callback(self, msg):
        self.cancel_current_task = True
        self.task_progress.update('收到中止请求，正在停车并恢复参数')
        self.manual_control_active = False
        self.stop_nav_cmd_tracking()
        stair.stop_stair_mode(self)


    def restore_parameters(self):
        self._nav_client.settle()
        navigation.restore_parameters(self)


def validate_task_modules(tasks):
    for task in tasks.values():
        module = import_module('chairman_tasks.' + task['module'])
        if not callable(getattr(module, 'run', None)):
            raise ValueError(f'{task["module"]} 缺少 run(ctx, request)')


def dispatch(node, request):
    task = node.task_configs.get(request.target)
    if task is None:
        if request.target not in node.PRESET_GOALS:
            raise ValueError(f'未知功能编号: {request.target}')
        task = {'module': 'fixed_point.task'}
    module = import_module('chairman_tasks.' + task['module'])
    result = module.run(TaskContext(node, task.get('parameters')), request)
    if type(result) is not bool:
        raise TypeError(f'{task["module"]}.run必须返回True或False')
    return result


class TaskContext:
    def __init__(self, node, parameters=None):
        self.node = node
        self.config = parameters or {}

    @property
    def cancelled(self):
        return self.node.cancel_current_task

    def log(self, message):
        self.node.get_logger().info(str(message))

    def point_mode(self, point_id):
        return self.node.PRESET_GOALS[point_id][5]

    def navigation(self, mode, callback):
        if self.cancelled:
            return False
        navigation.begin_parameters(self.node, mode)
        try:
            base = self.node.mode_configs[mode]['base_mode']
            return bool(callback(base)) and not self.cancelled
        finally:
            # 每个导航片段恢复参数后，组合任务才能执行下一步。
            self.node.restore_parameters()

    def go_to_point(self, point_id, mode=None):
        x, y, qz, qw, name, default_mode = self.node.PRESET_GOALS[point_id]
        if any(v is None for v in (x, y, qz, qw)):
            raise ValueError(f'{point_id}是动态目标，需调用对应特殊任务')
        return self.go_to_pose(x, y, qz, qw, default_mode if mode is None else mode, name)

    def go_to_pose(self, x, y, qz, qw, mode=1, name='自定义目标'):
        def navigate(base):
            if base == 3:
                return navigation.navigate_pre_align(self.node, x, y, qz, qw, name)
            return navigation.navigate(self.node, x, y, qz, qw, name, speed_profile=base)
        return self.navigation(mode, navigate)

    def align_region(self, mode=1):
        from chairman_tasks.special.align_region.task import navigate
        return self.navigation(mode, lambda base: navigate(self.node, speed_profile=base))

    def navigate_offset(self, offset, mode=1):
        from chairman_tasks.special.kfs_navigation.task import navigate
        return self.navigation(mode, lambda base: navigate(self.node, offset, speed_profile=base))

    def manual(self, callback, name='手写运动'):
        if self.cancelled:
            return False
        self.node.task_progress.stage(name, '正在执行手写控制')
        self.node.manual_control_active = True
        self.node.manual_control_reason = '按需任务手写运动'
        self.node.reset_filter_state()
        try:
            return bool(callback()) and not self.cancelled
        finally:
            stair.stop_stair_mode(self.node)
            self.node.manual_control_active = False
            self.node.manual_control_reason = ''
            self.node.publish_manual_zero_speed('手写运动片段结束')
            self.node.task_progress.end_stage('手写控制已结束，已发送零速度')

    def rotate(self, angle_degrees):
        from chairman_tasks.special.turn_left.task import rotate
        return self.manual(lambda: rotate(self.node, float(angle_degrees)), f'旋转 {angle_degrees}°')

    def move(self, local_x, local_y, distance):
        from chairman_tasks.special.move_forward.task import move
        return self.manual(lambda: move(self.node, local_x, local_y, distance), f'闭环移动 {distance}m')

    def move_offset(self, offset):
        from chairman_tasks.special.kfs_move.task import move_offset
        return self.manual(lambda: move_offset(self.node, offset), 'KFS 偏置移动')

    def uphill(self):
        from chairman_tasks.special.uphill.task import climb
        return self.manual(lambda: climb(self.node), '上坡')

    def stair(self, local_x, local_y, speed, name='登阶'):
        from chairman_tasks.special.stair_forward.task import start_corrected_stair_mode, wait_stair_stop_signal
        return self.manual(lambda: start_corrected_stair_mode(self.node, local_x, local_y, speed, name)
                           and wait_stair_stop_signal(self.node, name), name)

    def wait_lift(self):
        from chairman_tasks.special.lift_wait.task import wait_lift
        self.node.task_progress.stage('抬升检测', '等待高度变化达到阈值')
        try:
            return bool(wait_lift(self.node)) and not self.cancelled
        finally:
            self.node.task_progress.end_stage('抬升检测结束')

    def wait(self, seconds):
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            if self.cancelled:
                return False
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        return not self.cancelled
