"""Stable API for task.py authors. ROS wiring and rollback stay in the framework."""
import time


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
        return self.node.get_preset_goal(point_id)[5]

    def navigation(self, mode, callback):
        if self.cancelled:
            return False
        self.node.begin_temporary_parameters(mode)
        try:
            base = self.node.mode_configs[mode]['base_mode']
            return bool(callback(base)) and not self.cancelled
        finally:
            # Each navigation segment restores before a following manual operation.
            self.node._nav_client.settle()
            self.node.restore_temporary_parameters()

    def go_to_point(self, point_id, mode=None):
        x, y, qz, qw, name, default_mode = self.node.get_preset_goal(point_id)
        if any(v is None for v in (x, y, qz, qw)):
            raise ValueError(f'{point_id}是动态目标，需调用对应特殊任务')
        return self.go_to_pose(x, y, qz, qw, default_mode if mode is None else mode, name)

    def go_to_pose(self, x, y, qz, qw, mode=1, name='自定义目标'):
        def navigate(base):
            if base == 3:
                return self.node.execute_profile3_pre_yaw_align_goal(x, y, qz, qw, name)
            return self.node.execute_nav2_goal(x, y, qz, qw, name, speed_profile=base)
        return self.navigation(mode, navigate)

    def align_region(self, mode=1):
        from chairman_tasks.special.align_region.task import navigate
        return self.navigation(mode, lambda base: navigate(self.node, speed_profile=base))

    def navigate_offset(self, offset, mode=1):
        from chairman_tasks.special.kfs_navigation.task import navigate
        return self.navigation(mode, lambda base: navigate(self.node, offset, speed_profile=base))

    def manual(self, callback):
        if self.cancelled:
            return False
        self.node.manual_control_active = True
        self.node.manual_control_reason = '按需任务手写运动'
        self.node.reset_filter_state()
        try:
            return bool(callback()) and not self.cancelled
        finally:
            self.node.stop_stair_mode()
            self.node.manual_control_active = False
            self.node.manual_control_reason = ''
            self.node.publish_manual_zero_speed('手写运动片段结束')

    def rotate(self, angle_degrees):
        from chairman_tasks.special.turn_left.task import rotate
        return self.manual(lambda: rotate(self.node, float(angle_degrees)))

    def move(self, local_x, local_y, distance):
        from chairman_tasks.special.move_forward.task import move
        return self.manual(lambda: move(self.node, local_x, local_y, distance))

    def move_offset(self, offset):
        from chairman_tasks.special.kfs_move.task import move_offset
        return self.manual(lambda: move_offset(self.node, offset))

    def uphill(self):
        from chairman_tasks.special.uphill.task import climb
        return self.manual(lambda: climb(self.node))

    def stair(self, local_x, local_y, speed, name='登阶'):
        from chairman_tasks.special.stair_forward.task import start_corrected_stair_mode, wait_stair_stop_signal
        return self.manual(lambda: start_corrected_stair_mode(self.node, local_x, local_y, speed, name)
                           and wait_stair_stop_signal(self.node, name))

    def wait_lift(self):
        from chairman_tasks.special.lift_wait.task import wait_lift
        return bool(wait_lift(self.node)) and not self.cancelled

    def wait(self, seconds):
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            if self.cancelled:
                return False
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        return not self.cancelled
