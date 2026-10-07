"""Mode 3 must finish the first action before submitting the real goal."""
from concurrent.futures import Future
import importlib.util
import math
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).parents[2]


def load_source(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


actions = load_source('pre_align_actions', 'src/function/framework/core/actions.py')
pre_align = load_source('pre_align_impl', 'src/function/detail/on_demand/fixed_point/modes/pre_align.py')
fixed = load_source('pre_align_fixed', 'src/function/detail/on_demand/fixed_point/task.py')
progress = load_source('task_progress', 'src/function/framework/core/progress.py')


def done(value):
    future = Future()
    future.set_result(value)
    return future


class Handle:
    accepted = True

    def __init__(self, owner, status=None):
        self.owner = owner
        self.result = Future()
        self.cancel_count = 0
        if status is not None:
            self.result.set_result(SimpleNamespace(status=status))

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        if self.cancel_count == 1 and self.owner.cancel_delay is not None:
            def finish_cancel():
                if self.owner.emergency_during_handoff:
                    self.owner.cancel_current_task = True
                if not self.result.done():
                    self.result.set_result(SimpleNamespace(status=5))
            timer = threading.Timer(self.owner.cancel_delay, finish_cancel)
            timer.daemon = True
            timer.start()
            self.owner.timers.append(timer)
        # Cancel accepted immediately; the action still runs until the timer fires.
        return done(SimpleNamespace(return_code=0))


class Robot(pre_align.PreAlignMixin, fixed.FixedPointMixin):
    def __init__(self, initial_x, *, release='yaw', cancel_delay=0.08, emergency=False):
        self.initial_x = initial_x
        self.release = release
        self.cancel_delay = cancel_delay
        self.emergency_during_handoff = emergency
        self.cancel_current_task = False
        self.nav_cmd_tracking_enabled = False
        self.nav2_profile_3_pre_align_enabled = True
        self.nav2_profile_3_pre_align_distance = 0.5
        self.nav2_profile_3_pre_align_release_xy_tolerance = 0.1
        self.nav2_profile_3_pre_align_release_yaw_tolerance = 0.15
        self.nav2_profile_3_pre_align_near_adjust_timeout = 0.0
        self.handles = []
        self.goals = []
        self.timers = []
        self.events = []
        self.messages = []
        self.task_progress = progress.TaskProgress(self.get_logger())
        self.task_progress.begin('测试点')
        self._nav_client = actions.TrackedActionClient(self, self, timeout=0.5)

    def get_logger(self):
        return SimpleNamespace(info=self.messages.append, warn=self.messages.append,
                               error=self.messages.append, debug=self.messages.append)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: pre_align.rclpy.time.Time().to_msg()))

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        if not self.handles:
            handle = Handle(self, status=4 if self.release == 'natural' else None)
        else:
            ended = self.handles[0].result.done()
            self.events.append(('second_goal', ended))
            handle = Handle(self, status=4 if ended else 6)
        self.handles.append(handle)
        return done(handle)

    def get_current_map_pose(self, **kwargs):
        if not self.goals:
            return self.initial_x, 0.0, 0.0
        position = self.goals[0].pose.pose.position
        return position.x, position.y, 0.4 if self.release == 'near_timeout' else 0.0

    def select_initial_nav2_profile(self, profile, *args):
        return profile

    def apply_nav2_speed_profile(self, profile):
        pass

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def start_nav_cmd_tracking(self, desc):
        self.nav_cmd_tracking_enabled = True

    def stop_nav_cmd_tracking(self, desc):
        self.nav_cmd_tracking_enabled = False
        self.events.append(('stop', len(self.goals)))

    def wait_future_done(self, future, timeout_sec):
        return future.done()

    def maybe_switch_nav2_profile_2_by_distance(self, *args):
        pass

    def run(self):
        return self.execute_profile3_pre_yaw_align_goal(2.0, 0.0, 0.0, 1.0, '测试点')

    def close(self):
        for timer in self.timers:
            timer.join(timeout=1)


@pytest.fixture(autouse=True)
def ros_is_running(monkeypatch):
    monkeypatch.setattr(actions.rclpy, 'ok', lambda: True)


@pytest.mark.parametrize('initial_x', [0.0, 1.75], ids=['approach_point', 'turn_in_place'])
@pytest.mark.parametrize('release', ['yaw', 'near_timeout', 'natural'])
def test_second_stage_waits_for_terminal_result(initial_x, release):
    robot = Robot(initial_x, release=release)
    try:
        assert robot.run(), '\n'.join(robot.messages)
        assert robot.events[-2:] == [('second_goal', True), ('stop', 2)]
        assert robot.goals[1].pose.pose.position.x == 2.0
        first_end = next(i for i, m in enumerate(robot.messages) if '[阶段结束] 模式3 1/2' in m)
        second_start = next(i for i, m in enumerate(robot.messages) if '[阶段开始] 模式3 2/2' in m)
        assert first_end < second_start
        if release == 'near_timeout':
            assert '按配置放行' in robot.messages[first_end]
            assert '转正完成' not in '\n'.join(robot.messages)
        assert not any('[任务完成]' in m for m in robot.messages), '动作阶段不得提前报告整次任务完成'
        robot._nav_client.settle()
        assert not robot._nav_client.pending
    finally:
        robot.close()


def test_no_second_goal_when_first_stage_end_cannot_be_confirmed():
    robot = Robot(0.0, cancel_delay=None)
    robot._nav_client.timeout = 0.03
    try:
        with pytest.raises(TimeoutError, match='尚未确认结束'):
            robot.run()
        assert len(robot.goals) == 1
        assert not robot.nav_cmd_tracking_enabled
        assert robot._nav_client.pending
        # A subsequent recovery can finish the pending action.
        robot.handles[0].result.set_result(SimpleNamespace(status=5))
        robot._nav_client.settle()
        assert not robot._nav_client.pending
    finally:
        robot.close()


def test_emergency_during_handoff_does_not_launch_second_stage():
    robot = Robot(0.0, emergency=True)
    try:
        assert not robot.run()
        assert len(robot.goals) == 1
        assert not robot.nav_cmd_tracking_enabled
    finally:
        robot.close()
