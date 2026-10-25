"""Independent fast/slow parameters, direct navigation and live-value restoration."""
from concurrent.futures import Future
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import shutil

import pytest
import yaml
from rclpy.parameter import Parameter
from rclpy.time import Time

from chairman_tasks import config
from chairman_tasks.fixed_point import task as navigation
from chairman_tasks.task import TaskContext
from framework.core.parameters import ParameterTransaction
from framework.sum import TaskProgress

ROOT = Path(__file__).parents[2]


class ParameterBackend:
    def __init__(self, values):
        self.values = deepcopy(values)
        self.writes = []

    def get(self, remote, names):
        return {name: self.values[remote][name] for name in names}

    def set(self, remote, values):
        self.writes.append((remote, deepcopy(values)))
        self.values[remote].update(values)

    def settle(self):
        pass


@pytest.fixture
def robot(tmp_path, monkeypatch):
    directory = tmp_path / 'config'
    shutil.copytree(ROOT / 'src/function/config', directory)
    for source_id, new_id in ((1, 4), (2, 5)):
        source = directory / f'on_demand/modes/speed_{source_id}.yaml'
        mode = yaml.safe_load(source.read_text())
        mode.update(id=new_id, name=f'speed_{new_id}')
        mode['parameters']['controller_server']['ros__parameters']['FollowPath']['translation_kpx'] = 8.25
        (directory / f'on_demand/modes/speed_{new_id}.yaml').write_text(yaml.safe_dump(mode))
    monkeypatch.setattr(config, 'config_root', lambda root=None: directory)
    monkeypatch.setattr('ament_index_python.packages.get_package_share_directory',
                        lambda name: str(ROOT / 'src/function/detail/continuous/odometry'))
    declared = {}
    logger = SimpleNamespace(**{kind: lambda *args: None for kind in ('info', 'warn', 'error', 'debug')})
    node = SimpleNamespace(
        cancel_current_task=False, nav_cmd_tracking_enabled=False, now_seconds=0.0,
        pose=(0.0, 0.0, 0.0), goals=[], status=4,
        declare_parameter=lambda name, value: declared.setdefault(name, value),
        get_parameter=lambda name: SimpleNamespace(value=declared[name]),
        get_logger=lambda: logger, task_progress=TaskProgress(logger))
    config.load_configuration(node)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=node.now_seconds))
    node.get_current_map_pose = lambda **kwargs: node.pose
    node.start_nav_cmd_tracking = lambda *_: setattr(node, 'nav_cmd_tracking_enabled', True)
    node.stop_nav_cmd_tracking = lambda *_: setattr(node, 'nav_cmd_tracking_enabled', False)

    def completed(value):
        future = Future()
        future.set_result(value)
        return future

    def send_goal(goal):
        node.goals.append(goal)
        return completed(SimpleNamespace(accepted=True,
            get_result_async=lambda: completed(SimpleNamespace(status=node.status))))

    node._nav_client = SimpleNamespace(wait_for_server=lambda: True, send_goal_async=send_goal,
                                       pending=False, settle=lambda: None)
    values = {remote: {name: Parameter(name, value=value).get_parameter_value()
                      for name, value in fields.items()}
              for remote, fields in node.speed_profiles[1].items()}
    values['controller_server']['FollowPath.translation_kpx'] = Parameter('kp', value=0.75).get_parameter_value()
    node.backend = ParameterBackend(values)
    node.baseline = deepcopy(values)
    node.parameter_transaction = ParameterTransaction(node.backend)
    node.restore_parameters = lambda: navigation.restore_parameters(node)
    return node


def test_custom_mode_uses_own_values_and_preserves_unlisted_live_parameters(robot):
    # An absent field must keep its live value, never come from another mode.
    robot.speed_profiles[4]['controller_server'].pop('FollowPath.translation_kix')
    robot.backend.values['controller_server']['FollowPath.translation_kix'] = Parameter('ki', value=0.625).get_parameter_value()
    baseline = deepcopy(robot.backend.values)
    navigation.begin_parameters(robot, 4)
    navigation.apply_profile(robot, 4)
    assert robot.backend.values['controller_server']['FollowPath.translation_kpx'].double_value == 8.25
    assert robot.backend.values['controller_server']['FollowPath.translation_kix'].double_value == 0.625
    robot.restore_parameters()
    assert robot.backend.values == baseline
    assert not robot.parameter_transaction.active


@pytest.mark.parametrize('mode', [1, 2, 4, 5])
@pytest.mark.parametrize('start_x', [0.0, 1.9])
def test_navigation_sends_only_final_goal_and_uses_selected_parameters(robot, mode, start_x):
    robot.pose = (start_x, 0.0, 0.0)
    send = robot._nav_client.send_goal_async
    observed = []

    def record(goal):
        observed.append(deepcopy(robot.backend.values))
        return send(goal)

    robot._nav_client.send_goal_async = record
    assert TaskContext(robot).go_to_pose(2.0, 0.0, 0.0, 1.0, mode=mode)
    assert [goal.pose.pose.position.x for goal in robot.goals] == [2.0]
    assert len(observed) == 1
    for remote, values in navigation.profile_values(robot, mode).items():
        assert all(observed[0][remote][key] == value for key, value in values.items())
    assert robot.backend.values == robot.baseline
    assert not robot.parameter_transaction.active
    assert not robot.nav_cmd_tracking_enabled


@pytest.mark.parametrize('mode', [1, 2])
def test_approaching_goal_keeps_one_profile_and_waits_for_action_result(robot, mode, monkeypatch):
    steps = iter([0.0, 0.5, 1.5, 1.99, 2.0])
    polls = []

    class PendingResult:
        def done(self):
            x = next(steps, None)
            if x is None:
                return True
            robot.pose = (x, 0.0, 0.0)
            polls.append(x)
            assert robot.current_profile == mode
            for remote, values in navigation.profile_values(robot, mode).items():
                assert all(robot.backend.values[remote][key] == value for key, value in values.items())
            return False

        def result(self):
            return SimpleNamespace(status=4)

    result = PendingResult()
    def send(goal):
        robot.goals.append(goal)
        future = Future()
        future.set_result(SimpleNamespace(accepted=True, get_result_async=lambda: result))
        return future

    robot._nav_client.send_goal_async = send
    monkeypatch.setattr(navigation.time, 'sleep', lambda _: None)
    assert TaskContext(robot).go_to_pose(2.0, 0.0, 0.0, 1.0, mode=mode)
    assert polls == [0.0, 0.5, 1.5, 1.99, 2.0]
    assert len(robot.goals) == 1
    # One application and one restore per remote node, with no distance-based updates.
    assert len(robot.backend.writes) == 2 * len(robot.speed_profiles[mode])
    assert robot.backend.values == robot.baseline


@pytest.mark.parametrize('outcome', ['failure', 'cancel', 'exception'])
def test_navigation_restores_selected_mode_on_unsuccessful_exit(robot, outcome):
    def execute(mode):
        navigation.apply_profile(robot, mode)
        assert robot.backend.values['controller_server']['FollowPath.translation_kpx'].double_value == 8.25
        if outcome == 'exception':
            raise RuntimeError('navigation failed')
        robot.cancel_current_task = outcome == 'cancel'
        return outcome != 'failure'

    if outcome == 'exception':
        with pytest.raises(RuntimeError, match='navigation failed'):
            TaskContext(robot).navigation(4, execute)
    else:
        assert not TaskContext(robot).navigation(4, execute)
    assert robot.backend.values == robot.baseline
    assert not robot.parameter_transaction.active
