"""Opt-in local fake Nav2 integration; no drivers or real action servers."""
import os
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('CHAIRMAN_ROS_INTEGRATION') != '1',
    reason='Set CHAIRMAN_ROS_INTEGRATION=1 in an isolated ROS_DOMAIN_ID to run fake ROS servers')


def wait(future, timeout=12.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, 'ROS request timed out'
        time.sleep(0.01)
    return future.result()


@pytest.fixture
def scene():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from nav2_msgs.action import NavigateToPose
    from custom_msg.srv import SetNavTarget
    from framework.core.runtime import FunctionRuntime

    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use localhost-only DDS for fake tests'
    assert os.environ.get('ROS_DOMAIN_ID') == '197', 'Use the dedicated test domain 197'
    rclpy.init()
    node = FunctionRuntime()
    node.PRESET_GOALS = dict(node.PRESET_GOALS)
    for mode in (1, 2, 3):
        node.PRESET_GOALS[mode] = [0.0, 0.0, 0.0, 1.0, 'TEST', mode]
    node.get_current_map_pose = lambda **kwargs: (0.0, 0.0, 0.0)
    controller = Node('controller_server')
    smoother = Node('velocity_smoother')
    baselines = {}
    for remote, group in ((controller, 'controller'), (smoother, 'velocity_smoother')):
        values = node.NAV2_SPEED_PROFILES[1][group]
        for key, value in values.items():
            remote.declare_parameter(key, value)
        baselines[remote] = {k: remote.get_parameter(k).to_parameter_msg().value for k in values}
    fake = Node('fake_nav2')
    state = {'hold': False, 'started': False, 'cancelled': 0}

    def execute(handle):
        state['started'] = True
        state['observed_kp'] = controller.get_parameter('FollowPath.translation_kp').value
        while state['hold'] and not handle.is_cancel_requested:
            time.sleep(0.01)
        if handle.is_cancel_requested:
            state['cancelled'] += 1
            handle.canceled()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute_callback=execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    console = Node('test_function_client')
    client = console.create_client(SetNavTarget, '/set_nav_target')
    executor = MultiThreadedExecutor(num_threads=8)
    for instance in (node, controller, smoother, fake, console):
        executor.add_node(instance)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.wait_for_service(timeout_sec=5.0)

    def check_baseline():
        for remote, parameters in baselines.items():
            assert {k: remote.get_parameter(k).to_parameter_msg().value for k in parameters} == parameters
        assert not node.parameter_transaction.active
        assert not node._nav_client.pending

    yield node, controller, smoother, client, state, check_baseline
    state['hold'] = False
    node.cancel_current_task = True
    executor.shutdown(timeout_sec=5.0)
    thread.join(timeout=2.0)
    action.destroy()
    for instance in (node, controller, smoother, fake, console):
        instance.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('mode', [1, 2, 3])
def test_modes_restore_real_ros_parameters(scene, mode):
    from custom_msg.srv import SetNavTarget
    _, _, _, client, _, check = scene
    result = wait(client.call_async(SetNavTarget.Request(target=mode)))
    assert result.success, result.message
    check()


def test_partial_remote_rejection_restores_controller(scene):
    from custom_msg.srv import SetNavTarget
    from rcl_interfaces.msg import SetParametersResult
    node, controller, smoother, client, _, check = scene
    # Mode2 differs from baseline; reject only its smoother update, accept restore.
    smoother.add_on_set_parameters_callback(lambda ps: SetParametersResult(
        successful=not any(p.name == 'max_velocity' and list(p.value) == node.NAV2_SPEED_PROFILES[2]['velocity_smoother']['max_velocity'] for p in ps),
        reason='test rejection'))
    result = wait(client.call_async(SetNavTarget.Request(target=2)))
    assert not result.success
    check()


def test_emergency_cancel_settles_action_and_restores(scene):
    from custom_msg.srv import SetNavTarget
    from std_msgs.msg import Empty
    node, _, _, client, state, check = scene
    state['hold'] = True
    future = client.call_async(SetNavTarget.Request(target=1))
    deadline = time.monotonic() + 6
    while not state['started']:
        assert time.monotonic() < deadline
        time.sleep(0.01)
    node.stop_callback(Empty())
    result = wait(future)
    assert not result.success
    assert state['cancelled'] == 1
    check()


def test_switch_failure_during_running_goal_cancels_goal(scene):
    from custom_msg.srv import SetNavTarget
    from rcl_interfaces.msg import SetParametersResult
    node, controller, _, client, state, check = scene
    state['hold'] = True
    node.get_nav2_distance_to_goal = lambda *args, **kwargs: 0.0 if state['started'] else 10.0
    controller.add_on_set_parameters_callback(lambda ps: SetParametersResult(
        successful=not any(p.name == 'FollowPath.translation_kp' and p.value == node.NAV2_SPEED_PROFILES[2]['controller']['FollowPath.translation_kp'] for p in ps),
        reason='test fine-mode rejection'))
    result = wait(client.call_async(SetNavTarget.Request(target=2)))
    assert not result.success
    assert state['cancelled'] == 1
    check()


@pytest.mark.parametrize('base_mode', [1, 2, 3])
def test_new_mode_inherits_algorithm_applies_override_and_restores(scene, base_mode):
    from custom_msg.srv import SetNavTarget
    node, _, _, client, state, check = scene
    node.mode_configs[4] = {'id': 4, 'name': 'precise', 'base_mode': base_mode,
                            'overrides': {'controller_server': {'FollowPath.translation_kp': 9.125}}}
    node.PRESET_GOALS[33] = [0.0, 0.0, 0.0, 1.0, 'new point', 4]
    result = wait(client.call_async(SetNavTarget.Request(target=33)))
    assert result.success, result.message
    assert state['observed_kp'] == 9.125
    check()


def test_new_sequence_task_uses_context_without_changing_dispatch(scene, monkeypatch):
    import sys
    from types import ModuleType
    from custom_msg.srv import SetNavTarget
    node, controller, _, client, state, check = scene
    node.mode_configs[4] = {'id': 4, 'base_mode': 1, 'overrides': {'controller_server': {'FollowPath.translation_kp': 9.125}}}
    baseline_kp = controller.get_parameter('FollowPath.translation_kp').value
    observed = []

    def rotate(runtime, angle):
        assert runtime is node
        observed.append((angle, node.manual_control_active, controller.get_parameter('FollowPath.translation_kp').value))
        return True

    from chairman_tasks.special.turn_left import task as turn_task
    monkeypatch.setattr(turn_task, 'rotate', rotate)
    module = ModuleType('chairman_tasks.special.user_sequence.task')

    def run(ctx, request):
        return ctx.go_to_point(1, mode=4) and ctx.rotate(ctx.config['angle']) and ctx.go_to_point(1, mode=1)

    module.run = run
    monkeypatch.setitem(sys.modules, module.__name__, module)
    node.task_configs[-12] = {'module': 'special.user_sequence.task', 'parameters': {'angle': 45.0}}
    result = wait(client.call_async(SetNavTarget.Request(target=-12)))
    assert result.success, result.message
    assert observed == [(45.0, True, baseline_kp)]
    assert not node.manual_control_active
    check()



@pytest.mark.parametrize('field', [None, '4'])
def test_sum_without_arguments_cancels_then_returns_to_menu(field):
    import signal
    import subprocess
    from queue import Queue, Empty

    assert os.environ.get('ROS_DOMAIN_ID') == '197'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    # Real console and service in the isolated test domain; no odometry/driver.
    environment = {**os.environ, 'PYTHONUNBUFFERED': '1'}
    environment.pop('SELECTED_POSE', None)
    if field is not None:
        environment['SELECTED_POSE'] = field
    process = subprocess.Popen(['ros2', 'run', 'framework', 'sum'], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               env=environment, start_new_session=True)
    output = Queue()
    thread = threading.Thread(target=lambda: [output.put(line) for line in process.stdout], daemon=True)
    thread.start()
    transcript = []

    def until(marker):
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            try:
                line = output.get(timeout=0.1)
            except Empty:
                assert process.poll() is None, ''.join(transcript)
                continue
            transcript.append(line)
            if marker in line:
                return
        raise AssertionError('No marker ' + marker + '\n' + ''.join(transcript))

    try:
        until('菜单中 q 或 Ctrl+C 退出')
        assert '初始位置' not in ''.join(transcript)
        assert f'场地编号: {field or 1}' in ''.join(transcript)
        process.stdin.write('-11\n')
        process.stdin.flush()
        until('抬升自检测启动')
        os.killpg(process.pid, signal.SIGINT)
        until('未完成：')
        until('菜单中 q 或 Ctrl+C 退出')
        process.stdin.write('q\n')
        process.stdin.flush()
        assert process.wait(timeout=8) == 0
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
        process.stdin.close()
        thread.join(timeout=1)
        process.stdout.close()
