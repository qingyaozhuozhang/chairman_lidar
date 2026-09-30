import importlib.util
from pathlib import Path
import shlex


spec = importlib.util.spec_from_file_location('micro_agent', Path(__file__).parents[2] / 'src/function/detail/continuous/micro_ros/agent.py')
agent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agent)


def test_micro_ros_uses_current_environment_without_external_workspace():
    assert agent.agent_command({'device': '/dev/robot board', 'baud': 115200}) == [
        'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/robot board', '-b', '115200']


def test_relative_overlay_and_device_are_shell_quoted(tmp_path):
    root = tmp_path / 'navigation robot'
    root.mkdir()
    overlay = tmp_path / 'micro robot'
    (overlay / 'install').mkdir(parents=True)
    (overlay / 'install/setup.bash').touch()
    config = {'workspace': '../micro robot', 'device': '/dev/robot;unexpected', 'baud': 921600}
    command = agent.agent_command(config, root)
    assert command[:2] == ['bash', '-c']
    parts = shlex.split(command[2])
    assert parts[1] == str(root / '../micro robot/install/setup.bash')
    assert parts[-3:] == ['/dev/robot;unexpected', '-b', '921600']
