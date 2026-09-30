import importlib.util
from pathlib import Path
import signal

import pytest
from types import SimpleNamespace


spec = importlib.util.spec_from_file_location('main_boot', Path(__file__).parents[1] / 'main_boot.py')
boot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boot)


def test_waits_for_descendants_after_wrapper_exit(monkeypatch):
    calls = []
    remaining = [True, True, False]
    monkeypatch.setattr(boot, 'group_alive', lambda _: remaining.pop(0))
    monkeypatch.setattr(boot.os, 'killpg', lambda pid, sig: calls.append((pid, sig)))
    monkeypatch.setattr(boot.time, 'sleep', lambda _: None)
    process = SimpleNamespace(pid=1234, poll=lambda: 0, wait=lambda **_: 0)
    boot.stop_process_group(process, timeout=1.0)
    assert not remaining, 'Must wait for whole group, even if wrapper is already gone'
    assert calls == [(1234, signal.SIGINT)]


def test_stubborn_descendant_killed_after_grace_period(monkeypatch):
    calls = []
    monkeypatch.setattr(boot, 'group_alive', lambda _: True)
    monkeypatch.setattr(boot.os, 'killpg', lambda pid, sig: calls.append((pid, sig)))
    process = SimpleNamespace(pid=1234, poll=lambda: 0, wait=lambda **_: 0)
    boot.stop_process_group(process, timeout=0.0)
    assert calls == [(1234, signal.SIGINT), (1234, signal.SIGKILL)]


def test_commands_quote_relocated_workspace(tmp_path):
    root = tmp_path / 'robot workspace'
    (root / 'install').mkdir(parents=True)
    (root / 'install/setup.bash').touch()
    commands = boot.build_commands(root, 4, no_micro=True, headless=True)
    assert [name for name, _ in commands] == ['Navigation2', 'Function sum', 'Odometry']
    assert all("source '" in command and 'SELECTED_POSE=4' in command for _, command in commands)
    assert commands[1][1].endswith('preset_nav_node')


def test_shutdown_defers_repeated_signals_and_stops_function_first(monkeypatch):
    handlers = {signal.SIGINT: 'original_int', signal.SIGTERM: 'original_term'}
    calls = []

    def set_handler(sig, handler):
        previous = handlers[sig]
        handlers[sig] = handler
        return previous

    def stop(process):
        assert all(h == signal.SIG_IGN for h in handlers.values())
        calls.append(process)

    monkeypatch.setattr(boot.signal, 'signal', set_handler)
    monkeypatch.setattr(boot, 'stop_process_group', stop)
    boot.shutdown_processes([('Navigation2', ''), ('Function sum', ''), ('Odometry', '')], [1, 2, 3])
    assert calls == [2, 1, 3]
    assert handlers == {signal.SIGINT: 'original_int', signal.SIGTERM: 'original_term'}


def test_boot_selects_field_once_and_passes_it_to_all_commands(tmp_path, monkeypatch, capsys):
    (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').touch()
    monkeypatch.setattr(boot, 'find_workspace', lambda _: tmp_path)
    answers = iter(['invalid', '4'])
    monkeypatch.setattr('builtins.input', lambda _: next(answers))
    boot.main(['--dry-run'])
    output = capsys.readouterr().out
    assert output.count('export SELECTED_POSE=4') == 4
    for command in ('ros2 run micro_ros agent', 'ros2 run framework sum', 'ros2 run odometry odometry'):
        assert command in output


@pytest.mark.parametrize('headless', [False, True])
def test_no_sum_omits_all_function_servers_but_preserves_navigation(tmp_path, headless):
    (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').touch()
    commands = boot.build_commands(tmp_path, 3, no_sum=True, headless=headless)
    assert [title for title, _ in commands] == ['Micro ROS Agent', 'Navigation2', 'Odometry']
    assert any(command.endswith('ros2 launch fishbot_navigation2 navigation2.launch.py') for _, command in commands)
    assert all('framework' not in command and 'SELECTED_POSE=3' in command for _, command in commands)


def test_default_boot_opens_sum_in_its_own_terminal(tmp_path, monkeypatch):
    (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').touch()
    monkeypatch.setattr(boot, 'find_workspace', lambda _: tmp_path)
    monkeypatch.setattr(boot.shutil, 'which', lambda _: '/usr/bin/gnome-terminal')
    launches = []
    monkeypatch.setattr(boot.subprocess, 'Popen', lambda argv, **kwargs: launches.append(argv))
    boot.main(['--selected-pose', '2'])
    assert len(launches) == 4
    function_terminal = next(argv for argv in launches if '--title=Function sum' in argv)
    assert function_terminal[:1] == ['gnome-terminal']
    assert function_terminal[-1].endswith('exec ros2 run framework sum')
    assert all('export SELECTED_POSE=2' in argv[-1] for argv in launches)


def test_no_sum_cli_does_not_launch_function_terminal(tmp_path, monkeypatch):
    (tmp_path / 'install').mkdir()
    (tmp_path / 'install/setup.bash').touch()
    monkeypatch.setattr(boot, 'find_workspace', lambda _: tmp_path)
    monkeypatch.setattr(boot.shutil, 'which', lambda _: '/usr/bin/gnome-terminal')
    launches = []
    monkeypatch.setattr(boot.subprocess, 'Popen', lambda argv, **kwargs: launches.append(argv))
    boot.main(['--selected-pose', '4', '--no-sum', '--no-micro-ros'])
    assert len(launches) == 2
    assert all('framework' not in argv[-1] for argv in launches)
