"""选择场地，启动 navigation2.launch.py、持续功能和独立终端中的 sum 菜单。"""
import argparse
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time



def group_alive(process_group):
    try:
        os.killpg(process_group, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process_group(process, timeout=30.0):
    """Allow all descendants to finish, even when their ros2 wrapper exited."""
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        process.poll()
        return
    deadline = time.monotonic() + timeout
    while group_alive(process.pid):
        process.poll()  # Reap the group leader if it has become a zombie.
        if time.monotonic() >= deadline:
            print(f'进程组 {process.pid} 未在期限内退出，终止残留进程。', flush=True)
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            break
        time.sleep(0.05)
    process.wait(timeout=3.0)


def shutdown_processes(commands, processes):
    # A second Ctrl+C must not interrupt cleanup and orphan isolated groups.
    handlers = {sig: signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
    errors = []
    try:
        # sum restores its parameters while Nav2 services are still available.
        ordered = sorted(zip(commands, processes), key=lambda item: item[0][0] != 'Function sum')
        for (title, _), process in ordered:
            try:
                stop_process_group(process)
            except (OSError, subprocess.SubprocessError) as exc:
                errors.append(f'{title}: {exc}')
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    if errors:
        raise RuntimeError('退出清理异常: ' + '; '.join(errors))


def find_workspace(explicit=None):
    candidates = [Path(explicit)] if explicit else [Path(__file__).resolve(), Path.cwd()]
    for start in candidates:
        for path in (start, *start.parents):
            if (path / 'configuration/main_boot.py').is_file() and (path / 'src/fishbot').is_dir():
                return path.resolve()
        if explicit:
            break
    raise ValueError('找不到工作空间，请使用 --workspace 指定 chairman_navigation 目录')


def build_commands(root, selected_pose, no_micro=False, no_sum=False, headless=False):
    setup = root / 'install/setup.bash'
    if not setup.is_file():
        raise ValueError('尚未构建工作空间：缺少 install/setup.bash')
    common = f'source {shlex.quote(str(setup))} && export SELECTED_POSE={selected_pose} && cd {shlex.quote(str(root / "runtime"))} && '
    commands = []
    if not no_micro:
        commands.append(('Micro ROS Agent', common + 'exec ros2 run micro_ros agent'))
    # 导航服务统一从这个 launch 进入，不在这里重复启动雷达、FAST-LIO 或 Nav2。
    commands.append(('Navigation2', common + 'exec ros2 launch fishbot_navigation2 navigation2.launch.py'))
    if not no_sum:
        entry = 'preset_nav_node' if headless else 'sum'
        commands.append(('Function sum', common + 'exec ros2 run framework ' + entry))
    commands.append(('Odometry', common + 'exec ros2 run odometry odometry'))
    return commands


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', help='工程根目录；默认自动寻找，也可使用相对路径')
    parser.add_argument('--selected-pose', type=int, choices=(1,2,3,4), help='预选场地：1红武馆/2红对抗/3蓝武馆/4蓝对抗；省略则在本终端询问')
    parser.add_argument('--no-micro-ros', action='store_true', help='跳过 micro-ROS Agent，保留其他服务')
    parser.add_argument('--no-sum', action='store_true', help='完全跳过功能菜单及其服务；仍启动导航和持续功能')
    parser.add_argument('--headless', action='store_true', help='在当前终端管理子进程；功能端改为仅服务入口，无交互菜单')
    parser.add_argument('--dry-run', action='store_true', help='只打印启动命令，不启动设备')
    options = parser.parse_args(args)
    previous_sigterm = None
    if options.headless:
        def request_shutdown(signum, frame):
            raise KeyboardInterrupt
        previous_sigterm = signal.signal(signal.SIGTERM, request_shutdown)
    try:
        root = find_workspace(options.workspace)
        selected = options.selected_pose
        while selected is None:
            answer = input('初始位置 1红武馆 / 2红对抗 / 3蓝武馆 / 4蓝对抗: ').strip()
            if answer in ('1','2','3','4'):
                selected = int(answer)
        print(f'已选择场地 {selected}；所有启动的子进程共用 SELECTED_POSE={selected}。', flush=True)
        commands = build_commands(root, selected, no_micro=options.no_micro_ros,
                                  no_sum=options.no_sum, headless=options.headless)
        for title, command in commands:
            print(f'[{title}] {command}', flush=True)
        if options.dry_run:
            return
        for subdir in ('Log','PCD'):
            (root / 'runtime' / subdir).mkdir(parents=True, exist_ok=True)
        terminal = 'gnome-terminal'
        if not options.headless and not shutil.which(terminal):
            raise ValueError(f'找不到 {terminal}，可使用 --headless')
        processes = []
        try:
            for title, command in commands:
                argv = ['bash','-c',command] if options.headless else [terminal,'--title='+title,'--','bash','-c',command]
                processes.append(subprocess.Popen(argv, start_new_session=options.headless))
            print('启动命令已下发：' + ' → '.join(title for title, _ in commands) + '。请检查各节点日志。')
            if options.headless:
                while all(p.poll() is None for p in processes):
                    time.sleep(0.2)
                raise RuntimeError('一个子进程已退出，正在停止其余子进程')
        finally:
            if options.headless:
                shutdown_processes(commands, processes)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(2, f'启动失败: {exc}\n')
    finally:
        if previous_sigterm is not None:
            signal.signal(signal.SIGTERM, previous_sigterm)
