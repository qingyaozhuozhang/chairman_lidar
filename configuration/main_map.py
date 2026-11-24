"""独立启动 FAST-LIO 建图；Ctrl+C 保存原始地图后生成导航 final.pcd。"""
import argparse
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time


def find_workspace(explicit=None):
    """Resolve both source and installed entry points after moving the workspace."""
    candidates = [Path(explicit).resolve()] if explicit else [Path(__file__).resolve(), Path.cwd()]
    for start in candidates:
        for path in (start, *start.parents):
            if ((path / 'configuration/main_map.py').is_file()
                    and (path / 'src/fishbot/fast_lio').is_dir()):
                return path.resolve()
    raise ValueError('找不到工作空间，请使用 --workspace 指定 chairman_lidar 目录')


def _fingerprint(path):
    try:
        stat = path.stat()
        return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns
    except FileNotFoundError:
        return None


def _group_alive(group):
    # An exited ros2 launcher can leave children behind. Ignore zombies, which
    # cannot save a map and may be reaped by a different parent/container init.
    proc = Path('/proc')
    if proc.is_dir():
        for entry in proc.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == group and fields[0] != 'Z':
                    return True
            except (OSError, ValueError, IndexError):
                continue
        return False
    try:
        os.killpg(group, 0)
        return True
    except ProcessLookupError:
        return False


def _stop_mapping(process, timeout):
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + timeout
    while _group_alive(process.pid):
        process.poll()
        if time.monotonic() >= deadline:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
            raise RuntimeError('等待 FAST-LIO 保存超时，未更新 final.pcd')
        time.sleep(0.05)
    return process.wait(timeout=5)


def run_mapping(command, raw_map, final_map, downsample_command, shutdown_timeout=330.0):
    """Manage one mapping session and convert only its completed, fresh output."""
    raw_map, final_map = Path(raw_map), Path(final_map)
    before = _fingerprint(raw_map)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True  # Further signals must not interrupt a map write.

    handlers = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    process = None
    try:
        process = subprocess.Popen(command, start_new_session=True)
        print('FAST-LIO 建图已启动。按 Ctrl+C 停止，等待保存及降采样完成。', flush=True)
        while process.poll() is None and not stop_requested:
            time.sleep(0.05)
        interrupted = stop_requested
        if interrupted:
            print('正在等待 FAST-LIO 保存地图，请勿关闭终端……', flush=True)
        result = _stop_mapping(process, shutdown_timeout)
        process = None
        if not interrupted:
            raise RuntimeError(f'建图进程提前退出（状态 {result}），未更新 final.pcd')
        if result not in (0, 130, -signal.SIGINT):
            raise RuntimeError(f'建图进程退出异常（状态 {result}），未更新 final.pcd')
        after = _fingerprint(raw_map)
        if after is None or after == before or raw_map.stat().st_size == 0:
            raise RuntimeError('本次建图没有生成新的 scans.pcd，未更新 final.pcd')
        final_map.parent.mkdir(parents=True, exist_ok=True)
        converted = subprocess.run(downsample_command, check=False)
        if converted.returncode:
            raise RuntimeError(f'地图降采样失败（状态 {converted.returncode}），原 final.pcd 保留')
        print(f'建图完成：{raw_map}\n导航地图：{final_map}', flush=True)
    finally:
        try:
            if process is not None:
                _stop_mapping(process, shutdown_timeout)
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', help='工程根目录，默认根据源码或安装入口自动定位')
    parser.add_argument('--no-rviz', action='store_true', help='不启动 RViz')
    parser.add_argument('--global-leaf-size', '--voxel-size', type=float, default=0.05,
                        dest='leaf_size', help='降采样体素边长，单位 m，默认 0.05')
    parser.add_argument('--dry-run', action='store_true', help='只显示命令和地图路径，不启动设备')
    options = parser.parse_args(args)
    try:
        if not math.isfinite(options.leaf_size) or options.leaf_size <= 0:
            raise ValueError('体素边长必须是正有限数')
        root = find_workspace(options.workspace)
        setup = root / 'install/setup.bash'
        raw_map = root / 'src/fishbot/fast_lio/PCD/scans.pcd'
        final_map = root / 'src/fishbot/fishbot_navigation2/PCD/final.pcd'
        launch = ['ros2', 'launch', 'fast_lio', 'start.launch.py', f'workspace:={root}',
                  'start_livox_driver:=true', 'pcd_save_en:=true', 'pcd_save_interval:=-1',
                  'save_shutdown_timeout:=300.0', f'rviz:={str(not options.no_rviz).lower()}']
        command = ['bash', '-c', f'source {shlex.quote(str(setup))} && exec {shlex.join(launch)}']
        downsample = [sys.executable, str(root / 'tool/downsample_map.py'),
                      str(raw_map), str(final_map), '--global-leaf-size', str(options.leaf_size),
                      '--crop',
                      '--crop-min-xy', '-100.0', '-100.0',
                      '--crop-max-xy', '100.0', '100.0']
        print(f'原始地图：{raw_map}\n导航地图：{final_map}\n'
              f'建图命令：{shlex.join(command)}\n降采样命令：{shlex.join(downsample)}', flush=True)
        if options.dry_run:
            return 0
        if not setup.is_file():
            raise ValueError('尚未构建工作空间：缺少 install/setup.bash')
        dependency_check = subprocess.run(
            [sys.executable, '-c', 'import numpy; import open3d'], capture_output=True, text=True,
        )
        if dependency_check.returncode:
            raise ValueError(f'{sys.executable} 无法导入 numpy/open3d，请先激活已安装它们的 Python 环境')
        run_mapping(command, raw_map, final_map, downsample)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'建图失败：{exc}', file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
