#!/usr/bin/env python3
"""持续运行 micro-ROS Agent；连接参数由本包 config/boot.yaml 提供。"""
import os
from pathlib import Path
import shlex
import sys

import yaml


def find_workspace():
    for start in (Path(__file__).resolve(), Path.cwd()):
        for parent in (start, *start.parents):
            if (parent / 'src/config/manifest.json').is_file() and (parent / 'configuration/main_boot.py').is_file():
                return parent
    raise ValueError('找不到项目根目录；请从工程目录运行')


def agent_command(config, root=None):
    agent = ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial',
             '--dev', str(config.get('device', '/dev/ttyUSB0')),
             '-b', str(int(config.get('baud', 921600)))]
    workspace = str(config.get('workspace', '')).strip()
    if not workspace:
        return agent
    path = Path(workspace)
    if path.is_absolute():
        raise ValueError('workspace 请填写相对于项目根目录的路径，例如 ../uros_ws；使用本项目 Agent 时留空')
    workspace_path = (root or find_workspace()) / path
    setup = workspace_path / 'install/setup.bash'
    if not setup.is_file():
        raise ValueError(
            f'micro-ROS 工作空间未构建：{setup}；'
            '使用本项目 Agent 时请将 config/boot.yaml 的 workspace 留空，'
            '在项目根目录运行 colcon build 并 source install/setup.bash；'
            '使用外部 Agent 时请先构建对应工作空间'
        )
    # Replace this process so signals go straight to the ROS command.
    return ['bash', '-c', f'source {shlex.quote(str(setup))} && exec {shlex.join(agent)}']


def main():
    from ament_index_python.packages import get_package_share_directory
    try:
        config_path = Path(get_package_share_directory('micro_ros')) / 'config/boot.yaml'
        config = yaml.safe_load(config_path.read_text())
        command = agent_command(config)
        os.execvp(command[0], command)
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        sys.exit(f'micro-ROS 启动失败：{exc}')


if __name__ == '__main__':
    main()
