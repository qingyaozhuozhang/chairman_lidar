#!/usr/bin/env python3
"""odometry 命令入口：启动 odometry.launch.py 并转发命令行参数。"""
import os
import sys


def main():
    os.execvp('ros2', ['ros2', 'launch', 'odometry', 'odometry.launch.py', *sys.argv[1:]])


if __name__ == '__main__':
    sys.exit(main())
