#!/usr/bin/env python3
"""ROS2 entry point for the original odometry transform launch."""
import os
import sys


def main():
    os.execvp('ros2', ['ros2', 'launch', 'odometry', 'odometry.launch.py', *sys.argv[1:]])


if __name__ == '__main__':
    sys.exit(main())
