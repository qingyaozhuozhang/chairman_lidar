"""Exercise shared motion functions without creating ROS nodes or moving hardware."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from rclpy.time import Time

from chairman_tasks.config import load_special_parameters
from chairman_tasks.special.move_forward import task as translation
from chairman_tasks.special.turn_left import task as rotation
from chairman_tasks.special.stair_forward import task as stairs
from chairman_tasks.special.uphill import task as uphill


class Robot:
    def __init__(self, yaw=0.0):
        root = Path(__file__).parents[2] / 'src/function/config'
        for name, value in load_special_parameters(root).items():
            setattr(self, name, value)
        self.current_x = self.current_y = 0.0
        self.current_yaw = yaw
        self.cancel_current_task = False
        self.commands = []
        self.failures = []
        self.task_progress = SimpleNamespace(update=lambda _: None, fail=self.failures.append)
        self.ticks = 0

    def get_clock(self):
        def now():
            self.ticks += 1
            return Time(nanoseconds=self.ticks * 20_000_000)
        return SimpleNamespace(now=now)

    def wait_for_odom(self, **kwargs):
        return True

    def get_current_map_pose(self, **kwargs):
        return self.current_x, self.current_y, self.current_yaw

    def flush_odom_queue(self):
        pass

    def publish_twist_speed(self, command):
        self.commands.append((command.linear.x, command.linear.y, command.angular.z))

    def publish_manual_zero_speed(self):
        self.commands.append((0.0, 0.0, 0.0))


@pytest.fixture(autouse=True)
def ros_active(monkeypatch):
    monkeypatch.setattr(translation.rclpy, 'ok', lambda: True)


@pytest.mark.parametrize('direction', [(1.0, 0.0), (0.0, -1.0), (0.6, 0.8)])
def test_shared_translation_preserves_body_direction_and_stops(direction, monkeypatch):
    robot = Robot(yaw=math.pi / 2)
    x, y = direction

    def arrive(_):
        robot.current_x, robot.current_y = -y, x

    monkeypatch.setattr(translation.time, 'sleep', arrive)
    assert translation.move(robot, x, y, 1.0)
    vx, vy, omega = robot.commands[0]
    assert vx * x + vy * y > 0.0
    assert omega == pytest.approx(0.0)
    assert robot.commands[-1] == (0.0, 0.0, 0.0)
    assert not robot.failures


@pytest.mark.parametrize('angle', [90.0, -90.0, 180.0])
def test_shared_rotation_preserves_direction_and_stops(angle, monkeypatch):
    robot = Robot()
    monkeypatch.setattr(rotation.time, 'sleep', lambda _: setattr(robot, 'current_yaw', math.radians(angle)))
    assert rotation.rotate(robot, angle)
    vx, vy, omega = robot.commands[0]
    assert (vx, vy) == (0.0, 0.0)
    assert omega * angle > 0.0
    assert robot.commands[-1] == (0.0, 0.0, 0.0)
    assert not robot.failures


@pytest.mark.parametrize('direction', [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0)])
def test_shared_stair_timer_preserves_direction_and_obeys_stop_signal(direction):
    robot = Robot(yaw=math.pi / 2)
    x, y = direction
    assert stairs.start_corrected_stair_mode(robot, x, y, 0.3, 'test')
    stairs.stair_timer_callback(robot)
    vx, vy, omega = robot.commands[-1]
    assert vx * x + vy * y > 0.0
    assert omega == pytest.approx(0.0)
    robot.stair_cmd = 0
    assert stairs.wait_stair_stop_signal(robot, 'test')
    assert not robot.in_stair_mode
    assert robot.commands[-1] == (0.0, 0.0, 0.0)
    assert not robot.failures


def test_uphill_uses_map_x_and_stops(monkeypatch):
    robot = Robot(yaw=math.pi / 2)
    monkeypatch.setattr(uphill.time, 'sleep', lambda _: setattr(robot, 'current_x', 1.0))
    assert uphill.climb(robot, 1.0)
    vx, vy, omega = robot.commands[0]
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy < 0.0
    assert omega == pytest.approx(0.0)
    assert robot.commands[-1] == (0.0, 0.0, 0.0)
    assert not robot.failures


@pytest.mark.parametrize('operation', [
    lambda node: translation.move(node, 1.0, 0.0, 1.0),
    lambda node: rotation.rotate(node, 90.0),
    lambda node: uphill.climb(node, 1.0),
])
def test_cancelled_motion_emits_no_nonzero_speed(operation):
    robot = Robot()
    robot.cancel_current_task = True
    assert not operation(robot)
    assert robot.commands and all(command == (0.0, 0.0, 0.0) for command in robot.commands)
