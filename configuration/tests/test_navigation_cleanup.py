"""Region-free calibration and independent direct-navigation mode checks."""
from pathlib import Path
import importlib.util
import sys
import yaml

ROOT = Path(__file__).parents[2]


def test_calibration_workspace_without_regions(tmp_path):
    spec = importlib.util.spec_from_file_location('cleanup_point', ROOT / 'tool/point.py')
    point = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = point
    spec.loader.exec_module(point)
    for relative in (point.POINTS_PATH, point.POINTS_COPY):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('red: {14: {name: test, pose: [0, 0, 0, 1], mode: 1}}\nblue: {}\n')
    assert point.find_workspace(tmp_path) == tmp_path
    paths = point.calibration_paths(tmp_path, 'point')
    point.commit_updates({p: point.prepare_point_xy(p, 'red', 14, 2.25, -1.5) for p in paths})
    assert all(yaml.safe_load(p.read_text())['red'][14]['pose'] == [2.25, -1.5, 0, 1] for p in paths)


def test_slow_mode_loads_axis_parameters_and_only_two_modes():
    from chairman_tasks.config import load_modes, dynamic_parameters
    modes = load_modes(ROOT / 'src/function/config')
    assert set(modes) == {1, 2}
    values = dynamic_parameters(modes[2]['parameters'])['controller_server']
    assert values['FollowPath.translation_kpx'] == 2.0
    assert values['FollowPath.v_linear_maxx'] == 0.5


import os
import subprocess
import time
import pytest


@pytest.mark.skipif(os.environ.get('CHAIRMAN_ROS_INTEGRATION') != '1', reason='isolated ROS runtime')
def test_marker_colors_frame_lock_reload_and_latched_delivery(tmp_path):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray
    assert os.environ.get('ROS_DOMAIN_ID') == '197'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    points = tmp_path / 'points.yaml'
    data = {'red': {14: {'pose': [1.25, -2.5, 0, 1]}, 16: {'pose': [None]*4}},
            'blue': {14: {'pose': [-3.0, 4.5, 0, 1]}}}
    points.write_text(yaml.safe_dump(data))
    log = (tmp_path / 'visualizer.log').open('w+')
    process = subprocess.Popen(['ros2','run','odometry','points_visualizer','--ros-args',
                                '-p',f'points_file:={points}'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('test_point_markers')
    frames = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(MarkerArray, '/navigation_points', frames.append, qos)
    def until(predicate):
        deadline = time.monotonic()+7
        while not predicate():
            rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() is not None:
                log.seek(0)
                pytest.fail(log.read())
            assert time.monotonic()<deadline, 'No matching marker snapshot'
    def additions():
        return [m for m in frames[-1].markers if m.action == Marker.ADD]
    try:
        until(lambda:bool(frames))
        dots = [m for m in additions() if m.type == Marker.SPHERE]
        assert len(dots) == 2
        red = next(m for m in dots if m.color.r == 1.0)
        blue = next(m for m in dots if m.color.b == 1.0)
        assert red.ns != blue.ns and red.id == blue.id == 14
        assert red.color.b == 0 and blue.color.r == 0
        assert (red.pose.position.x,red.pose.position.y) == (1.25,-2.5)
        assert (blue.pose.position.x,blue.pose.position.y) == (-3.0,4.5)
        assert all(m.header.frame_id == 'map' and m.frame_locked for m in additions())
        assert len(additions()) == 4
        assert {m.text for m in additions() if m.type == Marker.TEXT_VIEW_FACING} == {'R14','B14'}
        data['red'][14]['pose'][0] = 2.0
        del data['blue'][14]
        temporary = points.with_suffix('.tmp')
        temporary.write_text(yaml.safe_dump(data));temporary.replace(points)
        until(lambda:any(m.type == Marker.SPHERE and m.pose.position.x == 2.0 for m in additions()))
        assert len(additions()) == 2
        assert any(m.action == Marker.DELETEALL for m in frames[-1].markers)
        node.destroy_subscription(sub);frames.clear()
        sub = node.create_subscription(MarkerArray, '/navigation_points', frames.append, qos)
        until(lambda:bool(frames))
        assert any(m.type == Marker.SPHERE and m.pose.position.x == 2.0 for m in additions())
        points.write_text('red: [')
        time.sleep(1.3)
        node.destroy_subscription(sub);frames.clear()
        sub = node.create_subscription(MarkerArray, '/navigation_points', frames.append, qos)
        until(lambda:bool(frames))
        assert any(m.type == Marker.SPHERE and m.pose.position.x == 2.0 for m in additions())
        points.write_text('red: {}\nblue: {}\n')
        until(lambda:all(m.action == Marker.DELETEALL for m in frames[-1].markers))
    finally:
        process.terminate()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill();process.wait()
        node.destroy_node();rclpy.shutdown();log.close()


@pytest.mark.parametrize('merged', [False, True])
def test_points_path_prefers_source_then_same_install_tree(tmp_path, monkeypatch, merged):
    spec = importlib.util.spec_from_file_location('cleanup_odom_launch', ROOT / 'src/function/detail/continuous/odometry/launch/odometry.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    share = tmp_path / ('install/share/odometry' if merged else 'install/odometry/share/odometry')
    installed = tmp_path / ('install/share/framework/config/on_demand/points.yaml' if merged else 'install/framework/share/framework/config/on_demand/points.yaml')
    source = tmp_path / 'src/function/config/on_demand/points.yaml'
    share.mkdir(parents=True)
    for path in (source, installed):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('red: {}\nblue: {}\n')
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda _: str(share))
    assert launch.points_file_path() == str(source)
    source.unlink()
    assert launch.points_file_path() == str(installed)
    installed.unlink()
    assert launch.points_file_path() == str(installed), 'Missing display config must not prevent odometry launch'
