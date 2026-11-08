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



def test_odometry_launch_starts_only_odometry_without_a_points_file(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location(
        'cleanup_odom_launch', ROOT / 'src/function/detail/continuous/odometry/launch/odometry.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    config = tmp_path / 'config/initial_poses.yaml'
    config.parent.mkdir()
    config.write_text('poses: {1: {lidar_init_x: 1.5, lidar_init_y: 2.5}}\n')
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda _: str(tmp_path))
    monkeypatch.setenv('SELECTED_POSE', '1')
    actions = launch.generate_launch_description().entities
    assert len(actions) == 1
    assert isinstance(actions[0], Node)
    assert actions[0].node_executable == 'odometry_transform_math'
