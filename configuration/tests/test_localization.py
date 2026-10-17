"""Startup poses enter GICP in map coordinates; a single node owns map -> odom."""
import importlib.util
import math
from pathlib import Path

from launch.actions import OpaqueFunction
import pytest
import yaml

ROOT = Path(__file__).parents[2]


def load_launch(relative):
    spec = importlib.util.spec_from_file_location('localization_launch', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def localization_launch(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    module = load_launch('src/fishbot/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py')
    config = tmp_path / 'odometry/config'
    config.mkdir(parents=True)
    (config / 'initial_poses.yaml').write_text(yaml.safe_dump({
        'mapping_pose': {'x': -5.0, 'y': -1.0, 'z': 0.2,
                         'roll': 0.03, 'pitch': -0.04, 'yaw': -math.pi / 2},
        'poses': {
        1: {'lidar_init_x': -5.0, 'lidar_init_y': -1.0, 'lidar_init_qz': -1.0, 'lidar_init_qw': 1.0},
        2: {'lidar_init_x': 6.0, 'lidar_init_y': -4.0, 'lidar_init_qz': 1.0, 'lidar_init_qw': 1.0},
        3: {'lidar_init_x': -5.0, 'lidar_init_y': 1.0, 'lidar_init_qz': 1.0, 'lidar_init_qw': 1.0},
        4: {'lidar_init_x': 6.0, 'lidar_init_y': 4.0, 'lidar_init_qz': -1.0, 'lidar_init_qw': 1.0}}}))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(tmp_path / package), raising=False)
    calls = []

    def node(**kwargs):
        calls.append(kwargs)
        return OpaqueFunction(function=lambda context: [])

    monkeypatch.setattr(module, 'Node', node)
    return module, calls, config / 'initial_poses.yaml'


@pytest.mark.parametrize('selected,expected', [
    (1, [-5.0, -1.0, 0.0, 0.0, 0.0, -math.pi / 2]),
    (2, [6.0, -4.0, 0.0, 0.0, 0.0, math.pi / 2]),
    (3, [-5.0, 1.0, 0.0, 0.0, 0.0, math.pi / 2]),
    (4, [6.0, 4.0, 0.0, 0.0, 0.0, -math.pi / 2]),
])
def test_selected_start_is_loaded_from_shared_yaml(localization_launch, monkeypatch, selected, expected):
    module, calls, _ = localization_launch
    monkeypatch.setenv('SELECTED_POSE', str(selected))
    module.generate_launch_description()
    assert len(calls) == 1
    assert calls[0]['executable'] == 'small_gicp_relocalization_node'
    assert all(isinstance(item, dict) for item in calls[0]['parameters'])
    parameters = {}
    for item in calls[0]['parameters']:
        if isinstance(item, dict):
            parameters.update(item)
    assert parameters['initial_map_pose'] == pytest.approx(expected)
    assert parameters['map_to_pcd'] == pytest.approx([-5.0, -1.0, 0.2, 0.03, -0.04, -math.pi / 2])


def test_mapping_origin_is_loaded_independently_of_selected_start(localization_launch, monkeypatch):
    module, calls, path = localization_launch
    monkeypatch.setenv('SELECTED_POSE', '2')
    values = yaml.safe_load(path.read_text())
    values['mapping_pose'].update(x=3.0, y=4.0, yaw=0.7)
    path.write_text(yaml.safe_dump(values))
    module.generate_launch_description()
    parameters = calls[0]['parameters'][-1]
    assert parameters['map_to_pcd'] == pytest.approx([3.0, 4.0, 0.2, 0.03, -0.04, 0.7])
    assert parameters['initial_map_pose'] == pytest.approx([6.0, -4.0, 0.0, 0.0, 0.0, math.pi / 2])


@pytest.mark.parametrize('invalid', [None, {}, [],
    {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': float('nan')},
    {'x': float('inf'), 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
    {'x': 'invalid', 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
    {'x': True, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
])
def test_invalid_mapping_origin_fails_before_launch(localization_launch, monkeypatch, invalid):
    module, calls, path = localization_launch
    monkeypatch.setenv('SELECTED_POSE', '1')
    values = yaml.safe_load(path.read_text())
    if invalid is None:
        del values['mapping_pose']
    else:
        values['mapping_pose'] = invalid
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match='mapping_pose'):
        module.generate_launch_description()
    assert not calls


@pytest.mark.parametrize('invalid', [float('nan'), 'not a number'])
def test_invalid_start_position_fails_before_launch(localization_launch, monkeypatch, invalid):
    module, calls, path = localization_launch
    monkeypatch.setenv('SELECTED_POSE', '1')
    values = yaml.safe_load(path.read_text())
    values['poses'][1]['lidar_init_x'] = invalid
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError):
        module.generate_launch_description()
    assert not calls


def test_zero_quaternion_cannot_seed_localization(localization_launch, monkeypatch):
    module, _, path = localization_launch
    monkeypatch.setenv('SELECTED_POSE', '1')
    values = yaml.safe_load(path.read_text())
    values['poses'][1].update(lidar_init_qz=0.0, lidar_init_qw=0.0)
    path.write_text(yaml.safe_dump(values))
    with pytest.raises(ValueError, match='四元数'):
        module.generate_launch_description()


def test_navigation_includes_one_localizer_with_map_and_clock(tmp_path, monkeypatch):
    from launch import LaunchContext
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    module = load_launch('src/fishbot/fishbot_navigation2/launch/navigation2.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(tmp_path / package))
    nodes, includes = [], []

    def node(**kwargs):
        nodes.append(kwargs)
        return OpaqueFunction(function=lambda context: [])

    def include(source, **kwargs):
        includes.append((source, dict(kwargs.get('launch_arguments', []))))
        return OpaqueFunction(function=lambda context: [])

    monkeypatch.setattr(module.launch_ros.actions, 'Node', node)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', include)
    monkeypatch.setattr(module, 'PythonLaunchDescriptionSource', lambda parts: ''.join(parts))
    module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = 'false'
    localizers = [args for source, args in includes
                  if source.endswith('/small_gicp_relocalization_launch.py')]
    assert len(localizers) == 1
    assert localizers[0]['prior_pcd_file'] == str(tmp_path / 'fishbot_navigation2/PCD/final.pcd')
    assert localizers[0]['use_sim_time'].perform(context) == 'false'
    assert not any(n['executable'] in ('tf_relay_node', 'small_gicp_relocalization_node') for n in nodes)


def test_odometry_does_not_overwrite_localizer_startup_pose(localization_launch, monkeypatch):
    shared, calls, _ = localization_launch
    module = load_launch('src/function/detail/continuous/odometry/launch/odometry.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', shared.get_package_share_directory)
    monkeypatch.setattr(module, 'Node', shared.Node)
    monkeypatch.setenv('SELECTED_POSE', '4')
    module.generate_launch_description()
    parameters = calls[0]['parameters'][0]
    assert parameters['selected_pose'] == 4
    assert parameters['initialpose_publish_after_stable_count'] == 0
