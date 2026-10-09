import importlib.util
from pathlib import Path
import shutil

import pytest
import yaml

ROOT = Path(__file__).parents[2] / 'src/function'
spec = importlib.util.spec_from_file_location('catalog', ROOT / 'detail/on_demand/config.py')
catalog = importlib.util.module_from_spec(spec)
spec.loader.exec_module(catalog)


@pytest.fixture
def config(tmp_path):
    shutil.copytree(ROOT / 'config', tmp_path / 'config')
    return tmp_path / 'config'


def test_new_speed_mode_and_point_discovered_without_python_edits(config):
    (config / 'on_demand/modes/precise.yaml').write_text('id: 4\nname: precise\nbase_mode: 1\nparameters: {}\n')
    path = config / 'on_demand/points.yaml'
    points = yaml.safe_load(path.read_text())
    points['red'][33] = {'name': 'new point', 'pose': [1.0, 2.0, 0.0, 1.0], 'mode': 4}
    path.write_text(yaml.safe_dump(points))
    modes = catalog.load_modes(config)
    red, blue = catalog.load_points(modes, config)
    assert red[33] == [1.0, 2.0, 0.0, 1.0, 'new point', 4]
    assert len(blue) == 33


def test_duplicate_mode_rejected(config):
    directory = config / 'on_demand/modes'
    (directory / 'duplicate.yaml').write_text((directory / 'speed_1.yaml').read_text())
    with pytest.raises(ValueError, match='重复'):
        catalog.load_modes(config)


def test_unknown_mode_in_point_rejected(config):
    p = config / 'on_demand/points.yaml'
    data = yaml.safe_load(p.read_text())
    data['red'][1]['mode'] = 99
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='模式'):
        catalog.load_points(catalog.load_modes(config), config)


def test_new_special_task_registration_and_id_range(config):
    p = config / 'on_demand/functions.yaml'
    data = yaml.safe_load(p.read_text())
    data['functions'].append({'id': -12, 'name': 'new_task', 'module': 'special.new_task.task', 'parameters': {'angle': 45.0}})
    p.write_text(yaml.safe_dump(data))
    red, blue = catalog.load_points(catalog.load_modes(config), config)
    tasks = catalog.load_tasks(red, blue, config)
    assert tasks[-12]['parameters']['angle'] == 45.0
    data['functions'][-1]['id'] = -129
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='int8'):
        catalog.load_tasks(red, blue, config)


def test_accidental_task_point_collision_rejected(config):
    p = config / 'on_demand/functions.yaml'
    data = yaml.safe_load(p.read_text())
    data['functions'].append({'id': 1, 'name': 'collision', 'module': 'special.collision.task'})
    p.write_text(yaml.safe_dump(data))
    red, blue = catalog.load_points(catalog.load_modes(config), config)
    with pytest.raises(ValueError, match='冲突'):
        catalog.load_tasks(red, blue, config)


def test_duplicate_point_yaml_key_rejected(config):
    p = config / 'on_demand/points.yaml'
    p.write_text('red:\n  33: {name: first}\n  33: {name: second}\nblue: {}\n')
    with pytest.raises(ValueError, match='键重复'):
        catalog.load_points(catalog.load_modes(config), config)


def test_field_is_read_without_prompt_and_matches_odometry_default(monkeypatch):
    monkeypatch.delenv('SELECTED_POSE', raising=False)
    assert catalog.read_selected_pose() == 1
    monkeypatch.setenv('SELECTED_POSE', '4')
    assert catalog.read_selected_pose() == 4
    monkeypatch.setenv('SELECTED_POSE', 'invalid')
    with pytest.raises(ValueError, match='main_boot'):
        catalog.read_selected_pose()


def test_all_mode_parameters_refer_to_existing_nav2_baseline_names():
    nav2_path = ROOT.parent / 'fishbot/fishbot_navigation2/config/nav2_params.yaml'
    nav2 = yaml.safe_load(nav2_path.read_text())
    modes = catalog.load_modes(ROOT / 'config')
    for mode in modes.values():
        for section in ('parameters', 'sprint_parameters'):
            parameters = catalog.flatten_parameters(mode.get(section, {}))
            for node, values in parameters.items():
                baseline = catalog.flatten_parameters({node: nav2[node]})[node]
                assert values.keys() <= baseline.keys()
                assert all(type(value) is type(baseline[name]) for name, value in values.items())


def test_native_parameter_paths_reject_collisions():
    with pytest.raises(ValueError, match='重复参数路径'):
        catalog.flatten_parameters({'controller_server': {'ros__parameters': {
            'FollowPath': {'translation_kp': 1.0}, 'FollowPath.translation_kp': 2.0}}})


def test_mode_directory_contains_complete_named_blocks_and_special_is_not_a_speed(config):
    directory = config / 'on_demand/modes'
    assert {p.name for p in directory.glob('*.yaml')} == {
        'speed_1.yaml', 'speed_2.yaml', 'speed_3.yaml', 'special.yaml'}
    baseline = yaml.safe_load((ROOT.parent / 'fishbot/fishbot_navigation2/config/nav2_params.yaml').read_text())
    modes = catalog.load_modes(config)
    assert set(modes) == {1, 2, 3}
    for mode in modes.values():
        for section in ('parameters', 'sprint_parameters'):
            for node, params in catalog.flatten_parameters(mode.get(section, {})).items():
                assert params.keys() == catalog.flatten_parameters({node: baseline[node]})[node].keys()
    assert not (config / 'on_demand/special.yaml').exists()


def test_startup_fields_are_visible_but_not_sent_during_speed_switch():
    values = catalog.dynamic_parameters({
        'controller_server': {'ros__parameters': {
            'use_sim_time': True, 'controller_frequency': 20.0,
            'controller_plugins': ['FollowPath'],
            'FollowPath': {'plugin': 'example', 'translation_kp': 4.0, 'lookahead_dist': 0.3},
        }},
        'velocity_smoother': {'ros__parameters': {
            'use_sim_time': True, 'scale_velocities': False,
            'max_velocity': [1.0, 1.0, 2.0], 'odom_topic': 'odom',
            'smoothing_frequency': 20.0, 'feedback': 'OPEN_LOOP',
        }},
    })
    assert values['controller_server'] == {'FollowPath.translation_kp': 4.0, 'FollowPath.lookahead_dist': 0.3}
    assert values['velocity_smoother'] == {
        'max_velocity': [1.0, 1.0, 2.0], 'odom_topic': 'odom',
        'smoothing_frequency': 20.0, 'feedback': 'OPEN_LOOP'}


def test_special_uses_native_names_and_preserves_control_values(config):
    special = catalog.read_yaml(config / 'on_demand/modes/special.yaml')
    native = catalog.flatten_parameters(special['parameters'])
    assert native['controller_server']['FollowPath.translation_kp'] == 0.5
    assert native['controller_server']['FollowPath.rotation_kp'] == 3.0
    assert native['controller_server']['general_goal_checker.xy_goal_tolerance'] == 0.005
    assert native['velocity_smoother']['max_decel'] == [-0.5, -0.5, -3.0]
    assert not {'kp_linear', 'max_vel_linear', 'error_tolerance_dist'} & native['simple_nav_node'].keys()
    values = catalog.load_special_parameters(config)
    assert values == {
        'MAX_ACCEL': [0.5, 0.5, 3.0], 'MAX_DECEL': [0.5, 0.5, 3.0],
        'MAX_VEL_LINEAR': 0.5, 'MIN_VEL_LINEAR': 0.05, 'KP_LINEAR': 0.5,
        'ERROR_TOLERANCE_DIST': 0.005, 'MAX_VEL_ANGULAR': 1.5,
        'MIN_VEL_ANGULAR': 0.1, 'KP_ANGULAR': 3.0, 'KP_YAW_CORRECT': 1.0,
        'YAW_CORRECT_MAX_RATIO': 0.5, 'ERROR_TOLERANCE_YAW': 0.01,
        'KP_CROSS_TRACK': 1.0, 'MAX_CROSS_TRACK_VEL': 0.1,
        'MIN_CROSS_TRACK_VEL': 0.01, 'ERROR_TOLERANCE_CROSS': 0.005,
        'CONTROL_PERIOD': 0.02, 'LIFT_CHECK_HEIGHT': 0.5,
        'UPHILL_GLOBAL_X_DISTANCE': 2.7, 'UPHILL_MAX_ACCEL': [1.0, 1.0, 1.0],
        'UPHILL_MAX_DECEL': [2.0, 2.0, 1.0], 'UPHILL_MAX_VEL_LINEAR': 2.0,
        'UPHILL_MIN_VEL_LINEAR': 0.05, 'UPHILL_KP_LINEAR': 2.0,
        'UPHILL_ERROR_TOLERANCE_DIST': 0.01, 'UPHILL_MAX_VEL_ANGULAR': 1.0,
        'UPHILL_MIN_VEL_ANGULAR': 0.05, 'UPHILL_KP_YAW_CORRECT': 1.0,
        'UPHILL_YAW_CORRECT_MAX_RATIO': 0.5, 'UPHILL_ERROR_TOLERANCE_YAW': 0.01,
        'UPHILL_KP_CROSS_TRACK': 1.5, 'UPHILL_MAX_CROSS_TRACK_VEL': 1.0,
        'UPHILL_MIN_CROSS_TRACK_VEL': 0.05, 'UPHILL_ERROR_TOLERANCE_CROSS': 0.01,
        'UPHILL_GLOBAL_X_TIMEOUT': 20.0,
    }


def test_special_parameter_edits_and_ros_overrides_reach_manual_controller(config):
    from types import SimpleNamespace
    path = config / 'on_demand/modes/special.yaml'
    data = catalog.read_yaml(path)
    data['parameters']['controller_server']['ros__parameters']['FollowPath']['translation_kp'] = 0.8
    path.write_text(yaml.safe_dump(data))
    declared = {}
    node = SimpleNamespace(
        declare_parameter=lambda name, value: declared.setdefault(name, value),
        get_parameter=lambda name: SimpleNamespace(value=100.0 if name == 'controller_server.controller_frequency' else declared[name]),
    )
    values = catalog.load_special_parameters(config, node=node)
    assert values['KP_LINEAR'] == 0.8
    assert values['CONTROL_PERIOD'] == 0.01
    assert 'controller_server.FollowPath.translation_kp' in declared
    assert 'kp_linear' not in declared


def test_special_has_complete_native_blocks_for_both_motion_groups(config):
    special = catalog.read_yaml(config / 'on_demand/modes/special.yaml')
    baseline = catalog.read_yaml(ROOT.parent / 'fishbot/fishbot_navigation2/config/nav2_params.yaml')
    for section in ('parameters', 'uphill_parameters'):
        native = catalog.flatten_parameters(special[section])
        for node in ('controller_server', 'velocity_smoother'):
            expected = catalog.flatten_parameters({node: baseline[node]})[node]
            assert native[node].keys() == expected.keys()
            assert all(type(value) is type(expected[name]) for name, value in native[node].items())


@pytest.mark.parametrize('section,node,path,value', [
    ('parameters', 'controller_server', ['controller_frequency'], 0.0),
    ('parameters', 'controller_server', ['FollowPath', 'translation_kp'], float('nan')),
    ('parameters', 'velocity_smoother', ['max_accel'], [0.5, 0.5]),
    ('parameters', 'velocity_smoother', ['max_decel'], [0.5, 0.5, 3.0]),
    ('uphill_parameters', 'velocity_smoother', ['max_decel'], [-2.0, -2.0, 0.0]),
])
def test_invalid_special_control_values_are_rejected(config, section, node, path, value):
    source = config / 'on_demand/modes/special.yaml'
    data = catalog.read_yaml(source)
    target = data[section][node]['ros__parameters']
    for name in path[:-1]:
        target = target[name]
    target[path[-1]] = value
    source.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        catalog.load_special_parameters(config)
