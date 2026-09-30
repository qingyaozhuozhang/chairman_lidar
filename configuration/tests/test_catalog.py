import importlib.util
from pathlib import Path
import shutil

import pytest
import yaml

ROOT = Path(__file__).parents[2] / 'src/function'
spec = importlib.util.spec_from_file_location('catalog', ROOT / 'framework/core/catalog.py')
catalog = importlib.util.module_from_spec(spec)
spec.loader.exec_module(catalog)


@pytest.fixture
def config(tmp_path):
    shutil.copytree(ROOT / 'config', tmp_path / 'config')
    return tmp_path / 'config'


def test_new_speed_mode_and_point_discovered_without_python_edits(config):
    (config / 'on_demand/modes/precise.yaml').write_text('id: 4\nname: precise\nbase_mode: 1\noverrides: {}\n')
    path = config / 'on_demand/points.yaml'
    points = yaml.safe_load(path.read_text())
    points['red'][33] = {'name': 'new point', 'pose': [1.0, 2.0, 0.0, 1.0], 'mode': 4}
    path.write_text(yaml.safe_dump(points))
    modes = catalog.load_modes(config)
    red, blue = catalog.load_points(modes, config)
    assert red[33] == [1.0, 2.0, 0.0, 1.0, 'new point', 4]
    assert len(blue) == 33


def test_duplicate_mode_rejected(config):
    (config / 'on_demand/modes/duplicate.yaml').write_text('id: 1\nname: duplicate\nbase_mode: 1\n')
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
