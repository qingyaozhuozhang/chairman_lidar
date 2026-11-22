"""Offline point maps preserve source geometry and replace the previous export."""
import importlib.util
import math
from pathlib import Path
import shutil
import subprocess
import sys

from PIL import Image
import pytest
import yaml

ROOT = Path(__file__).parents[2]
SCRIPT = ROOT / 'tool/show_points.py'


@pytest.fixture
def show_points():
    assert SCRIPT.is_file(), 'The manual show_points tool has not been implemented'
    spec = importlib.util.spec_from_file_location('show_points', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_inputs(directory, *, origin=(10.0, -4.0, 0.0), points=None):
    directory.mkdir(parents=True, exist_ok=True)
    image = directory / 'original.pgm'
    Image.new('L', (40, 30), 205).save(image)
    metadata = dict(image=image.name, resolution=0.5, origin=list(origin),
                    negate=0, occupied_thresh=0.65, free_thresh=0.196)
    map_yaml = directory / 'original.yaml'
    map_yaml.write_text(yaml.safe_dump(metadata))
    points_yaml = directory / 'points.yaml'
    points_yaml.write_text(yaml.safe_dump(points or {
        'red': {1: {'pose': [12.25, -2.25, 0, 1]}},
        'blue': {2: {'pose': [21.25, 6.25, 0, 1]}},
    }))
    return points_yaml, map_yaml, image, metadata


def test_colors_row_inversion_and_metadata_preserve_the_original(show_points, tmp_path):
    points, map_yaml, source, metadata = write_inputs(tmp_path)
    original = {path: path.read_bytes() for path in (points, map_yaml, source)}
    result = show_points.render_points_map(points, map_yaml, tmp_path / 'export')
    with Image.open(result['image']) as rendered:
        assert rendered.size == (40, 30)
        assert rendered.mode == 'RGB'
        assert rendered.getpixel((4, 26)) == (255, 0, 0)
        assert rendered.getpixel((22, 9)) == (0, 0, 255)
        assert rendered.getpixel((0, 0)) == (205, 205, 205)
    assert result['counts'] == {'red': 1, 'blue': 1}
    assert yaml.safe_load(result['yaml'].read_text()) == dict(metadata, image=result['image'].name)
    assert all(path.read_bytes() == data for path, data in original.items())


def test_rotated_map_origin_is_inverted_before_projection(show_points, tmp_path):
    # A +90° map yaw maps local (2.25, 1.75) to world (8.25, -1.75).
    points, map_yaml, _, _ = write_inputs(tmp_path, origin=(10, -4, math.pi / 2), points={
        'red': {1: {'pose': [8.25, -1.75, 0, 1]}}, 'blue': {},
    })
    result = show_points.render_points_map(points, map_yaml, tmp_path / 'export')
    with Image.open(result['image']) as rendered:
        assert rendered.getpixel((4, 26)) == (255, 0, 0)
        assert rendered.getpixel((20, 20)) == (205, 205, 205)


def test_invalid_dynamic_and_outside_points_are_reported_without_wrapping(show_points, tmp_path):
    points, map_yaml, _, _ = write_inputs(tmp_path, origin=(0, 0, 0), points={
        'red': {0: {'pose': [None] * 4}, 1: {'pose': []},
                2: {'pose': [math.nan, 0, 0, 1]}, 3: {'pose': [-0.1, 0, 0, 1]},
                4: {'pose': [20, 0, 0, 1]}, 5: {'pose': [0, 15, 0, 1]}},
        'blue': {6: {'pose': [0.1, 0.1, 0, 1]}},
    })
    result = show_points.render_points_map(points, map_yaml, tmp_path / 'export')
    assert result['counts'] == {'red': 0, 'blue': 1}
    assert len(result['skipped']) == 6
    assert all(any(f'red/{number}' in message for message in result['skipped']) for number in range(6))
    with Image.open(result['image']) as rendered:
        assert rendered.getpixel((0, 29)) == (0, 0, 255)
        assert rendered.getpixel((39, 29)) == (205, 205, 205)
        assert rendered.getpixel((0, 0)) == (205, 205, 205)


def test_second_export_replaces_previous_points_and_preserves_source(show_points, tmp_path):
    points, map_yaml, source, _ = write_inputs(tmp_path)
    original = source.read_bytes()
    output = tmp_path / 'export'
    first = show_points.render_points_map(points, map_yaml, output)
    points.write_text('red: {}\nblue: {}\n')
    second = show_points.render_points_map(points, map_yaml, output)
    assert first['image'] == second['image'] and first['yaml'] == second['yaml']
    assert {path.name for path in output.iterdir()} == {'points_map.png', 'points_map.yaml'}
    with Image.open(second['image']) as rendered:
        assert rendered.getpixel((4, 26)) == (205, 205, 205)
        assert rendered.getpixel((22, 9)) == (205, 205, 205)
    assert source.read_bytes() == original


def test_failed_validation_leaves_previous_export_unchanged(show_points, tmp_path):
    points, map_yaml, _, metadata = write_inputs(tmp_path)
    result = show_points.render_points_map(points, map_yaml, tmp_path / 'export')
    saved = {result[key]: result[key].read_bytes() for key in ('image', 'yaml')}
    map_yaml.write_text(yaml.safe_dump(dict(metadata, resolution=0)))
    with pytest.raises(ValueError, match='resolution'):
        show_points.render_points_map(points, map_yaml, tmp_path / 'export')
    assert all(path.read_bytes() == data for path, data in saved.items())


def test_export_cannot_replace_its_source_map(show_points, tmp_path):
    points, map_yaml, _, _ = write_inputs(tmp_path)
    output = tmp_path / 'export'
    result = show_points.render_points_map(points, map_yaml, output)
    original = result['image'].read_bytes()
    with pytest.raises(ValueError, match='输入'):
        show_points.render_points_map(points, result['yaml'], output)
    assert result['image'].read_bytes() == original


@pytest.mark.parametrize('entrypoint', ['tool/show_points.py', 'install/tool/lib/tool/show_points',
                                      'install/lib/tool/show_points'])
def test_manual_cli_finds_its_source_workspace_from_an_unrelated_directory(tmp_path, entrypoint):
    assert SCRIPT.is_file(), 'The manual show_points tool has not been implemented'
    workspace = tmp_path / 'workspace'
    points, map_yaml, _, _ = write_inputs(workspace / 'src/fishbot/fishbot_navigation2/maps')
    default_points = workspace / 'src/function/config/on_demand/points.yaml'
    default_points.parent.mkdir(parents=True)
    points.replace(default_points)
    map_yaml.rename(map_yaml.with_name('silou_edited.yaml'))
    executable = workspace / entrypoint
    executable.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(SCRIPT, executable)
    unrelated = tmp_path / 'unrelated'
    unrelated.mkdir()
    completed = subprocess.run([sys.executable, str(executable)], cwd=unrelated,
                               capture_output=True, text=True, timeout=10)
    assert completed.returncode == 0, completed.stderr
    image = default_points.parent / 'points_map/points_map.png'
    assert image.is_file()
    assert str(image) in completed.stdout
    assert not (unrelated / 'points_map').exists()


def test_map_override_is_relative_to_workspace_not_process_directory(show_points, tmp_path, monkeypatch):
    points, map_yaml, _, _ = write_inputs(tmp_path / 'workspace/maps')
    workspace = tmp_path / 'workspace'
    default_points = workspace / 'src/function/config/on_demand/points.yaml'
    default_points.parent.mkdir(parents=True)
    points.replace(default_points)
    monkeypatch.chdir(tmp_path)
    assert show_points.main(['--workspace', str(workspace), '--map', 'maps/original.yaml']) == 0
    assert (default_points.parent / 'points_map/points_map.png').is_file()
