"""Point calibration regression tests; live ROS test is opt-in and isolated."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import importlib.util
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).parents[2]
spec = importlib.util.spec_from_file_location('chairman_point_test', ROOT / 'tool/point.py')
point = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = point
spec.loader.exec_module(point)


def wait_collecting(collector):
    deadline = time.monotonic() + 1
    while not collector.collecting:
        assert time.monotonic() < deadline
        time.sleep(0.001)


def sample(value, yaw=0.0):
    return point.PoseSample(float(value), float(-value), 1.0, yaw)


def test_trimmed_pose_and_yaw_wraparound():
    values = [-100, 1, 2, 3, 4, 5, 6, 7, 8, 100]
    poses = [sample(v, math.radians(179 if i % 2 else -179)) for i, v in enumerate(values)]
    result = point.average_pose(poses)
    assert result.x == 4.5
    assert result.y == -4.5
    assert result.z == 1.0
    assert abs(result.yaw) == pytest.approx(math.pi)
    assert point.trimmed_average([3.0] * 10) == 3.0


def test_batches_take_ten_fresh_finite_messages_and_do_not_reuse_previous_batch():
    collector = point.PoseCollector()
    collector.pose_callback(sample(-999))
    old_timestamp = time.time_ns()
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(collector.collect_next_average, 1.0)
        wait_collecting(collector)
        collector.pose_callback(sample(-999), SimpleNamespace(received_timestamp=old_timestamp))
        collector.pose_callback(sample(float('nan')))
        for value in range(9):
            collector.pose_callback(sample(value))
        assert not future.done(), 'Nine valid samples must not finish the batch'
        collector.pose_callback(sample(9))
        assert future.result(timeout=1).x == 4.5
        collector.pose_callback(sample(999))  # menu-time sample must not enter the next batch
        future = executor.submit(collector.collect_next_average, 1.0)
        wait_collecting(collector)
        for _ in range(10):
            collector.pose_callback(sample(7))
        assert future.result(timeout=1).x == 7.0


def test_partial_batch_times_out_and_can_restart():
    collector = point.PoseCollector()
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(collector.collect_next_average, 0.1)
        wait_collecting(collector)
        for i in range(9):
            collector.pose_callback(sample(i))
        with pytest.raises(TimeoutError, match='9/10'):
            future.result(timeout=1)
        assert not collector.collecting
        future = executor.submit(collector.collect_next_average, 1.0)
        wait_collecting(collector)
        for _ in range(10):
            collector.pose_callback(sample(4))
        assert future.result(timeout=1).x == 4.0


@pytest.mark.parametrize('pose_text', [
    '    pose: [-1.000, 2.000, -0.707, 0.707] # keep orientation\n',
    '    pose:\n    - -1.000 # x\n    - 2.000 # y\n    - -0.707\n    - 0.707\n',
])
def test_point_write_preserves_format_orientation_metadata_and_other_team(tmp_path, pose_text):
    path = tmp_path / 'points.yaml'
    original = ('# header\nred:\n  1:\n    name: "点位 # 1"\n' + pose_text
                + '    mode: 3\nblue:\n  1: {name: 蓝方, pose: [0, 0, 0, 1], mode: 2}\n')
    path.write_text(original)
    point.update_point_xy(path, 'red', 1, 3.1234, -4.5678)
    assert path.read_text() == original.replace('-1.000', '3.123').replace('2.000', '-4.568')
    assert list(tmp_path.iterdir()) == [path]


def test_real_project_points_and_regions_change_only_selected_coordinates(tmp_path):
    points = tmp_path / 'points.yaml'
    points.write_bytes((ROOT / point.POINTS_PATH).read_bytes())
    before = yaml.safe_load(points.read_text())
    point.update_point_xy(points, 'blue', 3, 1.25, -2.5)
    expected = deepcopy(before)
    expected['blue'][3]['pose'][:2] = [1.25, -2.5]
    assert yaml.safe_load(points.read_text()) == expected
    regions = tmp_path / 'regions.yaml'
    regions.write_bytes((ROOT / point.REGIONS_PATH).read_bytes())
    before_text = regions.read_text()
    expected = yaml.safe_load(before_text)
    point.update_region_center(regions, 'red', 2, 1.5, 2.5)
    selected = next(x for x in expected['regions_red'] if x['id'] == 2)
    selected.update(x_min=0.9, x_max=2.1, y_min=1.9, y_max=3.1)
    assert yaml.safe_load(regions.read_text()) == expected
    assert regions.read_text().startswith(before_text.splitlines()[0])


def test_dynamic_points_and_nonfinite_values_do_not_write(tmp_path):
    path = tmp_path / 'points.yaml'
    original = (ROOT / point.POINTS_PATH).read_text()
    path.write_text(original)
    with pytest.raises(ValueError, match='动态目标'):
        point.update_point_xy(path, 'red', 15, 1.0, 2.0)
    with pytest.raises(ValueError, match='有限数值'):
        point.update_point_xy(path, 'red', 1, math.inf, 2.0)
    assert path.read_text() == original
    assert not (tmp_path / '.point_backups').exists()


def test_workspace_search_is_local_and_explicit_path_does_not_fall_back(tmp_path, monkeypatch):
    monkeypatch.delenv('POINT_WORKSPACE', raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(Path, 'rglob', lambda *_: pytest.fail('Must not scan the filesystem recursively'))
    assert point.resolve_project_files() == (ROOT / point.POINTS_PATH, ROOT / point.REGIONS_PATH)
    with pytest.raises(FileNotFoundError):
        point.resolve_project_files(tmp_path)


def calibration_workspace(root, is_region):
    point_text = ('# points\nred:\n'
                  '  14: {name: 目标, pose: [0.000, 0.000, -0.707, 0.707], mode: 3}\n'
                  'blue:\n  14: {name: 蓝方, pose: [9, 8, 0, 1], mode: 2}\n')
    region_text = ('# regions\nregions_red:\n'
                   '  - {id: 14, name: 区域, x_min: 0, x_max: 1, y_min: 0, y_max: 1}\n'
                   'regions_blue:\n  - {id: 14, x_min: 9, x_max: 10, y_min: 8, y_max: 9}\n')
    sources = {
        False: ('src/config/framework/on_demand/points.yaml',
                'src/function/config/on_demand/points.yaml'),
        True: ('src/config/odometry/regions.yaml',
               'src/function/detail/continuous/odometry/config/regions.yaml'),
    }
    for mode, paths in sources.items():
        for index, relative in enumerate(paths):
            path = root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            text = region_text if mode else point_text
            path.write_text(f'# source {index}\n' + text, encoding='utf-8')
    package, relative = ('odometry', 'regions.yaml') if is_region else ('framework', 'on_demand/points.yaml')
    installed = [root / 'install' / prefix / 'share' / package / 'config' / relative
                 for prefix in (package, '')]
    for path in installed:
        path.parent.mkdir(parents=True, exist_ok=True)
        # An invalid installed YAML must not interfere with source calibration.
        path.write_text('installed: [must remain untouched\n', encoding='utf-8')
    return [root / name for name in sources[is_region]], installed


def calibrate_from_menu(root, is_region, monkeypatch):
    monkeypatch.setenv('POINT_WORKSPACE', str(root))
    answers = iter(['r', '14', '', 'b', 'b'])
    monkeypatch.setattr('builtins.input', lambda _: next(answers))
    collector = SimpleNamespace(elapsed=0.1, collect_next_average=lambda _: sample(1.25))
    path = root / (point.REGIONS_PATH if is_region else point.POINTS_PATH)
    point.run_mode(collector, path, is_region, 5.0)


@pytest.mark.parametrize('is_region', [False, True])
def test_menu_updates_both_sources_and_leaves_install_untouched(tmp_path, monkeypatch, is_region):
    sources, installed = calibration_workspace(tmp_path, is_region)
    before = {path: path.read_text() for path in sources + installed}
    calibrate_from_menu(tmp_path, is_region, monkeypatch)
    for path in sources:
        expected = yaml.safe_load(before[path])
        if is_region:
            expected['regions_red'][0].update(x_min=0.65, x_max=1.85, y_min=-1.85, y_max=-0.65)
        else:
            expected['red'][14]['pose'][:2] = [1.25, -1.25]
        assert yaml.safe_load(path.read_text()) == expected
        assert path.read_text().splitlines()[:2] == before[path].splitlines()[:2]
        assert not (path.parent / '.point_backups').exists()
    for path in installed:
        assert path.read_text() == before[path]
    assert not list((tmp_path / 'install').rglob('.point_backups'))


@pytest.mark.parametrize('is_region', [False, True])
def test_invalid_source_copy_prevents_all_writes(tmp_path, monkeypatch, capsys, is_region):
    sources, installed = calibration_workspace(tmp_path, is_region)
    sources[1].write_text(sources[1].read_text().replace('14', '13'))
    before = {path: path.read_bytes() for path in sources + installed}
    calibrate_from_menu(tmp_path, is_region, monkeypatch)
    assert '本次标定未完成' in capsys.readouterr().out
    assert all(path.read_bytes() == original for path, original in before.items())
    assert not list(tmp_path.rglob('.point_backups'))


@pytest.mark.parametrize('is_region', [False, True])
def test_second_source_write_failure_restores_first_source(tmp_path, monkeypatch, capsys, is_region):
    sources, installed = calibration_workspace(tmp_path, is_region)
    before = {path: path.read_bytes() for path in sources + installed}
    replace = os.replace
    failed = False

    def fail_once(source, target):
        nonlocal failed
        if Path(target) == sources[1] and not failed:
            failed = True
            raise OSError('simulated write failure')
        return replace(source, target)

    monkeypatch.setattr(point.os, 'replace', fail_once)
    calibrate_from_menu(tmp_path, is_region, monkeypatch)
    assert failed
    assert '本次标定未完成' in capsys.readouterr().out
    assert all(path.read_bytes() == original for path, original in before.items())
    assert not list(tmp_path.rglob('.point_backups'))


def test_unchanged_central_point_still_updates_source_copy(tmp_path, monkeypatch):
    sources, _ = calibration_workspace(tmp_path, False)
    sources[0].write_text(sources[0].read_text().replace('[0.000, 0.000,', '[1.250, -1.250,'))
    calibrate_from_menu(tmp_path, False, monkeypatch)
    assert yaml.safe_load(sources[1].read_text())['red'][14]['pose'][:2] == [1.25, -1.25]
    assert not (sources[0].parent / '.point_backups').exists()


def test_source_symlink_into_install_is_rejected(tmp_path, monkeypatch, capsys):
    sources, installed = calibration_workspace(tmp_path, False)
    installed[0].write_bytes(sources[1].read_bytes())
    sources[1].unlink()
    sources[1].symlink_to(installed[0])
    before = {path: path.read_bytes() for path in sources + installed}
    calibrate_from_menu(tmp_path, False, monkeypatch)
    assert '本次标定未完成' in capsys.readouterr().out
    assert all(path.read_bytes() == original for path, original in before.items())
    assert not list(tmp_path.rglob('.point_backups'))


@pytest.mark.skipif(os.environ.get('CHAIRMAN_ROS_INTEGRATION') != '1', reason='Requires sourced ROS and test domain 197')
def test_live_ros_pose_messages_collect_without_rosout():
    import rclpy
    from custom_msg.msg import PoseEuler
    from rclpy.executors import SingleThreadedExecutor
    assert os.environ.get('ROS_DOMAIN_ID') == '197'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    node = rclpy.create_node('point_test_pubsub')
    collector = point.PoseCollector(rclpy.ok)
    collector.attach(node, '/point_calibration_test')
    publisher = node.create_publisher(PoseEuler, '/point_calibration_test', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0:
            assert time.monotonic() < deadline, 'DDS discovery timeout'
            executor.spin_once(timeout_sec=0.05)
        publisher.publish(PoseEuler(x=-999.0))
        while collector.latest_sample is None:
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.05)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(collector.collect_next_average, 2.0)
            wait_collecting(collector)
            for value in [-100, 1, 2, 3, 4, 5, 6, 7, 8, 100]:
                publisher.publish(PoseEuler(x=float(value), y=2.0, z=3.0, yaw=0.5))
                executor.spin_once(timeout_sec=0.05)
                time.sleep(0.02)
            assert future.result(timeout=2).x == 4.5
            assert collector.elapsed < 1.0
            print(f'10 条实时 ROS 消息采集耗时：{collector.elapsed:.3f}s')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
