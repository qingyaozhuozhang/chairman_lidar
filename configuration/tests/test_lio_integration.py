"""Navigation launch and mirrored MID360 configuration contracts."""
import importlib.util
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_navigation_uses_point_lio_and_offline_final_map(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    from launch import LaunchContext, LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import perform_substitutions

    path = ROOT / 'src/fishbot/fishbot_navigation2/launch/navigation2.launch.py'
    spec = importlib.util.spec_from_file_location('navigation_lio_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(ROOT / 'src/fishbot' / name))
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = 'false'
    arguments = {action.name: action for action in description.entities
                 if isinstance(action, DeclareLaunchArgument)}
    assert perform_substitutions(context, arguments['pcd_map'].default_value).endswith('/PCD/final.pcd')
    includes = []
    for action in description.entities:
        if isinstance(action, IncludeLaunchDescription):
            source = action.launch_description_source
            # Resolve include paths without importing or starting hardware children.
            monkeypatch.setattr(source, '_get_launch_description', lambda _: LaunchDescription())
            source.get_launch_description(context)
            includes.append(source.location)
    assert any(path.endswith('/point_lio/launch/start.launch.py') for path in includes)
    assert not any('/fast_lio/' in path for path in includes)
    assert sum('/livox_ros_driver2/' in path for path in includes) == 1


def test_mid360_mirrors_and_hardware_calibration_match():
    manifest = json.loads((ROOT / 'src/config/manifest.json').read_text())
    files = {}
    for package in ('fast_lio', 'point_lio'):
        key = package + '/mid360.yaml'
        central = ROOT / 'src/config' / key
        runtime = ROOT / manifest[key]
        assert central.read_bytes() == runtime.read_bytes()
        files[package] = yaml.safe_load(runtime.read_text())['/**']['ros__parameters']
    fast, point = files['fast_lio'], files['point_lio']
    for key in ('lid_topic', 'imu_topic'):
        assert point['common'][key] == fast['common'][key]
    for key in ('extrinsic_T', 'extrinsic_R'):
        assert point['mapping'][key] == fast['mapping'][key]
    for key in ('filter_size_surf', 'filter_size_map', 'point_filter_num'):
        assert point[key] == fast[key]
    assert point['publish']['scan_bodyframe_pub_en'] is True
    assert point['pcd_save']['pcd_save_en'] is False
