"""Mapping shutdown waits for a fresh scan and publishes only valid new maps."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import shlex
import shutil

import pytest


ROOT = Path(__file__).parents[2]
SCRIPT = ROOT / 'configuration/main_map.py'
PCD = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
       'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n'
       '0.01 0.01 0.01\n0.03 0.03 0.03\n')


def load_main_map():
    assert SCRIPT.is_file(), 'The standalone mapping entry point is missing'
    spec = importlib.util.spec_from_file_location('mapping_entry', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wait_for(path, process):
    deadline = time.monotonic() + 15
    while not path.exists():
        if process.poll() is not None:
            pytest.fail(process.communicate()[0])
        assert time.monotonic() < deadline, f'Timed out waiting for {path}'
        time.sleep(0.02)


def start_pipeline(tmp_path, mode):
    # The mapper is the external hardware boundary; signals and map conversion are real.
    load_main_map()
    child = tmp_path / 'fake_mapper.py'
    child.write_text('''import os, pathlib, signal, sys, time
raw, ready, saving, mode, payload = sys.argv[1:]
def stop(signum, frame):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    pathlib.Path(saving).touch()
    if mode == 'hang':
        while True: time.sleep(1)
    time.sleep(0.3)
    if mode in ('save', 'invalid', 'save_fail'):
        temporary = pathlib.Path(raw).with_suffix('.tmp')
        temporary.write_text(payload if mode != 'invalid' else 'invalid PCD')
        temporary.replace(raw)
    sys.exit(3 if mode == 'save_fail' else 0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
pathlib.Path(ready).touch()
if mode == 'fail': sys.exit(7)
while True: time.sleep(0.02)
''')
    raw, final = tmp_path / 'scans.pcd', tmp_path / 'output/final.pcd'
    raw.write_text('previous raw map')
    final.parent.mkdir()
    final.write_text('previous final map')
    ready, saving = tmp_path / 'ready', tmp_path / 'saving'
    command = [sys.executable, str(child), str(raw), str(ready), str(saving), mode, PCD]
    downsample = [sys.executable, str(ROOT / 'tool/downsample_map.py'), str(raw), str(final)]
    code = ('import json, sys\nfrom configuration.main_map import run_mapping\n'
            'try:\n run_mapping(*json.loads(sys.argv[1]))\n'
            'except (OSError, ValueError, RuntimeError) as exc:\n print(exc, flush=True); sys.exit(2)\n')
    import json
    env = dict(os.environ, PYTHONPATH=str(ROOT))
    process = subprocess.Popen(
        [sys.executable, '-c', code, json.dumps([command, str(raw), str(final), downsample, 0.8 if mode == 'hang' else 8.0])],
        cwd=tmp_path, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    return process, raw, final, ready, saving


def test_ctrl_c_waits_for_atomic_save_then_downsamples_and_ignores_second_signal(tmp_path):
    process, raw, final, ready, saving = start_pipeline(tmp_path, 'save')
    try:
        wait_for(ready, process)
        process.send_signal(signal.SIGINT)
        wait_for(saving, process)
        assert final.read_text() == 'previous final map'
        process.send_signal(signal.SIGINT)
        output = process.communicate(timeout=25)[0]
        assert process.returncode == 0, output
        assert raw.read_text() == PCD
        assert final.read_bytes().startswith(b'# chairman_lidar_voxel_size 0.05\n')
        import open3d as o3d
        import numpy as np
        points = np.asarray(o3d.io.read_point_cloud(str(final)).points)
        np.testing.assert_allclose(points, [[0.02, 0.02, 0.02]], atol=1e-6)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=15)


@pytest.mark.parametrize('mode', ['empty', 'fail', 'invalid', 'hang', 'save_fail'])
def test_failed_run_preserves_previous_final_and_does_not_reuse_stale_raw(tmp_path, mode):
    process, raw, final, ready, saving = start_pipeline(tmp_path, mode)
    try:
        if mode != 'fail':
            wait_for(ready, process)
            process.send_signal(signal.SIGTERM)
        output = process.communicate(timeout=25)[0]
        assert process.returncode != 0, output
        assert final.read_text() == 'previous final map'
        if mode in ('empty', 'fail', 'hang'):
            assert raw.read_text() == 'previous raw map'
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=15)


@pytest.mark.parametrize('installed', [False, True])
def test_workspace_resolution_is_independent_of_cwd(tmp_path, monkeypatch, installed):
    module = load_main_map()
    workspace = tmp_path / 'relocated workspace'
    (workspace / 'configuration').mkdir(parents=True)
    (workspace / 'configuration/main_map.py').touch()
    (workspace / 'src/fishbot/fast_lio').mkdir(parents=True)
    entry = workspace / ('install/configuration/local/lib/python3.10/dist-packages/configuration/main_map.py'
                         if installed else 'configuration/main_map.py')
    monkeypatch.setattr(module, '__file__', str(entry))
    monkeypatch.chdir(tmp_path)
    assert module.find_workspace() == workspace


def test_dry_run_prints_livox_and_map_paths_without_starting_or_creating_files(tmp_path):
    load_main_map()
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--workspace', str(ROOT), '--dry-run', '--no-rviz'],
        cwd=tmp_path, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert 'start_livox_driver:=true' in result.stdout
    assert 'pcd_save_en:=true' in result.stdout
    assert 'pcd_save_interval:=-1' in result.stdout
    assert 'src/fishbot/fast_lio/PCD/scans.pcd' in result.stdout
    assert 'src/fishbot/fishbot_navigation2/PCD/final.pcd' in result.stdout
    assert not list(tmp_path.iterdir())


def test_cpp_storage_resolves_relocated_install_and_keeps_existing_map_on_failure(tmp_path):
    header = ROOT / 'src/fishbot/fast_lio/include/map_storage.hpp'
    assert header.is_file(), 'Runtime map storage is missing'
    if not shutil.which('g++') or not shutil.which('pkg-config'):
        pytest.skip('C++ and PCL development tools are required')
    flags = subprocess.run(['pkg-config', '--cflags', '--libs', 'pcl_io'], capture_output=True, text=True)
    if flags.returncode:
        pytest.skip('PCL development files are required')
    workspace = tmp_path / 'relocated workspace'
    package = workspace / 'src/fishbot/fast_lio'
    package.mkdir(parents=True)
    (package / 'package.xml').touch()
    binary = workspace / 'install/fast_lio/lib/fast_lio/storage_test'
    binary.parent.mkdir(parents=True)
    source = tmp_path / 'storage_test.cpp'
    source.write_text(r'''#include "map_storage.hpp"
#include <fstream>
#include <iostream>
int main() {
    try {
        auto root = fast_lio_storage::package_root("");
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.push_back(pcl::PointXYZ(1, 2, 3));
        auto output = root / "PCD/scans.pcd";
        fast_lio_storage::save_atomic(output, cloud);
        try { fast_lio_storage::save_atomic(output, pcl::PointCloud<pcl::PointXYZ>()); return 2; }
        catch (const std::exception&) {}
        std::ofstream(root / "blocked").put('x');
        try { fast_lio_storage::save_atomic(root / "blocked/scans.pcd", cloud); return 3; }
        catch (const std::exception&) {}
        std::cout << root.string();
    } catch (const std::exception& error) { std::cerr << error.what(); return 1; }
}
''')
    compiled = subprocess.run(['g++', '-std=c++17', '-I', str(header.parent), str(source),
                               '-o', str(binary), *shlex.split(flags.stdout)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    result = subprocess.run([str(binary)], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout == str(package)
    assert b'POINTS 1\n' in (package / 'PCD/scans.pcd').read_bytes()
    assert sorted(path.name for path in (package / 'PCD').iterdir()) == ['scans.pcd']
