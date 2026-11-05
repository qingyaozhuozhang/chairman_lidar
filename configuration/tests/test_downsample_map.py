"""Offline voxel centroids and PCD writes preserve the source map and its frame."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).parents[2]
spec = importlib.util.spec_from_file_location('downsample_map', ROOT / 'tool/downsample_map.py')
downsample = importlib.util.module_from_spec(spec)
spec.loader.exec_module(downsample)


def write_cloud(path, points):
    path.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                    f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n'
                    + '\n'.join(' '.join(str(value) for value in point) for point in points))


def test_fixed_origin_voxels_use_centroids_on_both_sides_of_zero():
    points = [[-0.04, 0.01, 0.01], [-0.02, 0.03, 0.01],
              [0.01, 0.01, 0.01], [0.03, 0.03, 0.01],
              [0.051, 0.01, 0.01], [float('nan'), 0., 0.]]
    result = downsample.downsample_points(points, 0.05)
    result = result[np.argsort(result[:, 0])]
    np.testing.assert_allclose(result, [[-0.03, 0.02, 0.01], [0.02, 0.02, 0.01],
                                      [0.051, 0.01, 0.01]])


def test_every_point_in_large_voxel_contributes_to_one_centroid():
    points = np.full((3000, 3), 0.01)
    points[1000:, 0] = 0.04
    result = downsample.downsample_points(points, 0.05)
    assert result.shape == (1, 3)
    np.testing.assert_allclose(result[0], [0.03, 0.01, 0.01])


@pytest.mark.parametrize('size', [0., -1., float('nan'), float('inf')])
def test_invalid_voxel_size_is_rejected(size):
    with pytest.raises(ValueError, match='正有限数'):
        downsample.downsample_points([[0., 0., 0.]], size)


def test_nonfinite_cloud_and_out_of_range_coordinates_are_rejected():
    with pytest.raises(ValueError, match='没有有效'):
        downsample.downsample_points([[float('nan'), 0., 0.]], 0.05)
    with pytest.raises(ValueError, match='超出'):
        downsample.downsample_points([[1e20, 0., 0.]], 0.05)


def test_binary_pcd_output_preserves_input_and_map_coordinates(tmp_path):
    import open3d as o3d
    source, output = tmp_path / 'base.pcd', tmp_path / 'final.pcd'
    write_cloud(source, [[-2.03, 4.01, 0.01], [-2.01, 4.03, 0.03], [5.01, 6.01, 0.01]])
    original = source.read_bytes()
    total, finite, count = downsample.process_map(source, output)
    assert (total, finite, count) == (3, 3, 2)
    result = np.asarray(o3d.io.read_point_cloud(str(output)).points)
    np.testing.assert_allclose(result[np.argsort(result[:, 0])],
                               [[-2.02, 4.02, 0.02], [5.01, 6.01, 0.01]], atol=1e-6)
    assert source.read_bytes() == original
    assert b'DATA binary\n' in output.read_bytes()
    assert output.read_bytes().startswith(b'# chairman_lidar_voxel_size 0.05\n')
    assert not list(tmp_path.glob('.final_*.pcd'))


def test_failed_input_leaves_existing_output_untouched(tmp_path):
    source, output = tmp_path / 'base.pcd', tmp_path / 'final.pcd'
    write_cloud(source, [[float('nan'), 0., 0.]])
    output.write_bytes(b'previous map')
    with pytest.raises(ValueError):
        downsample.process_map(source, output)
    assert output.read_bytes() == b'previous map'
    with pytest.raises(ValueError, match='不能相同'):
        downsample.process_map(output, output)


def test_default_paths_follow_script_workspace_instead_of_working_directory(tmp_path, monkeypatch):
    workspace = tmp_path / 'workspace'
    directory = workspace / downsample.MAP_DIRECTORY
    directory.mkdir(parents=True)
    write_cloud(directory / 'base.pcd', [[0.01, 0.01, 0.01], [0.03, 0.03, 0.03]])
    monkeypatch.setattr(downsample, '__file__', str(workspace / 'install/tool/lib/tool/downsample_map'))
    monkeypatch.chdir(tmp_path)
    downsample.main([])
    assert (directory / 'final.pcd').is_file()


def test_explicit_paths_accept_legacy_voxel_option(tmp_path):
    source, output = tmp_path / 'input.pcd', tmp_path / 'output.pcd'
    write_cloud(source, [[0., 0., 0.]])
    downsample.main([str(source), str(output), '--voxel-size', '0.10'])
    assert output.is_file()
    assert output.read_bytes().startswith(b'# chairman_lidar_voxel_size 0.1\n')
