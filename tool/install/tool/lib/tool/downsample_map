#!/usr/bin/env python3
"""离线将 base.pcd 按体素质心降采样为 final.pcd，保留 PCD 坐标系。"""
import argparse
import math
import os
from pathlib import Path
import tempfile
import time

import numpy as np


# 地图体素边长，单位 m；定位节点直接使用输出的 final.pcd。
GLOBAL_LEAF_SIZE = 0.05
VOXEL_HEADER = "# chairman_lidar_voxel_size"
MAP_DIRECTORY = Path("src/fishbot/fishbot_navigation2/PCD")


def project_map_directory():
    """源码入口与 ros2 run 安装入口均定位到所属工程的源码地图目录。"""
    for parent in Path(__file__).resolve().parents:
        directory = parent / MAP_DIRECTORY
        if directory.is_dir():
            return directory
    raise ValueError("找不到项目地图目录，请显式指定输入和输出 PCD 路径")


def downsample_points(points, leaf_size):
    """使用固定原点格网，每个占用体素输出全部有效点的 XYZ 质心。"""
    if not math.isfinite(leaf_size) or leaf_size <= 0:
        raise ValueError("体素边长必须是正有限数")
    inverse_leaf = 1.0 / leaf_size
    if not math.isfinite(inverse_leaf):
        raise ValueError("体素边长过小，无法计算格网坐标")
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("点云必须为 N×3 的 XYZ 数组")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise ValueError("输入点云没有有效的 XYZ 点")
    # 与 small_gicp 使用相同的 floor(x / leaf) 格网和 21 位坐标范围。
    with np.errstate(over="ignore", invalid="ignore"):
        coordinates = np.floor(points * inverse_leaf)
    offset = 1 << 20
    if (coordinates < -offset).any() or (coordinates >= offset).any():
        raise ValueError("点坐标超出当前体素尺寸支持的范围，请检查地图单位或体素边长")
    coordinates = (coordinates.astype(np.int64) + offset).astype(np.uint64)
    keys = coordinates[:, 0] | (coordinates[:, 1] << 21) | (coordinates[:, 2] << 42)
    _, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
    result = np.column_stack(
        [np.bincount(inverse, weights=points[:, axis]) / counts for axis in range(3)]
    )
    return result


def process_map(input_path, output_path, leaf_size=GLOBAL_LEAF_SIZE):
    """读取有效 XYZ 并原子替换输出文件，输入地图始终保留。"""
    input_path, output_path = Path(input_path), Path(output_path)
    if input_path.resolve() == output_path.resolve():
        raise ValueError("输入和输出不能相同")
    if not input_path.is_file():
        raise ValueError(f"输入点云不存在：{input_path}")
    if not output_path.parent.is_dir():
        raise ValueError(f"输出目录不存在：{output_path.parent}")
    try:
        import open3d as o3d
    except ImportError as exc:
        raise ValueError("缺少 open3d，请安装 python3-open3d 或使用已安装该模块的环境") from exc
    cloud = o3d.io.read_point_cloud(str(input_path))
    points = np.asarray(cloud.points)
    finite_count = int(np.isfinite(points).all(axis=1).sum())
    sampled = downsample_points(points, leaf_size)
    result = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sampled))
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{output_path.stem}_", suffix=".pcd", dir=output_path.parent
    )
    os.close(descriptor)
    try:
        if not o3d.io.write_point_cloud(
            temporary, result, write_ascii=False, compressed=False
        ):
            raise ValueError("点云写入失败")
        # PCD 注释兼容标准读取器，定位端据此使用相同体素边长处理实时扫描。
        output = Path(temporary)
        payload = output.read_bytes()
        output.write_bytes(f"{VOXEL_HEADER} {leaf_size!r}\n".encode("ascii") + payload)
        os.replace(temporary, output_path)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return len(points), finite_count, len(sampled)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, nargs="?", help="默认使用项目 PCD/base.pcd")
    parser.add_argument("output", type=Path, nargs="?", help="默认使用项目 PCD/final.pcd")
    parser.add_argument(
        "--global-leaf-size",
        "--voxel-size",
        dest="leaf_size",
        type=float,
        default=GLOBAL_LEAF_SIZE,
        help=f"地图体素边长，单位 m，默认 {GLOBAL_LEAF_SIZE}",
    )
    options = parser.parse_args(args)
    if (options.input is None) != (options.output is None):
        parser.error("自定义路径时必须同时提供输入和输出文件")
    if not math.isfinite(options.leaf_size) or options.leaf_size <= 0:
        parser.error("--global-leaf-size 必须是正有限数")
    begin = time.monotonic()
    try:
        if options.input is None:
            directory = project_map_directory()
            options.input, options.output = (
                directory / "base.pcd",
                directory / "final.pcd",
            )
        print(
            f"输入：{options.input}\n输出：{options.output}\n体素边长：{options.leaf_size} m",
            flush=True,
        )
        total, finite, sampled = process_map(
            options.input, options.output, options.leaf_size
        )
    except (ValueError, OSError, RuntimeError) as exc:
        parser.error(str(exc))
    print(
        f"降采样完成：{total} 点（有效 {finite}）→ {sampled} 点；"
        f"耗时 {time.monotonic() - begin:.2f}s"
    )
    print("构建 fishbot_navigation2 并重新加载环境后，重启 Navigation2 使用新地图。")


if __name__ == "__main__":
    main()
