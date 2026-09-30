#!/usr/bin/env python3
"""对指定PCD地图做体素降采样；输入输出相对于当前终端目录。"""
import argparse
import math
from pathlib import Path


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--voxel-size', type=float, default=0.1)
    options = parser.parse_args(args)
    if not math.isfinite(options.voxel_size) or options.voxel_size <= 0:
        parser.error('--voxel-size 必须是正有限数')
    if not options.input.is_file():
        parser.error('输入点云不存在')
    if options.input.resolve() == options.output.resolve():
        parser.error('输入和输出不能相同')
    try:
        import open3d as o3d
    except ImportError:
        parser.error('缺少 open3d；请安装 python3-open3d')
    cloud = o3d.io.read_point_cloud(str(options.input))
    if cloud.is_empty():
        parser.error('输入点云为空或读取失败')
    result = cloud.voxel_down_sample(options.voxel_size)
    if not o3d.io.write_point_cloud(str(options.output), result):
        parser.error('点云写入失败；请检查输出目录')
    print(f'{len(cloud.points)} → {len(result.points)} 点；保存至 {options.output}')


if __name__ == '__main__':
    main()
