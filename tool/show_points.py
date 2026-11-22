#!/usr/bin/env python3
"""手动将红蓝点位画到栅格地图，覆盖保存 points_map/points_map.png 和 YAML。"""
import argparse
import math
import os
from pathlib import Path
import sys
import tempfile

from PIL import Image, ImageDraw
import yaml

POINTS_PATH = Path('src/function/config/on_demand/points.yaml')
DEFAULT_MAP_PATH = Path('src/fishbot/fishbot_navigation2/maps/room_zhengzhi.yaml')
OUTPUT_PATH = POINTS_PATH.parent / 'points_map'
COLORS = {'red': (255, 0, 0), 'blue': (0, 0, 255)}


def find_workspace(explicit=None):
    """源码和 ros2 run 安装入口均使用所属工程 src 下的点位与输出目录。"""
    if explicit is not None:
        root = Path(explicit).expanduser().resolve()
        if (root / POINTS_PATH).is_file():
            return root
        raise ValueError(f'工作空间缺少 {POINTS_PATH}：{root}')
    for start in (Path(__file__).resolve().parent, Path.cwd().resolve()):
        for root in (start, *start.parents):
            if (root / POINTS_PATH).is_file():
                return root
    raise ValueError('找不到源码工程，请用 --workspace 指定 chairman_lidar 根目录')


def read_mapping(path):
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path}：顶层必须为 YAML 字典')
    return data


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def world_to_pixel(x, y, resolution, origin, width, height):
    """逆变换地图原点位姿，再将左下角栅格索引转换为左上角图像行号。"""
    dx, dy = x - origin[0], y - origin[1]
    cosine, sine = math.cos(origin[2]), math.sin(origin[2])
    column = (cosine * dx + sine * dy) / resolution
    row_from_bottom = (-sine * dx + cosine * dy) / resolution
    if not (0 <= column < width and 0 <= row_from_bottom < height):
        return None
    return math.floor(column), height - 1 - math.floor(row_from_bottom)


def render_points_map(points_yaml, map_yaml, output_directory, radius=3):
    """从原图重新生成 RGB PNG；不修改输入地图、点位或原点/分辨率。"""
    points_yaml, map_yaml = Path(points_yaml).resolve(), Path(map_yaml).resolve()
    output_directory = Path(output_directory).resolve()
    if type(radius) is not int or radius < 1:
        raise ValueError('点半径必须为正整数像素')
    points, metadata = read_mapping(points_yaml), read_mapping(map_yaml)
    resolution, origin = metadata.get('resolution'), metadata.get('origin')
    if not finite_number(resolution) or resolution <= 0:
        raise ValueError('地图 resolution 必须为正有限数')
    if not isinstance(origin, list) or len(origin) != 3 or not all(map(finite_number, origin)):
        raise ValueError('地图 origin 必须为有限数值 [x, y, yaw]，yaw 单位 rad')
    image_name = metadata.get('image')
    if not isinstance(image_name, str) or not image_name:
        raise ValueError('地图 YAML 缺少 image 路径')
    source_image = (map_yaml.parent / Path(image_name).expanduser()).resolve()
    output_image = output_directory / 'points_map.png'
    output_yaml = output_directory / 'points_map.yaml'
    inputs = {points_yaml, map_yaml, source_image}
    if any(path.resolve() in inputs for path in (output_image, output_yaml)):
        raise ValueError('输出不能覆盖输入地图或点位文件')
    with Image.open(source_image) as original:
        rendered = original.convert('RGB')
    width, height = rendered.size
    draw = ImageDraw.Draw(rendered)
    counts = {team: 0 for team in COLORS}
    skipped = []
    for team, color in COLORS.items():
        items = points.get(team)
        if not isinstance(items, dict):
            raise ValueError(f'points.yaml 缺少 {team} 点位字典')
        for number, item in items.items():
            pose = item.get('pose') if isinstance(item, dict) else None
            name = f'{team}/{number}'
            if not isinstance(pose, list) or len(pose) < 2 or any(v is None for v in pose[:2]):
                skipped.append(f'{name}：动态目标或空坐标')
                continue
            if not all(map(finite_number, pose[:2])):
                skipped.append(f'{name}：坐标不是有限数值')
                continue
            pixel = world_to_pixel(*pose[:2], resolution, origin, width, height)
            if pixel is None:
                skipped.append(f'{name}：坐标 ({pose[0]:g}, {pose[1]:g}) 超出地图边界')
                continue
            x, y = pixel
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)
            counts[team] += 1

    # 先完成读取、校验与绘图，再替换固定文件名；再次运行不会叠加旧点位。
    output_directory.mkdir(parents=True, exist_ok=True)
    temporary_paths = []
    try:
        for target in (output_image, output_yaml):
            descriptor, temporary = tempfile.mkstemp(prefix='.' + target.stem + '_',
                                                     suffix=target.suffix, dir=output_directory)
            os.close(descriptor)
            temporary_paths.append(Path(temporary))
        rendered.save(temporary_paths[0], format='PNG')
        exported_metadata = dict(metadata, image=output_image.name)
        temporary_paths[1].write_text(yaml.safe_dump(exported_metadata, sort_keys=False,
                                                    allow_unicode=True), encoding='utf-8')
        os.replace(temporary_paths[0], output_image)
        os.replace(temporary_paths[1], output_yaml)
    finally:
        for temporary in temporary_paths:
            temporary.unlink(missing_ok=True)
    return {'image': output_image, 'yaml': output_yaml, 'counts': counts, 'skipped': skipped}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, help='源码工程根目录；默认根据脚本位置查找')
    parser.add_argument('--map', type=Path, dest='map_yaml', default=DEFAULT_MAP_PATH,
                        help=f'地图 YAML，默认 {DEFAULT_MAP_PATH}；相对路径以工程根目录为基准')
    parser.add_argument('--radius', type=int, default=3, help='点半径，单位像素，默认 3')
    options = parser.parse_args(args)
    try:
        workspace = find_workspace(options.workspace)
        map_yaml = workspace / options.map_yaml.expanduser()
        result = render_points_map(workspace / POINTS_PATH, map_yaml,
                                   workspace / OUTPUT_PATH, options.radius)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f'生成失败：{exc}', file=sys.stderr)
        return 1
    print(f'点位：{workspace / POINTS_PATH}\n底图：{map_yaml.resolve()}')
    print(f'已绘制红方 {result["counts"]["red"]} 个、蓝方 {result["counts"]["blue"]} 个点；'
          f'跳过 {len(result["skipped"])} 个点。')
    for message in result['skipped']:
        print(f'跳过 {message}')
    print(f'已覆盖保存：{result["image"]}\n地图描述：{result["yaml"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
