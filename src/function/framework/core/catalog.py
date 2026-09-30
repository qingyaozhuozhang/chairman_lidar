"""按需任务、点位与速度模式的目录。"""
import math
import os
from pathlib import Path
import re

import yaml


def read_selected_pose():
    """读取 main_boot 传入的场地；独立调试时缺省为 1，不询问用户。"""
    value = os.environ.get('SELECTED_POSE', '1')
    if value not in ('1', '2', '3', '4'):
        raise ValueError('SELECTED_POSE 必须是 1、2、3 或 4，请由 main_boot 选择场地')
    return int(value)


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f'YAML键重复: {key}（行 {key_node.start_mark.line + 1}）')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def config_root(root=None):
    if root is not None:
        return Path(root)
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('framework')) / 'config'


def read_yaml(path):
    with path.open(encoding='utf-8') as stream:
        data = yaml.load(stream, Loader=UniqueKeyLoader)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: 顶层必须是字典')
    return data


def load_modes(root=None):
    result = {}
    for path in sorted((config_root(root) / 'on_demand/modes').glob('*.yaml')):
        mode = read_yaml(path)
        key = mode.get('id')
        if type(key) is not int or key < 1:
            raise ValueError(f'{path.name}: 模式id必须是正整数')
        if key in result:
            raise ValueError(f'模式id重复: {key}')
        if not isinstance(mode.get('name'), str) or not mode['name'].strip():
            raise ValueError(f'{path.name}: name必须为非空字符串')
        base = mode.get('base_mode')
        if type(base) is not int or base not in (1, 2, 3):
            raise ValueError(f'{path.name}: base_mode 必须为1/2/3')
        if key in (1, 2, 3) and key != base:
            raise ValueError('内置模式1/2/3的base_mode必须等于自身编号')
        for section in ('simple_nav_node', 'overrides'):
            if not isinstance(mode.get(section, {}), dict):
                raise ValueError(f'{path.name}: {section}必须为字典')
        if key not in (1, 2, 3) and mode.get('simple_nav_node'):
            raise ValueError('新增速度模式使用overrides；simple_nav_node仅供内置模式使用')
        for node, parameters in mode.get('overrides', {}).items():
            if not isinstance(node, str) or not node.strip('/') or not isinstance(parameters, dict):
                raise ValueError(f'{path.name}: overrides需按节点名称分组')
        result[key] = mode
    if not {1, 2, 3}.issubset(result):
        raise ValueError('缺少原模式1、2或3')
    return result


def check_target_id(value):
    if type(value) is not int or not -128 <= value <= 127:
        raise ValueError(f'功能编号必须在 SetNavTarget int8 范围 -128～127 内: {value}')


def load_points(modes, root=None):
    data = read_yaml(config_root(root) / 'on_demand/points.yaml')
    teams = []
    for team in ('red', 'blue'):
        if not isinstance(data.get(team), dict):
            raise ValueError(f'points.yaml 缺少 {team} 点位字典')
        points = {}
        for target, point in data[team].items():
            check_target_id(target)
            if target < 0 or not isinstance(point, dict):
                raise ValueError('点位编号必须非负，点位内容必须是字典')
            if type(point.get('mode')) is not int or point['mode'] not in modes:
                raise ValueError(f'{team}/{target}: 未定义模式 {point.get("mode")}')
            pose = point.get('pose')
            if not isinstance(pose, list) or len(pose) != 4:
                raise ValueError(f'{team}/{target}: pose需为[x,y,qz,qw]')
            if pose != [None] * 4:
                if any(type(v) not in (int, float) or not math.isfinite(v) for v in pose):
                    raise ValueError(f'{team}/{target}: 坐标必须是有限数值')
                if math.hypot(pose[2], pose[3]) < 1e-9:
                    raise ValueError(f'{team}/{target}: 四元数不能全零')
            points[target] = [*pose, str(point.get('name', target)), point['mode']]
        teams.append(points)
    return tuple(teams)


def load_tasks(red, blue, root=None):
    data = read_yaml(config_root(root) / 'on_demand/functions.yaml')
    result = {}
    entries = data.get('functions')
    if not isinstance(entries, list):
        raise ValueError('functions.yaml/functions必须为列表')
    for task in entries:
        if not isinstance(task, dict):
            raise ValueError('功能登记项必须为字典')
        target = task.get('id')
        check_target_id(target)
        if target in result:
            raise ValueError(f'功能编号重复: {target}')
        if target in red or target in blue:
            if task.get('uses_point') is not True:
                raise ValueError(f'功能编号与点位冲突: {target}')
        if task.get('uses_point') is True and (target not in red or target not in blue):
            raise ValueError(f'{target}: uses_point功能需要在红蓝双方点位中登记')
        if not re.fullmatch(r'(?:[a-z][a-z0-9_]*\.)+task', str(task.get('module', ''))):
            raise ValueError(f'{target}: module示例为special.my_task.task')
        if not isinstance(task.get('parameters', {}), dict):
            raise ValueError(f'{target}: parameters必须为字典')
        result[target] = task
    for points in (red, blue):
        for target, point in points.items():
            if point[:4] == [None] * 4 and target not in result:
                raise ValueError(f'{target}: 空坐标必须有对应的特殊任务')
    return result
