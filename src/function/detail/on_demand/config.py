"""读取并校验点位、功能、速度模式和手写运动参数。"""
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
        if path.name == 'special.yaml':
            continue
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
        for section in ('parameters', 'sprint_parameters'):
            flatten_parameters(mode.get(section, {}))
        if key in (1, 2, 3) and not mode.get('parameters'):
            raise ValueError(f'{path.name}: 缺少 parameters 原生参数表')
        if key == 2 and not mode.get('sprint_parameters'):
            raise ValueError('speed_2.yaml 缺少 sprint_parameters')
        if not isinstance(mode.get('pre_align', {}), dict):
            raise ValueError(f'{path.name}: pre_align 必须为字典')
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


def flatten_parameters(nodes):
    """将节点/ros__parameters/插件的原生 YAML 树展开为参数服务要求的名称。"""
    if not isinstance(nodes, dict):
        raise ValueError('parameters 必须按节点名称分组')
    result = {}
    def visit(mapping, prefix, output):
        for key, value in mapping.items():
            if not isinstance(key, str) or not key:
                raise ValueError('参数名必须为非空字符串')
            name = prefix + key
            if isinstance(value, dict):
                visit(value, name + '.', output)
            else:
                if name in output:
                    raise ValueError(f'重复参数路径: {name}')
                values = value if isinstance(value, list) else [value]
                if not values or any(type(v) not in (int, float, str, bool) for v in values):
                    raise ValueError(f'{name}: 参数值类型不支持')
                if any(type(v) in (int, float) and not math.isfinite(v) for v in values):
                    raise ValueError(f'{name}: 参数必须为有限数值')
                output[name] = value
    for remote, config in nodes.items():
        if not isinstance(remote, str) or not remote.strip('/') or not isinstance(config, dict):
            raise ValueError('参数配置必须包含节点名和 ros__parameters')
        if set(config) != {'ros__parameters'} or not isinstance(config['ros__parameters'], dict):
            raise ValueError(f'{remote}: 必须使用 ros__parameters')
        name = remote.strip('/')
        if name in result:
            raise ValueError(f'重复节点路径: {name}')
        result[name] = {}
        visit(config['ros__parameters'], '', result[name])
    return result


def dynamic_parameters(nodes):
    """完整参数块保留启动上下文，速度切换只发送行驶期间可更新的字段。

    Humble 的 controller_server 节点级参数在 FollowPath 执行期间被锁定；
    插件选择、时钟和 scale_velocities 也统一由 Nav2 启动基准管理。
    插件内部参数以及平滑器的速度、加减速、反馈和里程计配置参与切换。
    """
    result = {}
    for remote, values in flatten_parameters(nodes).items():
        selected = {}
        for name, value in values.items():
            if name == 'use_sim_time' or name.endswith('.plugin'):
                continue
            if remote == 'controller_server' and '.' not in name:
                continue
            if remote == 'velocity_smoother' and name == 'scale_velocities':
                continue
            selected[name] = value
        if selected:
            result[remote] = selected
    return result


def load_special_parameters(root=None, node=None):
    """读取 Nav2 同名参数，供本地手写控制使用；不修改运行中的 Nav2。

    YAML 和 ROS 参数使用原生名称，下面显式对应手写控制函数的内部属性。
    减速度按 Nav2 约定写负值，内部制动计算使用其幅值。
    """
    config = read_yaml(config_root(root) / 'on_demand/modes/special.yaml')
    bindings = {
        'parameters': {
            'CONTROL_PERIOD': 'controller_server.controller_frequency',
            'KP_LINEAR': 'controller_server.FollowPath.translation_kp',
            'KP_ANGULAR': 'controller_server.FollowPath.rotation_kp',
            'MAX_VEL_LINEAR': 'controller_server.FollowPath.v_linear_max',
            'MAX_VEL_ANGULAR': 'controller_server.FollowPath.v_angular_max',
            'MIN_VEL_LINEAR': 'controller_server.FollowPath.min_approach_linear_velocity',
            'MIN_VEL_ANGULAR': 'controller_server.FollowPath.min_approach_angular_velocity',
            'ERROR_TOLERANCE_DIST': 'controller_server.general_goal_checker.xy_goal_tolerance',
            'ERROR_TOLERANCE_YAW': 'controller_server.general_goal_checker.yaw_goal_tolerance',
            'MAX_ACCEL': 'velocity_smoother.max_accel',
            'MAX_DECEL': 'velocity_smoother.max_decel',
            'KP_YAW_CORRECT': 'simple_nav_node.yaw_correction.rotation_kp',
            'YAW_CORRECT_MAX_RATIO': 'simple_nav_node.yaw_correction.max_velocity_ratio',
            'KP_CROSS_TRACK': 'simple_nav_node.cross_track.translation_kp',
            'MAX_CROSS_TRACK_VEL': 'simple_nav_node.cross_track.v_linear_max',
            'MIN_CROSS_TRACK_VEL': 'simple_nav_node.cross_track.min_approach_linear_velocity',
            'ERROR_TOLERANCE_CROSS': 'simple_nav_node.cross_track.xy_goal_tolerance',
            'LIFT_CHECK_HEIGHT': 'simple_nav_node.lift_check_height',
        },
        'uphill_parameters': {
            'UPHILL_KP_LINEAR': 'controller_server.FollowPath.translation_kp',
            'UPHILL_MAX_VEL_LINEAR': 'controller_server.FollowPath.v_linear_max',
            'UPHILL_MAX_VEL_ANGULAR': 'controller_server.FollowPath.v_angular_max',
            'UPHILL_MIN_VEL_LINEAR': 'controller_server.FollowPath.min_approach_linear_velocity',
            'UPHILL_MIN_VEL_ANGULAR': 'controller_server.FollowPath.min_approach_angular_velocity',
            'UPHILL_ERROR_TOLERANCE_DIST': 'controller_server.general_goal_checker.xy_goal_tolerance',
            'UPHILL_ERROR_TOLERANCE_YAW': 'controller_server.general_goal_checker.yaw_goal_tolerance',
            'UPHILL_MAX_ACCEL': 'velocity_smoother.max_accel',
            'UPHILL_MAX_DECEL': 'velocity_smoother.max_decel',
            'UPHILL_KP_YAW_CORRECT': 'simple_nav_node.yaw_correction.rotation_kp',
            'UPHILL_YAW_CORRECT_MAX_RATIO': 'simple_nav_node.yaw_correction.max_velocity_ratio',
            'UPHILL_KP_CROSS_TRACK': 'simple_nav_node.cross_track.translation_kp',
            'UPHILL_MAX_CROSS_TRACK_VEL': 'simple_nav_node.cross_track.v_linear_max',
            'UPHILL_MIN_CROSS_TRACK_VEL': 'simple_nav_node.cross_track.min_approach_linear_velocity',
            'UPHILL_ERROR_TOLERANCE_CROSS': 'simple_nav_node.cross_track.xy_goal_tolerance',
            'UPHILL_GLOBAL_X_DISTANCE': 'simple_nav_node.global_x_distance',
            'UPHILL_GLOBAL_X_TIMEOUT': 'simple_nav_node.timeout',
        },
    }
    result = {}
    for section, fields in bindings.items():
        if section not in config:
            raise ValueError(f'special.yaml 缺少 {section}')
        native = flatten_parameters(config[section])
        for attribute, path in fields.items():
            remote, name = path.split('.', 1)
            if name not in native.get(remote, {}):
                raise ValueError(f'special.yaml/{section} 缺少 {path}')
            value = native[remote][name]
            # 参数均属于 simple_nav_node；上坡组加 uphill 前缀区分普通组。
            parameter = name if remote == 'simple_nav_node' else path
            if section == 'uphill_parameters':
                parameter = 'uphill.' + parameter
            if node is not None:
                node.declare_parameter(parameter, value)
                value = node.get_parameter(parameter).value
            vector = attribute.endswith(('MAX_ACCEL', 'MAX_DECEL'))
            values = value if vector and isinstance(value, (list, tuple)) else [value]
            if vector and len(values) != 3:
                raise ValueError(f'{parameter}: 必须为 [x, y, yaw] 三个数值')
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise ValueError(f'{parameter}: 必须为有限数值')
            if attribute.endswith('MAX_DECEL'):
                if any(v >= 0 for v in values):
                    raise ValueError(f'{parameter}: 按 Nav2 约定，减速度必须为负值')
                value = [abs(v) for v in values]
            elif any(v < 0 for v in values):
                raise ValueError(f'{parameter}: 必须为非负数值')
            if attribute == 'CONTROL_PERIOD':
                if value <= 0:
                    raise ValueError(f'{parameter}: 控制频率必须大于 0')
                value = 1.0 / value
            result[attribute] = value
    return result


def load_configuration(node):
    """加载功能目录和运动参数；Nav2 参数在执行模式时通过服务赋值。"""
    node.mode_configs = load_modes()
    red, blue = load_points(node.mode_configs)
    node.task_configs = load_tasks(red, blue)
    selected = read_selected_pose()
    node.PRESET_GOALS = red if selected in (1, 2) else blue
    node.speed_profiles = {key: dynamic_parameters(node.mode_configs[key]['parameters']) for key in (1,2,3)}
    node.speed_profiles['2_sprint'] = dynamic_parameters(node.mode_configs[2]['sprint_parameters'])
    node.active_mode = None
    node.current_profile = None
    node.dynamic_sprint_key = '2_sprint'
    node.dynamic_last_check_time = None
    dynamic = node.mode_configs[2]
    node.dynamic_switch_distance = float(dynamic.get('switch_distance', 1.5))
    node.dynamic_switch_check_period = float(dynamic.get('switch_check_period', 0.05))
    align = node.mode_configs[3].get('pre_align', {})
    for name, default in {'enabled': True, 'distance': 0.5, 'release_xy_tolerance': 0.1,
                          'release_yaw_tolerance': 0.15, 'near_adjust_timeout': 0.5}.items():
        value = align.get(name, default)
        if name == 'enabled':
            if type(value) is not bool:
                raise ValueError('pre_align.enabled 必须为布尔值')
        elif type(value) not in (float, int) or not math.isfinite(value) or value < 0:
            raise ValueError(f'pre_align.{name} 必须为非负有限数值')
        setattr(node, 'pre_align_' + name, value)
    for name, value in load_special_parameters(node=node).items():
        setattr(node, name, value)
    from ament_index_python.packages import get_package_share_directory
    regions = read_yaml(Path(get_package_share_directory('odometry')) / 'config/regions.yaml')
    key = 'regions_red' if selected in (1,2) else 'regions_blue'
    node.merlin_regions = {r['id']: [r['x_min'], r['x_max'], r['y_min'], r['y_max'], r['name']]
                           for r in regions[key] if 1 <= r['id'] <= 13}
    node.get_logger().info(f'场地编号: {selected}；功能服务 /set_nav_target 已就绪')
