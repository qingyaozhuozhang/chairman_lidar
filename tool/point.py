#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""chairman_lidar 点位标定：直接采集 /odom_map 的 10 条消息，逐项去极值取平均。

ros2 run tool point
  定点：src/function/config/on_demand/points.yaml，只更新 x/y。
  区域：src/config/odometry/regions.yaml，中心 ±0.6 m。
只写源码配置；定点修改后构建 framework、重启功能进程，区域修改后 sync、重启相关节点。
"""
import argparse
from copy import deepcopy
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import uuid

import yaml
from yaml.nodes import MappingNode, ScalarNode, SequenceNode

SAMPLE_COUNT = 10
REGION_HALF_SIZE = 0.6
POINTS_PATH = Path('src/function/config/on_demand/points.yaml')
REGIONS_PATH = Path('src/config/odometry/regions.yaml')


@dataclass(frozen=True)
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float


def trimmed_average(values):
    """逐项剔除一个最小值和一个最大值，重复值也只各剔除一个。"""
    if len(values) != SAMPLE_COUNT or not all(math.isfinite(v) for v in values):
        raise ValueError('需要恰好 10 个有限数值')
    return math.fsum(sorted(values)[1:-1]) / (SAMPLE_COUNT - 2)


def average_pose(samples):
    if len(samples) != SAMPLE_COUNT:
        raise ValueError('需要恰好 10 条位姿消息')
    # 先相对第一条解开 ±pi 环绕，避免 179° 与 -179° 平均成 0°。
    reference = samples[0].yaw
    yaw = trimmed_average([
        reference + math.atan2(math.sin(s.yaw - reference), math.cos(s.yaw - reference))
        for s in samples
    ])
    return PoseSample(
        trimmed_average([s.x for s in samples]),
        trimmed_average([s.y for s in samples]),
        trimmed_average([s.z for s in samples]),
        math.atan2(math.sin(yaw), math.cos(yaw)),
    )


class PoseCollector:
    """后台持续消费消息；仅记录本次确认之后接收的 10 条有效位姿。"""
    def __init__(self, ros_ok=lambda: True):
        self.ros_ok = ros_ok
        self.condition = threading.Condition()
        self.latest_sample = None
        self.samples = []
        self.collecting = False
        self.started_at = 0.0
        self.started_wall_ns = 0
        self.elapsed = 0.0

    def attach(self, node, topic='/odom_map'):
        from custom_msg.msg import PoseEuler
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        # 深度 1、非持久化：菜单输入期间继续消费，不积压一串旧位姿。
        # BEST_EFFORT 同时兼容可靠与尽力而为的发布者。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = node.create_subscription(PoseEuler, topic, self.pose_callback, qos)

    def pose_callback(self, msg, info=None):
        received_at = time.monotonic()
        sample = PoseSample(float(msg.x), float(msg.y), float(msg.z), float(msg.yaw))
        if not all(math.isfinite(v) for v in (sample.x, sample.y, sample.z, sample.yaw)):
            return
        with self.condition:
            self.latest_sample = sample
            # 若 DDS 提供本机接收时间，排除确认前已到达队列的消息。
            received_ns = getattr(info, 'received_timestamp', 0)
            if (not self.collecting or received_at < self.started_at
                    or (received_ns > 0 and received_ns < self.started_wall_ns)):
                return
            if len(self.samples) < SAMPLE_COUNT:
                self.samples.append(sample)
                self.condition.notify_all()

    def collect_next_average(self, timeout=5.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('采集超时必须为正有限秒数')
        with self.condition:
            if self.collecting:
                raise RuntimeError('已有采集正在执行')
            self.samples = []
            self.started_at = time.monotonic()
            self.started_wall_ns = time.time_ns()
            self.collecting = True
            deadline = self.started_at + timeout
            try:
                while len(self.samples) < SAMPLE_COUNT:
                    if not self.ros_ok():
                        raise RuntimeError('ROS 已停止，取消采集')
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(
                            f'{timeout:g} 秒内仅收到 {len(self.samples)}/10 条有效位姿；'
                            '未写入文件，请检查 odometry、TF 和位姿话题')
                    self.condition.wait(timeout=min(remaining, 0.1))
                samples = list(self.samples)
                self.elapsed = time.monotonic() - self.started_at
            finally:
                self.collecting = False
        return average_pose(samples)


def find_workspace(explicit=None):
    """只检查脚本/当前目录的祖先，不递归扫描其他工程。"""
    explicit = explicit or os.environ.get('POINT_WORKSPACE')
    if explicit:
        root = Path(explicit).expanduser().resolve()
        if (root / POINTS_PATH).is_file() and (root / REGIONS_PATH).is_file():
            return root
        raise FileNotFoundError(f'工作空间缺少 points.yaml 或中央 regions.yaml：{root}')
    for start in (Path(__file__).resolve().parent, Path.cwd().resolve()):
        for root in (start, *start.parents):
            if (root / POINTS_PATH).is_file() and (root / REGIONS_PATH).is_file():
                return root
    raise FileNotFoundError('找不到 chairman_lidar 源码工程；请用 --workspace 指定项目根目录')


def resolve_project_files(workspace=None):
    root = find_workspace(workspace)
    return root / POINTS_PATH, root / REGIONS_PATH


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f'YAML 键重复：{key}（第 {key_node.start_mark.line + 1} 行）')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def read_document(path):
    text = path.read_text(encoding='utf-8')
    data = yaml.load(text, Loader=UniqueKeyLoader)
    tree = yaml.compose(text, Loader=yaml.SafeLoader)
    if not isinstance(data, dict) or not isinstance(tree, MappingNode):
        raise ValueError(f'{path}：顶层必须是 YAML 字典')
    return text, data, tree


def mapping_value(node, key):
    if not isinstance(node, MappingNode):
        raise ValueError('配置节点必须是字典')
    for key_node, value in node.value:
        if key_node.value == str(key):
            return value
    raise ValueError(f'配置中找不到 {key}')


def point_items(path, side):
    _, data, _ = read_document(path)
    items = data.get(side)
    if not isinstance(items, dict):
        raise ValueError(f'points.yaml 中找不到 {side} 点位字典')
    for point_id, item in items.items():
        if (type(point_id) is not int or not isinstance(item, dict)
                or not isinstance(item.get('pose'), list) or len(item['pose']) != 4):
            raise ValueError(f'{side}/{point_id}：点位格式应包含 pose: [x,y,qz,qw]')
    return items


def region_items(path, side):
    _, data, _ = read_document(path)
    entries = data.get('regions_' + side)
    if not isinstance(entries, list):
        raise ValueError(f'regions.yaml 中找不到 regions_{side} 列表')
    items = {}
    for item in entries:
        if not isinstance(item, dict) or type(item.get('id')) is not int:
            raise ValueError('区域条目必须包含整数 id')
        if item['id'] in items:
            raise ValueError(f'区域 id 重复：{item["id"]}')
        for key in ('x_min', 'x_max', 'y_min', 'y_max'):
            if type(item.get(key)) not in (float, int) or not math.isfinite(item[key]):
                raise ValueError(f'区域 {item["id"]} 的 {key} 必须为有限数值')
        items[item['id']] = item
    return items


def write_scalar_updates(path, original, replacements, expected):
    """只替换数值，保留其他内容、注释和布局，备份后原子写入。"""
    if any(isinstance(token, (yaml.tokens.AliasToken, yaml.tokens.AnchorToken))
           for token in yaml.scan(original)):
        raise ValueError('标定文件含 YAML 锚点/别名，请展开后再修改，避免联动其他点位')
    updated = original
    ranges = []
    for node, value in replacements:
        if not isinstance(node, ScalarNode) or not math.isfinite(value):
            raise ValueError('仅允许替换有限数值标量')
        ranges.append((node.start_mark.index, node.end_mark.index, f'{value:.3f}'))
    for start, end, value in sorted(ranges, reverse=True):
        updated = updated[:start] + value + updated[end:]
    if yaml.load(updated, Loader=UniqueKeyLoader) != expected:
        raise ValueError('更新校验失败：取消写入，避免修改其他字段')
    if path.read_text(encoding='utf-8') != original:
        raise RuntimeError('配置已被其他编辑器修改，请重新选择点位')
    if updated == original:
        return None
    backups = path.parent / '.point_backups'
    backups.mkdir(exist_ok=True)
    backup = backups / f'{path.name}.{time.time_ns()}-{uuid.uuid4().hex[:8]}.bak'
    backup.write_text(original, encoding='utf-8')
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    if path.read_text(encoding='utf-8') != updated:
        raise RuntimeError(f'写入后校验失败；原文件备份在 {backup}')
    return backup


def update_point_xy(path, side, point_id, x, y):
    point_items(path, side)
    text, data, tree = read_document(path)
    if point_id not in data[side]:
        raise ValueError(f'{side} 中找不到点位 {point_id}')
    pose = data[side][point_id]['pose']
    if any(v is None for v in pose):
        raise ValueError('动态目标由特殊任务计算，不能改成固定点')
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in pose):
        raise ValueError('原点位包含无效数值')
    node = mapping_value(mapping_value(mapping_value(tree, side), point_id), 'pose')
    if not isinstance(node, SequenceNode) or len(node.value) != 4:
        raise ValueError('pose 必须为四个数值的列表')
    expected = deepcopy(data)
    expected[side][point_id]['pose'][:2] = [round(x, 3), round(y, 3)]
    return write_scalar_updates(path, text, [(node.value[0], x), (node.value[1], y)], expected)


def update_region_center(path, side, region_id, x, y):
    region_items(path, side)
    text, data, tree = read_document(path)
    key = 'regions_' + side
    index = next((i for i, item in enumerate(data[key]) if item['id'] == region_id), None)
    if index is None:
        raise ValueError(f'{key} 中找不到区域 {region_id}')
    group = mapping_value(tree, key)
    if not isinstance(group, SequenceNode):
        raise ValueError('区域配置必须为列表')
    values = dict(x_min=x - REGION_HALF_SIZE, x_max=x + REGION_HALF_SIZE,
                  y_min=y - REGION_HALF_SIZE, y_max=y + REGION_HALF_SIZE)
    expected = deepcopy(data)
    expected[key][index].update({k: round(v, 3) for k, v in values.items()})
    return write_scalar_updates(
        path, text, [(mapping_value(group.value[index], k), v) for k, v in values.items()], expected)


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return 'q'


def choose_side():
    while True:
        value = ask('半场：1/r 红方，2/blue 蓝方，b 返回，q 退出：')
        if value in ('q', 'quit', 'exit'):
            raise KeyboardInterrupt
        if value in ('b', 'back'):
            return None
        if value in ('1', 'r', 'red', '红', '红方'):
            return 'red'
        if value in ('2', 'blue', '蓝', '蓝方'):
            return 'blue'
        print('输入无效。')


def show_items(items, is_region):
    print('\n ID    X/中心X    Y/中心Y    名称')
    for key, item in sorted(items.items()):
        if is_region:
            x, y = (item['x_min'] + item['x_max']) / 2, (item['y_min'] + item['y_max']) / 2
        else:
            x, y = item['pose'][:2]
        xy = '动态目标（不可标定）' if x is None or y is None else f'{x:10.3f} {y:10.3f}'
        print(f'{key:3} {xy}  {item.get("name", key)}')


def run_mode(collector, path, is_region, timeout):
    while True:
        side = choose_side()
        if side is None:
            return
        while True:
            items = region_items(path, side) if is_region else point_items(path, side)
            show_items(items, is_region)
            value = ask('选择编号（b 返回半场选择，q 退出）：')
            if value in ('q', 'quit', 'exit'):
                raise KeyboardInterrupt
            if value in ('b', 'back'):
                break
            try:
                selected = int(value)
                if selected not in items:
                    raise ValueError('编号不存在')
                if not is_region and any(v is None for v in items[selected]['pose']):
                    raise ValueError('动态目标不可标定，请选择固定点')
            except ValueError as exc:
                print(f'无法选择：{exc}')
                continue
            action = ask(f'已选 {side}/{selected}（{items[selected].get("name", selected)}）。'
                         '停稳后回车采集并保存，b 重新选点，q 退出：')
            if action == 'q':
                raise KeyboardInterrupt
            if action == 'b':
                continue
            if action not in ('', 'y', 'yes'):
                print('未开始采集，请重新选择。')
                continue
            try:
                print('正在接收 10 条新位姿……', flush=True)
                avg = collector.collect_next_average(timeout)
                print(f'采集用时 {collector.elapsed:.2f}s；逐项去掉最大/最小值，剩余 8 条平均：'
                      f'X={avg.x:.3f}, Y={avg.y:.3f}, Z={avg.z:.3f}, Yaw={avg.yaw:.3f}')
                update = update_region_center if is_region else update_point_xy
                backup = update(path, side, selected, avg.x, avg.y)
                print(f'已保存并校验：{path}' if backup else '坐标未变化。')
                if backup:
                    print(f'原文件备份：{backup}')
                if is_region:
                    print('区域为中心±0.6m。生效：ros2 run chairman_config sync --files odometry/regions.yaml；'
                          '重启 odometry 与 framework 功能进程。')
                else:
                    print('已保留朝向、名称和模式。生效：colcon build --packages-select framework；'
                          '重启 framework 功能进程。')
            except (OSError, ValueError, RuntimeError, yaml.YAMLError) as exc:
                print(f'本次标定未完成：{exc}')


def main(args=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args

    argv = sys.argv if args is None else [sys.argv[0], *args]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', help='chairman_lidar 项目根目录；默认由脚本位置定位')
    parser.add_argument('--topic', default='/odom_map', help='map 坐标的 custom_msg/PoseEuler 话题')
    parser.add_argument('--timeout', type=float, default=5.0, help='每批采集超时秒数，默认 5')
    options = parser.parse_args(remove_ros_args(args=argv)[1:])
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout 必须是正有限数值')
    try:
        points, regions = resolve_project_files(options.workspace)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print('================ chairman_lidar 点位标定 ================')
    print(f'采集：{options.topic}（map 位姿）；每批 10 条，去掉最大/最小值后平均。')
    print(f'定点文件：{points}\n区域文件：{regions}')
    print('请先启动导航与 odometry，等待定位就绪。每次采集前将机器人停稳。')
    rclpy.init(args=argv[1:], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    worker = None
    try:
        node = rclpy.create_node('point_calibrator')
        collector = PoseCollector(rclpy.ok)
        collector.attach(node, options.topic)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, name='point_pose_receiver', daemon=True)
        worker.start()
        while rclpy.ok():
            print('\n1：区域中心标定（中心±0.6m）  2：固定点标定（仅 x/y）  q：退出')
            choice = ask('选择模式：')
            if choice == 'q':
                break
            try:
                if choice == '1':
                    run_mode(collector, regions, True, options.timeout)
                elif choice == '2':
                    run_mode(collector, points, False, options.timeout)
                else:
                    print('输入无效。')
            except (OSError, ValueError, yaml.YAMLError) as exc:
                print(f'配置读取失败：{exc}')
    except KeyboardInterrupt:
        print('\n已退出。')
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if worker is not None:
            worker.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
