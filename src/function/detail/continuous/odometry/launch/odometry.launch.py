import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def points_file_path():
    """源码工作空间优先使用运行包内点位，支持 point 工具保存后刷新。"""
    share = Path(get_package_share_directory('odometry'))
    for parent in share.parents:
        path = parent / 'src/function/config/on_demand/points.yaml'
        if path.is_file():
            return str(path)
    # 仅部署 install 时，使用同一安装树中的 framework 配置。
    for parent in share.parents:
        for relative in ('framework/share/framework/config/on_demand/points.yaml',
                         'share/framework/config/on_demand/points.yaml'):
            path = parent / relative
            if path.is_file():
                return str(path)
    # 显示文件暂缺时由显示节点告警并重试，不阻止里程计输出。
    if share.parent.parent.name == 'odometry':
        return str(share.parents[2] / 'framework/share/framework/config/on_demand/points.yaml')
    return str(share.parent / 'framework/config/on_demand/points.yaml')


def generate_launch_description():
    selected = int(os.environ.get('SELECTED_POSE', '1'))
    config = Path(get_package_share_directory('odometry')) / 'config/initial_poses.yaml'
    poses = yaml.safe_load(config.read_text())['poses']
    if selected not in poses:
        raise ValueError('SELECTED_POSE 必须是 1、2、3、4')
    params = dict(poses[selected], initialpose_publish_after_stable_count=0)
    return LaunchDescription([
        DeclareLaunchArgument('points_file', default_value=points_file_path()),
        Node(package='odometry', executable='points_visualizer',
             name='points_visualizer', output='screen',
             parameters=[{'points_file': LaunchConfiguration('points_file')}]),
        Node(package='odometry', executable='odometry_transform_math',
             name='odometry_transform_math_node', output='screen', parameters=[params],
             remappings=[('odom_raw', '/Odometry'), ('odom_map', '/odom_map')]),
    ])
