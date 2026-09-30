import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    selected = int(os.environ.get('SELECTED_POSE', '1'))
    config = Path(get_package_share_directory('odometry')) / 'config/initial_poses.yaml'
    poses = yaml.safe_load(config.read_text())['poses']
    if selected not in poses:
        raise ValueError('SELECTED_POSE 必须是 1、2、3、4')
    params = dict(poses[selected], selected_pose=selected)
    return LaunchDescription([Node(
        package='odometry', executable='odometry_transform_math',
        name='odometry_transform_math_node', output='screen', parameters=[params],
        remappings=[('odom_raw', '/Odometry'), ('odom_map', '/odom_map')])])
