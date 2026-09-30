import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


# =========================
# Livox driver 参数
# =========================
xfer_format = 1      # 0: PointCloud2, 1: Livox 自定义点云格式
multi_topic = 0      # 0: 所有 LiDAR 共用一个话题
data_src = 0         # 0: real lidar
publish_freq = 10.0
output_type = 0
frame_id = 'livox_frame'

lvx_file_path = ''
cmdline_bd_code = 'livox0000000001'

user_config_path = os.path.join(get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json')

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
]


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    # 关键参数：
    # 默认 false，表示 start.launch.py 默认不启动 livox_ros_driver2_node
    # 因为 navigation2.launch.py 里面已经启动了 msg_MID360_launch.py
    start_livox_driver = LaunchConfiguration('start_livox_driver')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=default_config_path,
        description='FAST-LIO config file path'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value='mid360.yaml',
        description='FAST-LIO config file name'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Use RViz to monitor FAST-LIO'
    )

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    declare_start_livox_driver_cmd = DeclareLaunchArgument(
        'start_livox_driver',
        default_value='false',
        description='Whether start.launch.py should start livox_ros_driver2_node'
    )

    # 可选启动 Livox 驱动
    # 默认不会启动，避免和 navigation2.launch.py 里的 msg_MID360_launch.py 重复
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=IfCondition(start_livox_driver)
    )

    # FAST-LIO 主节点
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_start_livox_driver_cmd)

    ld.add_action(livox_driver)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld