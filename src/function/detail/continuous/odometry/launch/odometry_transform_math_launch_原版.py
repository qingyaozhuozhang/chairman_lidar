import os
from launch import LaunchDescription
from launch_ros.actions import Node
# 地图原点预设编号。
SELECTED_MAP = 1

# 雷达起点编号由 SELECTED_POSE 指定。
SELECTED_LIDAR = int(os.environ.get('SELECTED_POSE', '1'))
# 地图原点：位置单位 m，朝向使用四元数。
PRESET_MAPS = {
    1: {'map_origin_x': -5.543, 'map_origin_y': -5.998, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0},
    2: {'map_origin_x': -5.548, 'map_origin_y': -0.007, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0}
}

# 雷达初始位姿：场地编号 1～4。
PRESET_LIDARS = {
    1: {'lidar_init_x': -5.082, 'lidar_init_y': -1.500, 'lidar_init_z': 0.0, 'lidar_init_qz': -0.707, 'lidar_init_qw': 0.707},
    2: {'lidar_init_x': 1.500, 'lidar_init_y':  2.000, 'lidar_init_z': 0.0, 'lidar_init_qz':  0.000, 'lidar_init_qw': 1.000},
    3: {'lidar_init_x': 10.000, 'lidar_init_y': 5.000, 'lidar_init_z': 0.0, 'lidar_init_qz':  0.000, 'lidar_init_qw': 1.000},
    4: {'lidar_init_x': 0.000, 'lidar_init_y':  0.000, 'lidar_init_z': 0.0, 'lidar_init_qz':  0.000, 'lidar_init_qw': 1.000}
}

def generate_launch_description():
    
    # 自动合并选中的地图和雷达参数
    current_params = {}
    current_params.update(PRESET_MAPS[SELECTED_MAP])
    current_params.update(PRESET_LIDARS[SELECTED_LIDAR])

    # 向节点传递场地编号，用于选择区域配置。
    current_params['selected_pose'] = SELECTED_LIDAR

    odometry_transform_math_node = Node(
        package='odometry',
        executable='odometry_transform_math', 
        name='odometry_transform_math_node',
        output='screen',
        parameters=[current_params], 
        remappings=[
            ('odom_raw', '/Odometry'), 
            ('odom_map', '/odom_map')  
        ]
    )

    return LaunchDescription([
        odometry_transform_math_node
    ])