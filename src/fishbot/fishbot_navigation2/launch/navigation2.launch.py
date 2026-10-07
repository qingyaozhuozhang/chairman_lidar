import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.parameter_descriptions

# =======================================================
# 🚗 [快捷修改区] 通过读取环境变量获取用户选择的初始位置！
# =======================================================
# 默认使用 1，如果系统环境变量中有 SELECTED_POSE，则读取该值
SELECTED_POSE = int(os.environ.get('SELECTED_POSE', '1'))

# 定义 4 个固定的初始位姿预设，格式为: {编号: [INIT_X, INIT_Y, INIT_Z, INIT_W]}
PRESET_POSES = {
    1: [-5.080,  -1.500, -0.707, 0.707],  # 红_武馆
    2: [5.915,   -5.457, 0.707,  0.707],  # 红_对抗
    3: [-5.080,  1.500,  0.707, 0.707],   # 蓝_武馆
    4: [5.915,   5.457,  -0.707, 0.707],  # 蓝_对抗
}

# 根据选择的编号，自动解析对应的 XYZW 坐标
INIT_X, INIT_Y, INIT_Z, INIT_W = PRESET_POSES[SELECTED_POSE]
# =======================================================


def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')
    livox_ros_driver2_dir = get_package_share_directory('livox_ros_driver2')
    terrain_analysis_dir = get_package_share_directory('terrain_analysis')
    terrain_analysis_ext_dir = get_package_share_directory('terrain_analysis_ext')

    rviz_config_dir = os.path.join(fishbot_navigation2_dir, 'config', 'rviz', 'nav2_default_view.rviz')
    map_yaml_path = os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml')
    pcd_file_path = os.path.join(fishbot_navigation2_dir, 'PCD', 'scans.pcd')
    nav2_param_path = os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml')
    urdf_path = os.path.join(fishbot_navigation2_dir, 'urdf', 'my_robot', 'my_robot.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    init_pose_str = (
        '{header: {frame_id: "map"}, pose: {pose: {'
        'position: {x: %s, y: %s, z: 0.0}, '
        'orientation: {x: 0.0, y: 0.0, z: %s, w: %s}}}}'
        % (INIT_X, INIT_Y, INIT_Z, INIT_W)
    )

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),

        # 0. 启动真实 Livox 雷达驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([livox_ros_driver2_dir, '/launch_ROS2', '/msg_MID360_launch.py']),
        ),

        # 0.5 直接发布机器人的 TF 和 URDF
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 1. 启动 Fast-LIO，发布 /cloud_registered_body 等点云
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([fast_lio_dir, '/launch', '/start.launch.py']),
            launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items(),
        ),

        # 2. TF 坐标系桥接
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init']
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_footprint',
            arguments=['-0.2365', '0', '0', '0', '0', '0', 'body', 'base_footprint']
        ),

        # 3. 点云坐标系转换：/cloud_registered_body -> /livox/lidar/pointcloud_odom
        # terrain_analysis / terrain_analysis_ext 订阅的就是 /livox/lidar/pointcloud_odom。
        launch_ros.actions.Node(
            package='odometry',
            executable='trans',
            name='pc2_transformer_node',
            output='screen',
            parameters=[{
                'target_frame': 'odom',
                'input_topic': '/cloud_registered_body',
                'output_topic': '/livox/lidar/pointcloud_odom',
                'use_sim_time': use_sim_time,
            }]
        ),

        # 4. 启动实时点云地形分析，发布 /terrain_map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([terrain_analysis_dir, '/launch', '/terrain_analysis.launch.py']),
        ),

        # 5. 启动扩展地形分析，发布 /terrain_map_ext
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([terrain_analysis_ext_dir, '/launch', '/terrain_analysis_ext.launch.py']),
        ),

        # 6. 将 /terrain_map 转成局部 /scan_local
        launch_ros.actions.Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_local',
            remappings=[('cloud_in', '/terrain_map'), ('scan', '/scan_local')],
            parameters=[{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.05,
                'min_height': 0.05,
                'max_height': 6.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.008765,
                'scan_time': 0.05,
                'range_min': 0.3,
                'range_max': 10.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'use_sim_time': use_sim_time,
            }]
        ),

        # 7. 将 /terrain_map_ext 转成全局 /scan_global
        launch_ros.actions.Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_global',
            remappings=[('cloud_in', '/terrain_map_ext'), ('scan', '/scan_global')],
            parameters=[{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.05,
                'min_height': 0.05,
                'max_height': 6.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.008765,
                'scan_time': 0.05,
                'range_min': 0.3,
                'range_max': 10.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'use_sim_time': use_sim_time,
            }]
        ),

        # 8. 启动 GICP 定位节点
        launch_ros.actions.Node(
            package='small_gicp_relocalization',
            executable='small_gicp_relocalization_node',
            name='small_gicp_relocalization',
            remappings = [("/tf", "tf"), ("/tf_static", "tf_static")],  
            parameters=[{
                'num_threads': 16,
                'num_neighbors': 10,
                'global_leaf_size': 0.05,
                'registered_leaf_size': 0.05,
                'max_dist_sq': 4.0,
                'map_frame': 'map1',
                'odom_frame': 'odom1',
                'base_frame': 'camera_init',
                'lidar_frame': 'camera_init',
                'robot_base_frame': 'base_footprint',
                'prior_pcd_file': pcd_file_path,
            }]
        ),

        # 9. 启动 Nav2 Map Server
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml_path, 'use_sim_time': use_sim_time}]
        ),
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}]
        ),

        # 10. 启动 Nav2 核心导航框架
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_param_path}.items(),
        ),

        # 11. 启动 RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])