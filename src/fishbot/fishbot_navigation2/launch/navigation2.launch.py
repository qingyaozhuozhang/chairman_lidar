import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.parameter_descriptions

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio')
    livox_ros_driver2_dir = get_package_share_directory('livox_ros_driver2')
    terrain_analysis_dir = get_package_share_directory('terrain_analysis')
    terrain_analysis_ext_dir = get_package_share_directory('terrain_analysis_ext')
    gicp_dir = get_package_share_directory('small_gicp_relocalization')

    rviz_config_dir = os.path.join(fishbot_navigation2_dir, 'config', 'rviz', 'nav2_default_view.rviz')
    map_yaml_path = os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml')
    pcd_file_path = os.path.join(fishbot_navigation2_dir, 'PCD', 'final2.pcd')
    nav2_param_path = os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml')
    urdf_path = os.path.join(fishbot_navigation2_dir, 'urdf', 'my_robot', 'my_robot.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

        # 8. 根据 SELECTED_POSE 初始化 GICP，由同一节点持续发布 map -> odom。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gicp_dir, '/launch', '/small_gicp_relocalization_launch.py']),
            launch_arguments={'prior_pcd_file': pcd_file_path, 'use_sim_time': use_sim_time}.items(),
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
