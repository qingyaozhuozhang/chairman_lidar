"""从原 preset_nav_node 拆分；算法与运动参数保持原行为。"""
from .imports import *
from .catalog import load_points, load_tasks, read_selected_pose


class NodeSetupMixin:
    def __init__(self):
        selected_pose = read_selected_pose()
        super().__init__('simple_nav_node')

        # 标志位管理
        self.cancel_current_task = False
        self.stair_cmd = -1
        self.in_stair_mode = False

        # stair 模式速度状态
        self.stair_current_vx = 0.0
        self.stair_current_vy = 0.0
        self.stair_current_omega = 0.0

        self.stair_target_vx = 0.0
        self.stair_target_vy = 0.0
        self.stair_target_speed = 0.0

        self.stair_start_x = None
        self.stair_start_y = None
        self.stair_start_yaw = None

        self.stair_global_dir_x = 0.0
        self.stair_global_dir_y = 0.0
        self.stair_cross_dir_x = 0.0
        self.stair_cross_dir_y = 0.0

        self.stair_last_time = self.get_clock().now()

        # 回调组
        self.reentrant_group = ReentrantCallbackGroup()
        self.exclusive_group = MutuallyExclusiveCallbackGroup()

        # 动作与话题发布
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.reentrant_group
        )
        self._spin_client = ActionClient(
            self,
            Spin,
            'spin',
            callback_group=self.reentrant_group
        )

        # 手写闭环速度直接转换为 /nav_speed_heading_data，避免和 Nav2 抢同一个速度话题。
        self.manual_control_active = False
        self.manual_control_reason = ""


        # Nav2 参数切换服务客户端：按预设点最后一位 1/2 动态切换 PID 与速度档位
        self.controller_param_client = self.create_client(
            SetParametersSrv,
            '/controller_server/set_parameters',
            callback_group=self.reentrant_group
        )
        self.velocity_smoother_param_client = self.create_client(
            SetParametersSrv,
            '/velocity_smoother/set_parameters',
            callback_group=self.reentrant_group
        )
        self.current_nav2_speed_profile = None

        # 订阅与服务
        self._odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10,
            callback_group=self.reentrant_group
        )
        # -11 抬升自检测使用 odometry_transform_math 发布的真实 map 位姿。
        self._odom_map_sub = self.create_subscription(
            PoseEuler,
            '/odom_map',
            self.odom_map_callback,
            10,
            callback_group=self.reentrant_group
        )
        self._stair_sub = self.create_subscription(
            Int8,
            'nav_topic',
            self.stair_callback,
            10,
            callback_group=self.reentrant_group
        )
        self.stop_sub = self.create_subscription(
            Empty,
            '/emergency_stop',
            self.stop_callback,
            10,
            callback_group=self.reentrant_group
        )
        self._srv = self.create_service(
            SetNavTarget,
            '/set_nav_target',
            self.srv_callback,
            callback_group=self.reentrant_group
        )

        self.data_pub = self.create_publisher(SpeedHeading, '/nav_speed_heading_data', 10)
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10,
            callback_group=self.reentrant_group
        )

        # TF2 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 定时器管理
        self.dt = 1.0 / 50.0
        self.tracker_timer = self.create_timer(
            self.dt,
            self.tracker_timer_callback,
            callback_group=self.reentrant_group
        )

        self.stair_timer = self.create_timer(
            self.dt,
            self.stair_timer_callback,
            callback_group=self.reentrant_group
        )

        # 卡尔曼滤波器初始化
        self.kf_x = np.zeros((6, 1))
        self.kf_P = np.eye(6) * 1.0

        self.F = np.eye(6)
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt
        self.F[3, 3] = 0.8
        self.F[4, 4] = 0.8
        self.F[5, 5] = 0.8

        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.Q = np.eye(6) * 0.50
        self.R = np.eye(3) * 0.1

        self.latest_z = None
        self.new_meas_available = False
        self.is_active = False

        # Nav2 /cmd_vel 转发只允许由 execute_nav2_goal() 显式开启/结束。
        # 不再监听 Nav2 status、不再根据 /cmd_vel=0、不再用超时自动清零。
        self.nav_cmd_tracking_enabled = False
        self.current_nav_goal_handle = None

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # /odom_map 是 odometry_transform_math_launch.py 输出的真实 map 位姿。
        # -11 抬升自检测只使用这里的 z 值判断高度变化。
        self.odom_map_x = None
        self.odom_map_y = None
        self.odom_map_z = None
        self.odom_map_yaw = None
        self.odom_map_last_time = None

        # 即使 simple_nav_node 不是通过 --params-file 启动，也主动从 fishbot_navigation2/config/nav2_params.yaml
        # 读取 Nav2 档位参数作为默认值，方便直接在 YAML 里调 1/2 两套参数。
        self.yaml_default_params = self.load_simple_nav_yaml_defaults()

        # ================= 加载运动控制参数 =================
        self.load_motion_params()
        self.build_nav2_speed_profiles()

        # ================= 加载梅林区域配置 =================
        PRESET_GOALS_RED, PRESET_GOALS_BLUE = load_points(self.mode_configs)
        self.task_configs = load_tasks(PRESET_GOALS_RED, PRESET_GOALS_BLUE)

        # ================= 根据 selected_pose 选择预设点 =================
        if selected_pose in [1, 2]:
            self.PRESET_GOALS = PRESET_GOALS_RED
            team_name = "红方"
        else:
            self.PRESET_GOALS = PRESET_GOALS_BLUE
            team_name = "蓝方"

        self.merlin_regions = {}

        try:
            pkg_share = get_package_share_directory("odometry")
            yaml_path = os.path.join(pkg_share, "config", "regions.yaml")

            with open(yaml_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)

            region_key = "regions_red" if selected_pose in [1, 2] else "regions_blue"

            if region_key in config:
                for node_cfg in config[region_key]:
                    rid = node_cfg['id']
                    if 1 <= rid <= 13:
                        self.merlin_regions[rid] = [
                            node_cfg['x_min'],
                            node_cfg['x_max'],
                            node_cfg['y_min'],
                            node_cfg['y_max'],
                            node_cfg['name']
                        ]

                self.get_logger().info(
                    f"成功从 YAML 加载了 {len(self.merlin_regions)} 个梅林区域 ({region_key})"
                )
            else:
                self.get_logger().error(f"YAML 中找不到 {region_key} 配置！")

        except Exception as e:
            self.get_logger().error(f"读取 regions.yaml 失败: {e}")

        self.get_logger().info(f"场地编号: {selected_pose} ({team_name})")
        self.get_logger().info("Preset Nav Service 已全面启动！")
