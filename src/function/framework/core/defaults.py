"""从原 preset_nav_node 拆分；算法与运动参数保持原行为。"""
from .imports import *


class DefaultsMixin:
    def load_simple_nav_yaml_defaults(self):
        """读取 fishbot_navigation2/config/nav2_params.yaml 中 simple_nav_node 的参数作为默认值。"""
        try:
            pkg_share = get_package_share_directory("fishbot_navigation2")
            yaml_path = os.path.join(pkg_share, "config", "nav2_params.yaml")

            with open(yaml_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}

            defaults = (
                config.get("simple_nav_node", {})
                .get("ros__parameters", {})
            )
            from .catalog import load_modes
            self.mode_configs = load_modes()
            for mode in self.mode_configs.values():
                defaults.update(mode.get('simple_nav_node', {}))
            return defaults

        except Exception as e:
            self.get_logger().warn(f"⚠️ 无法读取 nav2_params.yaml 中的 simple_nav_node 默认参数，将使用代码内置默认值: {e}")
            raise RuntimeError(f'功能参数加载失败: {e}') from e

    def declare_and_get(self, name, default_value):
        """声明并读取 ROS2 参数，主要用于 Nav2 速度/PID 档位配置。"""
        yaml_default = self.yaml_default_params.get(name, default_value)
        self.declare_parameter(name, yaml_default)
        return self.get_parameter(name).value

    def make_parameter_msg(self, name, value):
        """把 Python 值转换成 /set_parameters 服务需要的 Parameter 消息。"""
        param = ParameterMsg()
        param.name = name
        param.value = ParameterValue()

        if isinstance(value, bool):
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        elif isinstance(value, int) and not isinstance(value, bool):
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = int(value)
        elif isinstance(value, float):
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
        elif isinstance(value, (list, tuple)):
            if all(isinstance(v, (int, float)) for v in value):
                param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param.value.double_array_value = [float(v) for v in value]
            elif all(isinstance(v, str) for v in value):
                param.value.type = ParameterType.PARAMETER_STRING_ARRAY
                param.value.string_array_value = list(value)
            else:
                raise TypeError(f"参数 {name} 的数组类型不支持: {value}")
        elif isinstance(value, str):
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = value
        else:
            raise TypeError(f"参数 {name} 的类型不支持: {type(value)}")

        return param

    def wait_future_done(self, future, timeout_sec=1.0):
        start_time = self.get_clock().now()

        while rclpy.ok() and not future.done():
            if self.cancel_current_task:
                return False

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return False

            time.sleep(0.01)

        return future.done()

    def build_nav2_speed_profiles(self):
        """从 nav2_params.yaml/simple_nav_node 下读取 Nav2 PID、速度与到点容差参数。

        说明：
          - 预设点最后一位可以写 1 / 2 / 3。
          - 1 = 慢速固定档。
          - 2 = 动态快速档：距离目标点 > nav2_profile_2_switch_distance 时使用 2_sprint；
                距离目标点 <= nav2_profile_2_switch_distance 后自动切回 2 精调档。
          - 3 = 中档速度；send_goal_and_wait() 会对模式 3 启用提前转正两段式导航。
        """
        self.nav2_profile_2_switch_distance = float(
            self.declare_and_get("nav2_profile_2_switch_distance", 0.50)
        )
        self.nav2_profile_2_switch_check_period = float(
            self.declare_and_get("nav2_profile_2_switch_check_period", 0.05)
        )
        self.nav2_profile_2_sprint_key = "2_sprint"
        self.nav2_dynamic_last_check_time = None

        # 模式 3 专用：提前转正两段式导航参数
        # 预设点最后一位为 3 时：
        #   - 当前距离目标点 > nav2_profile_3_pre_align_distance：
        #       先到目标点前方 nav2_profile_3_pre_align_distance 米处转正，再进入真实目标点。
        #   - 当前距离目标点 <= nav2_profile_3_pre_align_distance：
        #       先在当前位置原地转正，再进入真实目标点。
        # 不再使用第一段提前转正超时参数，避免额外卡死/失败条件。
        self.nav2_profile_3_pre_align_enabled = bool(
            self.declare_and_get("nav2_profile_3_pre_align_enabled", True)
        )
        self.nav2_profile_3_pre_align_distance = float(
            self.declare_and_get("nav2_profile_3_pre_align_distance", 0.80)
        )
        # 模式 3 第一段提前转正点不再死等 Nav2 完全成功。
        # 到提前点附近后，只要 yaw 差不多，或者在提前点附近调整超过限定时间，就直接进入真实目标点。
        self.nav2_profile_3_pre_align_release_xy_tolerance = float(
            self.declare_and_get("nav2_profile_3_pre_align_release_xy_tolerance", 0.08)
        )
        self.nav2_profile_3_pre_align_release_yaw_tolerance = float(
            self.declare_and_get("nav2_profile_3_pre_align_release_yaw_tolerance", 0.12)
        )
        self.nav2_profile_3_pre_align_near_adjust_timeout = float(
            self.declare_and_get("nav2_profile_3_pre_align_near_adjust_timeout", 1.0)
        )

        self.NAV2_SPEED_PROFILES = {
            1: {
                "name": self.declare_and_get("nav2_profile_1_name", "慢速"),
                "controller": {
                    "general_goal_checker.xy_goal_tolerance": self.declare_and_get("nav2_profile_1_xy_goal_tolerance", 0.01),
                    "general_goal_checker.yaw_goal_tolerance": self.declare_and_get("nav2_profile_1_yaw_goal_tolerance", 0.01),
                    "FollowPath.use_rotate_to_heading": self.declare_and_get("nav2_profile_1_use_rotate_to_heading", True),
                    "FollowPath.rotate_to_goal_heading_only": self.declare_and_get("nav2_profile_1_rotate_to_goal_heading_only", True),
                    "FollowPath.yaw_hold_tolerance": self.declare_and_get("nav2_profile_1_yaw_hold_tolerance", 0.08),
                    "FollowPath.v_linear_min": self.declare_and_get("nav2_profile_1_v_linear_min", -1.0),
                    "FollowPath.v_linear_max": self.declare_and_get("nav2_profile_1_v_linear_max", 1.0),
                    "FollowPath.v_angular_min": self.declare_and_get("nav2_profile_1_v_angular_min", -1.0),
                    "FollowPath.v_angular_max": self.declare_and_get("nav2_profile_1_v_angular_max", 1.0),
                    "FollowPath.translation_kp": self.declare_and_get("nav2_profile_1_translation_kp", 3.0),
                    "FollowPath.translation_ki": self.declare_and_get("nav2_profile_1_translation_ki", 0.0),
                    "FollowPath.translation_kd": self.declare_and_get("nav2_profile_1_translation_kd", 0.0),
                    "FollowPath.enable_rotation": self.declare_and_get("nav2_profile_1_enable_rotation", True),
                    "FollowPath.rotation_kp": self.declare_and_get("nav2_profile_1_rotation_kp", 1.0),
                    "FollowPath.rotation_ki": self.declare_and_get("nav2_profile_1_rotation_ki", 0.0),
                    "FollowPath.rotation_kd": self.declare_and_get("nav2_profile_1_rotation_kd", 0.0),
                    "FollowPath.min_approach_linear_velocity": self.declare_and_get("nav2_profile_1_min_approach_linear_velocity", 0.05),
                    "FollowPath.min_approach_angular_velocity": self.declare_and_get("nav2_profile_1_min_approach_angular_velocity", 0.05),
                    "FollowPath.approach_velocity_scaling_dist": self.declare_and_get("nav2_profile_1_approach_velocity_scaling_dist", 0.30),
                },
                "velocity_smoother": {
                    "max_velocity": self.declare_and_get("nav2_profile_1_max_velocity", [1.0, 1.0, 1.0]),
                    "min_velocity": self.declare_and_get("nav2_profile_1_min_velocity", [-1.0, -1.0, -1.0]),
                    "max_accel": self.declare_and_get("nav2_profile_1_max_accel", [0.5, 0.5, 1.0]),
                    "max_decel": self.declare_and_get("nav2_profile_1_max_decel", [-0.5, -0.5, -1.0]),
                },
            },

            # 模式 2 近距离精调档：50cm 内自动切到这里，保证最后到点精度。
            2: {
                "name": self.declare_and_get("nav2_profile_2_name", "快速"),
                "controller": {
                    "general_goal_checker.xy_goal_tolerance": self.declare_and_get("nav2_profile_2_xy_goal_tolerance", 0.01),
                    "general_goal_checker.yaw_goal_tolerance": self.declare_and_get("nav2_profile_2_yaw_goal_tolerance", 0.01),
                    "FollowPath.use_rotate_to_heading": self.declare_and_get("nav2_profile_2_use_rotate_to_heading", True),
                    "FollowPath.rotate_to_goal_heading_only": self.declare_and_get("nav2_profile_2_rotate_to_goal_heading_only", True),
                    "FollowPath.yaw_hold_tolerance": self.declare_and_get("nav2_profile_2_yaw_hold_tolerance", 0.01),
                    "FollowPath.v_linear_min": self.declare_and_get("nav2_profile_2_v_linear_min", -1.5),
                    "FollowPath.v_linear_max": self.declare_and_get("nav2_profile_2_v_linear_max", 1.5),
                    "FollowPath.v_angular_min": self.declare_and_get("nav2_profile_2_v_angular_min", -2.0),
                    "FollowPath.v_angular_max": self.declare_and_get("nav2_profile_2_v_angular_max", 2.0),
                    "FollowPath.translation_kp": self.declare_and_get("nav2_profile_2_translation_kp", 4.0),
                    "FollowPath.translation_ki": self.declare_and_get("nav2_profile_2_translation_ki", 0.0),
                    "FollowPath.translation_kd": self.declare_and_get("nav2_profile_2_translation_kd", 0.0),
                    "FollowPath.enable_rotation": self.declare_and_get("nav2_profile_2_enable_rotation", True),
                    "FollowPath.rotation_kp": self.declare_and_get("nav2_profile_2_rotation_kp", 2.5),
                    "FollowPath.rotation_ki": self.declare_and_get("nav2_profile_2_rotation_ki", 0.0),
                    "FollowPath.rotation_kd": self.declare_and_get("nav2_profile_2_rotation_kd", 0.0),
                    "FollowPath.min_approach_linear_velocity": self.declare_and_get("nav2_profile_2_min_approach_linear_velocity", 0.08),
                    "FollowPath.min_approach_angular_velocity": self.declare_and_get("nav2_profile_2_min_approach_angular_velocity", 0.10),
                    "FollowPath.approach_velocity_scaling_dist": self.declare_and_get("nav2_profile_2_approach_velocity_scaling_dist", 0.30),
                },
                "velocity_smoother": {
                    "max_velocity": self.declare_and_get("nav2_profile_2_max_velocity", [1.5, 1.5, 2.0]),
                    "min_velocity": self.declare_and_get("nav2_profile_2_min_velocity", [-1.5, -1.5, -2.0]),
                    "max_accel": self.declare_and_get("nav2_profile_2_max_accel", [2.0, 2.0, 2.5]),
                    "max_decel": self.declare_and_get("nav2_profile_2_max_decel", [-2.0, -2.0, -2.5]),
                },
            },

            # 模式 3 = 中档速度：专用提前转正两段式导航使用的速度档。
            3: {
                "name": self.declare_and_get("nav2_profile_3_name", "中档速度"),
                "controller": {
                    "general_goal_checker.xy_goal_tolerance": self.declare_and_get("nav2_profile_3_xy_goal_tolerance", 0.01),
                    "general_goal_checker.yaw_goal_tolerance": self.declare_and_get("nav2_profile_3_yaw_goal_tolerance", 0.01),
                    "FollowPath.use_rotate_to_heading": self.declare_and_get("nav2_profile_3_use_rotate_to_heading", True),
                    "FollowPath.rotate_to_goal_heading_only": self.declare_and_get("nav2_profile_3_rotate_to_goal_heading_only", True),
                    "FollowPath.yaw_hold_tolerance": self.declare_and_get("nav2_profile_3_yaw_hold_tolerance", 0.01),
                    "FollowPath.v_linear_min": self.declare_and_get("nav2_profile_3_v_linear_min", -1.5),
                    "FollowPath.v_linear_max": self.declare_and_get("nav2_profile_3_v_linear_max", 1.5),
                    "FollowPath.v_angular_min": self.declare_and_get("nav2_profile_3_v_angular_min", -1.5),
                    "FollowPath.v_angular_max": self.declare_and_get("nav2_profile_3_v_angular_max", 1.5),
                    "FollowPath.translation_kp": self.declare_and_get("nav2_profile_3_translation_kp", 4.0),
                    "FollowPath.translation_ki": self.declare_and_get("nav2_profile_3_translation_ki", 0.0),
                    "FollowPath.translation_kd": self.declare_and_get("nav2_profile_3_translation_kd", 0.0),
                    "FollowPath.enable_rotation": self.declare_and_get("nav2_profile_3_enable_rotation", True),
                    "FollowPath.rotation_kp": self.declare_and_get("nav2_profile_3_rotation_kp", 1.5),
                    "FollowPath.rotation_ki": self.declare_and_get("nav2_profile_3_rotation_ki", 0.0),
                    "FollowPath.rotation_kd": self.declare_and_get("nav2_profile_3_rotation_kd", 0.0),
                    "FollowPath.min_approach_linear_velocity": self.declare_and_get("nav2_profile_3_min_approach_linear_velocity", 0.08),
                    "FollowPath.min_approach_angular_velocity": self.declare_and_get("nav2_profile_3_min_approach_angular_velocity", 0.05),
                    "FollowPath.approach_velocity_scaling_dist": self.declare_and_get("nav2_profile_3_approach_velocity_scaling_dist", 0.30),
                },
                "velocity_smoother": {
                    "max_velocity": self.declare_and_get("nav2_profile_3_max_velocity", [1.5, 1.5, 1.5]),
                    "min_velocity": self.declare_and_get("nav2_profile_3_min_velocity", [-1.5, -1.5, -1.5]),
                    "max_accel": self.declare_and_get("nav2_profile_3_max_accel", [2.0, 2.0, 1.5]),
                    "max_decel": self.declare_and_get("nav2_profile_3_max_decel", [-2.0, -2.0, -1.5]),
                },
            },

            # 模式 2 远距离冲刺档：距离目标点 50cm 以外自动使用。
            self.nav2_profile_2_sprint_key: {
                "name": self.declare_and_get("nav2_profile_2_sprint_name", "快速冲刺"),
                "controller": {
                    "general_goal_checker.xy_goal_tolerance": self.declare_and_get("nav2_profile_2_sprint_xy_goal_tolerance", 0.03),
                    "general_goal_checker.yaw_goal_tolerance": self.declare_and_get("nav2_profile_2_sprint_yaw_goal_tolerance", 0.03),
                    "FollowPath.use_rotate_to_heading": self.declare_and_get("nav2_profile_2_sprint_use_rotate_to_heading", True),
                    "FollowPath.rotate_to_goal_heading_only": self.declare_and_get("nav2_profile_2_sprint_rotate_to_goal_heading_only", True),
                    "FollowPath.yaw_hold_tolerance": self.declare_and_get("nav2_profile_2_sprint_yaw_hold_tolerance", 0.03),
                    "FollowPath.v_linear_min": self.declare_and_get("nav2_profile_2_sprint_v_linear_min", -3.0),
                    "FollowPath.v_linear_max": self.declare_and_get("nav2_profile_2_sprint_v_linear_max", 3.0),
                    "FollowPath.v_angular_min": self.declare_and_get("nav2_profile_2_sprint_v_angular_min", -2.5),
                    "FollowPath.v_angular_max": self.declare_and_get("nav2_profile_2_sprint_v_angular_max", 2.5),
                    "FollowPath.translation_kp": self.declare_and_get("nav2_profile_2_sprint_translation_kp", 10.0),
                    "FollowPath.translation_ki": self.declare_and_get("nav2_profile_2_sprint_translation_ki", 0.0),
                    "FollowPath.translation_kd": self.declare_and_get("nav2_profile_2_sprint_translation_kd", 0.0),
                    "FollowPath.enable_rotation": self.declare_and_get("nav2_profile_2_sprint_enable_rotation", True),
                    "FollowPath.rotation_kp": self.declare_and_get("nav2_profile_2_sprint_rotation_kp", 3.0),
                    "FollowPath.rotation_ki": self.declare_and_get("nav2_profile_2_sprint_rotation_ki", 0.0),
                    "FollowPath.rotation_kd": self.declare_and_get("nav2_profile_2_sprint_rotation_kd", 0.0),
                    "FollowPath.min_approach_linear_velocity": self.declare_and_get("nav2_profile_2_sprint_min_approach_linear_velocity", 0.12),
                    "FollowPath.min_approach_angular_velocity": self.declare_and_get("nav2_profile_2_sprint_min_approach_angular_velocity", 0.10),
                    "FollowPath.approach_velocity_scaling_dist": self.declare_and_get("nav2_profile_2_sprint_approach_velocity_scaling_dist", 0.18),
                },
                "velocity_smoother": {
                    "max_velocity": self.declare_and_get("nav2_profile_2_sprint_max_velocity", [3.0, 3.0, 2.5]),
                    "min_velocity": self.declare_and_get("nav2_profile_2_sprint_min_velocity", [-3.0, -3.0, -2.5]),
                    "max_accel": self.declare_and_get("nav2_profile_2_sprint_max_accel", [6.0, 6.0, 3.5]),
                    "max_decel": self.declare_and_get("nav2_profile_2_sprint_max_decel", [-7.0, -7.0, -4.0]),
                },
            },
        }

    def normalize_nav2_speed_profile_key(self, speed_profile):
        """把预设点最后一位或内部动态档位转换成 NAV2_SPEED_PROFILES 的 key。"""
        if speed_profile is None:
            return 1

        if isinstance(speed_profile, str):
            value = speed_profile.strip()
            if value in self.NAV2_SPEED_PROFILES:
                return value
            try:
                return int(value)
            except Exception:
                return value

        try:
            return int(speed_profile)
        except Exception:
            return speed_profile

    def load_motion_params(self):
        """通过 ROS2 参数系统声明并加载所有运动控制参数"""
        self.declare_parameter("max_accel", [0.5, 0.5, 3.0])
        self.MAX_ACCEL = self.get_parameter("max_accel").value

        self.declare_parameter("max_decel", [0.5, 0.5, 3.0])
        self.MAX_DECEL = self.get_parameter("max_decel").value

        self.declare_parameter("max_vel_linear", 0.5)
        self.MAX_VEL_LINEAR = self.get_parameter("max_vel_linear").value

        self.declare_parameter("min_vel_linear", 0.05)
        self.MIN_VEL_LINEAR = self.get_parameter("min_vel_linear").value

        self.declare_parameter("kp_linear", 0.5)
        self.KP_LINEAR = self.get_parameter("kp_linear").value

        self.declare_parameter("error_tolerance_dist", 0.005)
        self.ERROR_TOLERANCE_DIST = self.get_parameter("error_tolerance_dist").value

        self.declare_parameter("max_vel_angular", 1.5)
        self.MAX_VEL_ANGULAR = self.get_parameter("max_vel_angular").value

        self.declare_parameter("min_vel_angular", 0.1)
        self.MIN_VEL_ANGULAR = self.get_parameter("min_vel_angular").value

        self.declare_parameter("kp_angular", 3.0)
        self.KP_ANGULAR = self.get_parameter("kp_angular").value

        self.declare_parameter("kp_yaw_correct", 1.0)
        self.KP_YAW_CORRECT = self.get_parameter("kp_yaw_correct").value

        self.declare_parameter("yaw_correct_max_ratio", 0.5)
        self.YAW_CORRECT_MAX_RATIO = self.get_parameter("yaw_correct_max_ratio").value

        self.declare_parameter("error_tolerance_yaw", 0.01)
        self.ERROR_TOLERANCE_YAW = self.get_parameter("error_tolerance_yaw").value

        self.declare_parameter("kp_cross_track", 1.0)
        self.KP_CROSS_TRACK = self.get_parameter("kp_cross_track").value

        self.declare_parameter("max_cross_track_vel", 0.10)
        self.MAX_CROSS_TRACK_VEL = self.get_parameter("max_cross_track_vel").value

        self.declare_parameter("min_cross_track_vel", 0.01)
        self.MIN_CROSS_TRACK_VEL = self.get_parameter("min_cross_track_vel").value

        self.declare_parameter("error_tolerance_cross", 0.005)
        self.ERROR_TOLERANCE_CROSS = self.get_parameter("error_tolerance_cross").value

        self.declare_parameter("control_period", 0.02)
        self.CONTROL_PERIOD = self.get_parameter("control_period").value

        # --- -11 抬升自检测专用参数 ---
        # 当前 /odom_map.z - 启动时 z0 >= lift_check_height 时，-11 返回成功。
        # 只判断正向抬升，不使用 abs，不主动清零底盘速度。
        self.LIFT_CHECK_HEIGHT = float(self.declare_and_get("lift_check_height", 0.50))

        # --- -2 上坡 / 全局 X 轴直走专用参数 ---
        self.declare_parameter("uphill_global_x_distance", 2.7)
        self.UPHILL_GLOBAL_X_DISTANCE = self.get_parameter("uphill_global_x_distance").value

        self.declare_parameter("uphill_max_accel", [1.0, 1.0, 1.0])
        self.UPHILL_MAX_ACCEL = self.get_parameter("uphill_max_accel").value

        self.declare_parameter("uphill_max_decel", [2.0, 2.0, 1.0])
        self.UPHILL_MAX_DECEL = self.get_parameter("uphill_max_decel").value

        self.declare_parameter("uphill_max_vel_linear", 2.0)
        self.UPHILL_MAX_VEL_LINEAR = self.get_parameter("uphill_max_vel_linear").value

        self.declare_parameter("uphill_min_vel_linear", 0.05)
        self.UPHILL_MIN_VEL_LINEAR = self.get_parameter("uphill_min_vel_linear").value

        self.declare_parameter("uphill_kp_linear", 2.0)
        self.UPHILL_KP_LINEAR = self.get_parameter("uphill_kp_linear").value

        self.declare_parameter("uphill_error_tolerance_dist", 0.01)
        self.UPHILL_ERROR_TOLERANCE_DIST = self.get_parameter("uphill_error_tolerance_dist").value

        self.declare_parameter("uphill_max_vel_angular", 1.0)
        self.UPHILL_MAX_VEL_ANGULAR = self.get_parameter("uphill_max_vel_angular").value

        self.declare_parameter("uphill_min_vel_angular", 0.05)
        self.UPHILL_MIN_VEL_ANGULAR = self.get_parameter("uphill_min_vel_angular").value

        self.declare_parameter("uphill_kp_yaw_correct", 1.0)
        self.UPHILL_KP_YAW_CORRECT = self.get_parameter("uphill_kp_yaw_correct").value

        self.declare_parameter("uphill_yaw_correct_max_ratio", 0.5)
        self.UPHILL_YAW_CORRECT_MAX_RATIO = self.get_parameter("uphill_yaw_correct_max_ratio").value

        self.declare_parameter("uphill_error_tolerance_yaw", 0.01)
        self.UPHILL_ERROR_TOLERANCE_YAW = self.get_parameter("uphill_error_tolerance_yaw").value

        self.declare_parameter("uphill_kp_cross_track", 1.5)
        self.UPHILL_KP_CROSS_TRACK = self.get_parameter("uphill_kp_cross_track").value

        self.declare_parameter("uphill_max_cross_track_vel", 1.0)
        self.UPHILL_MAX_CROSS_TRACK_VEL = self.get_parameter("uphill_max_cross_track_vel").value

        self.declare_parameter("uphill_min_cross_track_vel", 0.05)
        self.UPHILL_MIN_CROSS_TRACK_VEL = self.get_parameter("uphill_min_cross_track_vel").value

        self.declare_parameter("uphill_error_tolerance_cross", 0.01)
        self.UPHILL_ERROR_TOLERANCE_CROSS = self.get_parameter("uphill_error_tolerance_cross").value

        self.declare_parameter("uphill_global_x_timeout", 20.0)
        self.UPHILL_GLOBAL_X_TIMEOUT = self.get_parameter("uphill_global_x_timeout").value
