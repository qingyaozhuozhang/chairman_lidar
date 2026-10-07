"""固定点导航模式的实现。"""
from framework.core.imports import *

class DynamicProfileMixin:
    def get_nav2_distance_to_goal(self, target_x, target_y, timeout_sec=0.02):
        """使用 TF 的 map -> base_footprint 坐标计算当前距离目标点的平面距离。"""
        pose = self.get_current_map_pose(timeout_sec=timeout_sec, log_error=False)
        if pose is None:
            return None

        current_map_x, current_map_y, _ = pose
        return math.hypot(float(target_x) - current_map_x, float(target_y) - current_map_y)

    def select_initial_nav2_profile(self, speed_profile, target_x, target_y, desc):
        """导航开始前选择实际应用的 Nav2 档位。模式 2 会根据起点距离决定冲刺/精调。"""
        profile_key = self.normalize_nav2_speed_profile_key(speed_profile)

        if profile_key != 2:
            return profile_key

        distance = self.get_nav2_distance_to_goal(target_x, target_y, timeout_sec=0.05)
        if distance is not None and distance <= self.nav2_profile_2_switch_distance:
            return 2

        if distance is None:
            self.get_logger().warn(
                f'[模式2] 暂时无法获取 map 位姿，先使用冲刺档；'
                f'位姿恢复后按 {self.nav2_profile_2_switch_distance:.2f}m 阈值切换精调'
            )

        return self.nav2_profile_2_sprint_key

    def maybe_switch_nav2_profile_2_by_distance(self, requested_speed_profile, target_x, target_y, desc):
        """模式 2 行驶中距离目标点 50cm 内时，自动从快速冲刺切回快速精调。"""
        requested_key = self.normalize_nav2_speed_profile_key(requested_speed_profile)
        if requested_key != 2:
            return

        # 已经切到精调档后不再切回冲刺，避免 50cm 附近来回抖动。
        if self.current_nav2_speed_profile == 2:
            return

        now = self.get_clock().now()
        if self.nav2_dynamic_last_check_time is not None:
            elapsed = (now - self.nav2_dynamic_last_check_time).nanoseconds / 1e9
            if elapsed < self.nav2_profile_2_switch_check_period:
                return
        self.nav2_dynamic_last_check_time = now

        distance = self.get_nav2_distance_to_goal(target_x, target_y, timeout_sec=0.01)
        if distance is None:
            return

        if distance <= self.nav2_profile_2_switch_distance:
            self.task_progress.update('已进入精调距离，正在切换速度参数')
            self.apply_nav2_speed_profile(2)
            self.task_progress.end_stage(
                f'距目标 {distance:.3f}m <= {self.nav2_profile_2_switch_distance:.3f}m，精调参数已生效')
            self.task_progress.stage(f'模式2 精调【{desc}】', '继续当前 Nav2 目标，进行到点精调')
