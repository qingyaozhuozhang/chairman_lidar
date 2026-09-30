"""三个登阶任务共用的周期纠偏与停止控制。方向和完成条件在各自 task.py。"""
from framework.core.imports import *


class StairMixin:
    def stair_callback(self, msg):
        self.stair_cmd = msg.data

    def stair_timer_callback(self):
        if not self.in_stair_mode:
            return

        if (
            self.current_x is None or
            self.current_y is None or
            self.current_yaw is None or
            self.stair_start_x is None or
            self.stair_start_y is None or
            self.stair_start_yaw is None
        ):
            self.get_logger().warn('⚠️ 运动纠偏所需里程计丢失，立即刹车退出')
            self.stop_stair_mode()
            return

        now = self.get_clock().now()
        dt = (now - self.stair_last_time).nanoseconds / 1e9

        if dt <= 0:
            dt = self.CONTROL_PERIOD

        dt = max(0.001, min(dt, 0.1))
        self.stair_last_time = now

        dx = self.current_x - self.stair_start_x
        dy = self.current_y - self.stair_start_y

        # 垂直路线方向误差：
        cross_error = dx * self.stair_cross_dir_x + dy * self.stair_cross_dir_y
        cross_speed = self.compute_cross_track_speed(cross_error)

        # 全局速度 = 主方向速度 + 垂直方向纠偏速度。
        target_vx_global = (
            self.stair_global_dir_x * self.stair_target_speed
            + self.stair_cross_dir_x * cross_speed
        )
        target_vy_global = (
            self.stair_global_dir_y * self.stair_target_speed
            + self.stair_cross_dir_y * cross_speed
        )

        # 转换到当前车体坐标系。
        target_vx, target_vy = self.global_velocity_to_body(
            target_vx_global,
            target_vy_global,
            self.current_yaw
        )

        yaw_error = self.normalize_angle(self.stair_start_yaw - self.current_yaw)
        target_omega = self.KP_YAW_CORRECT * yaw_error
        target_omega = self.clamp(
            target_omega,
            -self.MAX_VEL_ANGULAR * self.YAW_CORRECT_MAX_RATIO,
            self.MAX_VEL_ANGULAR * self.YAW_CORRECT_MAX_RATIO
        )

        self.stair_current_vx = self.apply_accel_limits(
            self.stair_current_vx,
            target_vx,
            self.MAX_ACCEL[0],
            self.MAX_DECEL[0],
            dt
        )

        self.stair_current_vy = self.apply_accel_limits(
            self.stair_current_vy,
            target_vy,
            self.MAX_ACCEL[1],
            self.MAX_DECEL[1],
            dt
        )

        self.stair_current_omega = self.apply_accel_limits(
            self.stair_current_omega,
            target_omega,
            self.MAX_ACCEL[2],
            self.MAX_DECEL[2],
            dt
        )

        twist_msg = Twist()
        twist_msg.linear.x = self.stair_current_vx
        twist_msg.linear.y = self.stair_current_vy
        twist_msg.angular.z = self.stair_current_omega

        self.publish_twist_speed(twist_msg)

    def stop_stair_mode(self):
        self.in_stair_mode = False

        self.stair_target_vx = 0.0
        self.stair_target_vy = 0.0
        self.stair_target_speed = 0.0

        self.stair_current_vx = 0.0
        self.stair_current_vy = 0.0
        self.stair_current_omega = 0.0

        self.stair_start_x = None
        self.stair_start_y = None
        self.stair_start_yaw = None

        self.stair_global_dir_x = 0.0
        self.stair_global_dir_y = 0.0
        self.stair_cross_dir_x = 0.0
        self.stair_cross_dir_y = 0.0

        self.publish_manual_zero_speed()

