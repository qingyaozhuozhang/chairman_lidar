"""原功能的共享实现；新增任务通常不需要修改这里。"""
from .imports import *

class TrackingMixin:
    def stop_callback(self, msg):
        self.get_logger().warn('[中止请求] 已收到停止指令，正在停车；等待动作结束并恢复参数')

        self.cancel_current_task = True
        self.in_stair_mode = False

        self.stair_target_vx = 0.0
        self.stair_target_vy = 0.0
        self.stair_target_speed = 0.0
        self.stair_current_vx = 0.0
        self.stair_current_vy = 0.0
        self.stair_current_omega = 0.0

        self.manual_control_active = False
        self.in_stair_mode = False
        self.stop_nav_cmd_tracking("🛑 紧急中断，显式停止 Nav2 速度转发")
        self.publish_manual_zero_speed("🛑 紧急中断，手写控制速度清零")

    def predict(self):
        self.kf_x = self.F @ self.kf_x
        self.kf_P = self.F @ self.kf_P @ self.F.T + self.Q

    def update(self, z):
        S = self.H @ self.kf_P @ self.H.T + self.R
        K = self.kf_P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.kf_x
        self.kf_x = self.kf_x + K @ y
        I = np.eye(6)
        self.kf_P = (I - K @ self.H) @ self.kf_P

    def reset_filter_state(self):
        """清空滤波器内部速度状态，防止到点/暂停后残余速度继续输出。"""
        self.kf_x = np.zeros((6, 1))
        self.kf_P = np.eye(6) * 1.0
        self.latest_z = None
        self.new_meas_available = False
        self.is_active = False

    def publish_zero_speed(self, reason: str = ""):
        """立即向底盘速度话题发布 0 速度。

        注意：高频定时器里调用本函数时不要传 reason，避免日志刷屏。
        """
        stop_msg = SpeedHeading()
        stop_msg.linear_x = 0.0
        stop_msg.linear_y = 0.0
        stop_msg.angular_z = 0.0
        self.data_pub.publish(stop_msg)

        if reason:
            self.get_logger().debug(reason)

    def publish_twist_speed(self, twist_msg: Twist, reason: str = ""):
        """把 Twist 速度直接转换成底盘 SpeedHeading 输出，不再回写 /cmd_vel。"""
        speed_msg = SpeedHeading()
        speed_msg.linear_x = float(twist_msg.linear.x)
        speed_msg.linear_y = float(twist_msg.linear.y)
        speed_msg.angular_z = float(twist_msg.angular.z)
        self.data_pub.publish(speed_msg)

        if reason:
            self.get_logger().debug(reason)

    def publish_manual_zero_speed(self, reason: str = ""):
        """手写闭环/登阶结束时直接清底盘速度。"""
        self.reset_filter_state()
        self.publish_zero_speed(reason)

    def start_nav_cmd_tracking(self, desc: str = ""):
        """
        显式开始 Nav2 /cmd_vel 转发。
        只在 execute_nav2_goal() 里调用，避免旧 goal/status/零速帧误触发。
        """
        self.reset_filter_state()
        self.nav_cmd_tracking_enabled = True
        if desc:
            self.get_logger().debug(f"✅ Nav2速度转发已开启：{desc}")
        else:
            self.get_logger().debug("✅ Nav2速度转发已开启")

    def stop_nav_cmd_tracking(self, reason: str = ""):
        """
        显式结束 Nav2 /cmd_vel 转发，并立即给底盘发 0。
        这是 Nav2 速度链路唯一的停车入口，不再依赖：
          - /navigate_to_pose/_action/status
          - /cmd_vel == 0
          - /cmd_vel 超时
        """
        was_enabled = self.nav_cmd_tracking_enabled
        self.nav_cmd_tracking_enabled = False
        self.reset_filter_state()
        self.publish_zero_speed(reason if reason else "🛑 Nav2速度转发已显式结束，底盘速度清零")
        if was_enabled:
            self.get_logger().debug("🛑 Nav2速度转发已关闭，后续旧 /cmd_vel 将被忽略")

    def cmd_callback(self, msg):
        """
        Nav2 /cmd_vel 输入回调。
        这里只在显式开启 nav_cmd_tracking_enabled 后转发速度；
        不再把 /cmd_vel=0 当成停车条件。
        """
        if self.manual_control_active or self.in_stair_mode:
            return

        if not self.nav_cmd_tracking_enabled:
            return

        linear_x = float(msg.linear.x)
        linear_y = float(msg.linear.y)
        angular_z = float(msg.angular.z)

        self.latest_z = np.array([
            [linear_x],
            [linear_y],
            [angular_z]
        ])

        self.new_meas_available = True

        if not self.is_active:
            self.is_active = True

    def tracker_timer_callback(self):
        """
        Nav2 速度转发定时器。
        只在 nav_cmd_tracking_enabled=True 时输出；
        停车只由 stop_nav_cmd_tracking() 显式触发。
        """
        if not self.nav_cmd_tracking_enabled:
            return

        if not self.is_active and self.latest_z is None:
            return

        self.predict()

        if self.new_meas_available and self.latest_z is not None:
            self.update(self.latest_z)
            self.new_meas_available = False

        pub_msg = SpeedHeading()
        pub_msg.linear_x = float(self.kf_x[0, 0])
        pub_msg.linear_y = float(self.kf_x[1, 0])
        pub_msg.angular_z = float(self.kf_x[2, 0])

        self.data_pub.publish(pub_msg)
