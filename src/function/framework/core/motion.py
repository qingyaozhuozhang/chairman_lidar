"""原功能的共享实现；新增任务通常不需要修改这里。"""
from .imports import *

class MotionMixin:
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def odom_map_callback(self, msg):
        """接收 odometry_transform_math_launch.py 输出的 /odom_map。"""
        self.odom_map_x = float(msg.x)
        self.odom_map_y = float(msg.y)
        self.odom_map_z = float(msg.z)
        self.odom_map_yaw = float(msg.yaw)
        self.odom_map_last_time = time.time()

    def wait_for_fresh_odom_map_z(self, after_time, timeout=3.0):
        """等待 -11 启动之后的新 /odom_map.z，确保起点 z 是功能开始后的值。"""
        wait_start = time.time()

        while rclpy.ok():
            if self.cancel_current_task:
                return None

            if (
                self.odom_map_z is not None and
                self.odom_map_last_time is not None and
                self.odom_map_last_time >= after_time
            ):
                return float(self.odom_map_z)

            if time.time() - wait_start > timeout:
                self.get_logger().error(
                    "⌛ -11 等待 /odom_map.z 数据超时！请确认 odometry_transform_math_launch.py 已启动并发布 /odom_map"
                )
                return None

            time.sleep(self.CONTROL_PERIOD)

        return None

    def wait_for_odom(self, timeout=1.0):
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        start_wait = self.get_clock().now()

        while rclpy.ok():
            if self.cancel_current_task:
                return False

            if (
                self.current_x is not None and
                self.current_y is not None and
                self.current_yaw is not None
            ):
                return True

            if (self.get_clock().now() - start_wait).nanoseconds / 1e9 > timeout:
                self.get_logger().error("⌛ 等待里程计数据超时！")
                return False

            time.sleep(0.005)

        return False

    def flush_odom_queue(self):
        """短暂等待，让订阅回调刷新到较新的里程计 / TF 数据。"""
        time.sleep(0.15)

    def apply_accel_limits(self, current_v, target_v, max_accel, max_decel, dt):
        accel = abs(max_accel)
        decel = abs(max_decel)

        if target_v * current_v < 0 and current_v != 0:
            max_dv = decel * dt
        elif abs(target_v) > abs(current_v):
            max_dv = accel * dt
        else:
            max_dv = decel * dt

        if target_v > current_v:
            return min(target_v, current_v + max_dv)
        else:
            return max(target_v, current_v - max_dv)

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def snap_yaw_to_nearest_90(self, yaw_rad):
        current_deg = math.degrees(yaw_rad)
        snapped_deg = round(current_deg / 90.0) * 90.0

        if snapped_deg > 180:
            snapped_deg -= 360

        if snapped_deg <= -180:
            snapped_deg += 360

        return math.radians(snapped_deg)

    def global_velocity_to_body(self, vx_global, vy_global, current_yaw):
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)

        vx_body = cos_yaw * vx_global + sin_yaw * vy_global
        vy_body = -sin_yaw * vx_global + cos_yaw * vy_global

        return vx_body, vy_body

    def compute_cross_track_speed(self, cross_error):
        if abs(cross_error) <= self.ERROR_TOLERANCE_CROSS:
            return 0.0

        speed = -self.KP_CROSS_TRACK * cross_error
        speed = self.clamp(speed, -self.MAX_CROSS_TRACK_VEL, self.MAX_CROSS_TRACK_VEL)

        if 0.0 < abs(speed) < self.MIN_CROSS_TRACK_VEL:
            speed = math.copysign(self.MIN_CROSS_TRACK_VEL, speed)

        return speed

    def compute_uphill_cross_track_speed(self, cross_error):
        """-2 专用 Y 轴纠偏速度。cross_error = 当前 map_y - 起点 map_y。"""
        if abs(cross_error) <= self.UPHILL_ERROR_TOLERANCE_CROSS:
            return 0.0

        speed = -self.UPHILL_KP_CROSS_TRACK * cross_error
        speed = self.clamp(speed, -self.UPHILL_MAX_CROSS_TRACK_VEL, self.UPHILL_MAX_CROSS_TRACK_VEL)

        if 0.0 < abs(speed) < self.UPHILL_MIN_CROSS_TRACK_VEL:
            speed = math.copysign(self.UPHILL_MIN_CROSS_TRACK_VEL, speed)

        return speed
