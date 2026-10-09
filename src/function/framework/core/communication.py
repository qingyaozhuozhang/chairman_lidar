"""公共 ROS 连接：话题、服务、位姿查询和底盘速度输出。"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from custom_msg.srv import SetNavTarget
from custom_msg.msg import SpeedHeading, PoseEuler

from .actions import TrackedActionClient
from .parameters import RosParameterBackend, ParameterTransaction

class CommunicationNode(Node):
    def __init__(self, task_callback, stop_callback, restore_callback):
        super().__init__('simple_nav_node')
        self.reentrant_group = ReentrantCallbackGroup()
        self.cancel_current_task = False
        self.manual_control_active = False
        self.in_stair_mode = False
        self.stair_cmd = -1
        self.current_x = self.current_y = self.current_yaw = None
        self.odom_map_x = self.odom_map_y = self.odom_map_z = self.odom_map_yaw = None
        self.odom_map_last_time = None
        self.nav_cmd_tracking_enabled = False
        self.current_nav_goal_handle = None
        self._nav_client = TrackedActionClient(self, ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.reentrant_group))
        self.parameter_transaction = ParameterTransaction(RosParameterBackend(self))
        self.data_pub = self.create_publisher(SpeedHeading, '/nav_speed_heading_data', 10)
        for message, topic, callback in (
            (Odometry, '/Odometry', self.odom_callback),
            (PoseEuler, '/odom_map', self.odom_map_callback),
            (Twist, '/cmd_vel', self.cmd_callback),
            (Int8, '/nav_topic', self.stair_callback),
            (Empty, '/emergency_stop', stop_callback),
        ):
            self.create_subscription(message, topic, callback, 10, callback_group=self.reentrant_group)
        self.create_service(SetNavTarget, '/set_nav_target', task_callback, callback_group=self.reentrant_group)
        self.create_service(Trigger, '/restore_navigation_parameters', restore_callback,
                            callback_group=self.reentrant_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dt = 1.0 / 50.0
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = self.dt
        self.F[3, 3] = self.F[4, 4] = self.F[5, 5] = 0.8
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.Q = np.eye(6) * 0.50
        self.R = np.eye(3) * 0.1
        self.reset_filter_state()
        self.tracker_timer = self.create_timer(self.dt, self.tracker_timer_callback,
                                              callback_group=self.reentrant_group)


    def stair_callback(self, msg):
        self.stair_cmd = msg.data

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def odom_map_callback(self, msg):
        """缓存 odometry 节点发布的地图位姿及本机接收时间。"""
        self.odom_map_x = float(msg.x)
        self.odom_map_y = float(msg.y)
        self.odom_map_z = float(msg.z)
        self.odom_map_yaw = float(msg.yaw)
        self.odom_map_last_time = time.time()

    def wait_for_fresh_odom_map_z(self, after_time, timeout=3.0):
        """等待 after_time 之后收到的高度值；超时或取消时返回 None。"""
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
        """等待 0.15 秒供回调更新位姿；不清空消息队列，也不保证时间同步。"""
        time.sleep(0.15)

    def get_current_map_pose(self, timeout_sec=1.0, log_error=True):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                Time(),
                timeout=Duration(seconds=timeout_sec)
            )

            map_x = trans.transform.translation.x
            map_y = trans.transform.translation.y

            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            map_yaw = math.atan2(siny_cosp, cosy_cosp)

            return map_x, map_y, map_yaw

        except Exception as e:
            if log_error:
                self.get_logger().error(f'❌ 无法获取 TF 地图坐标转换 map -> base_footprint: {e}')
            return None

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
        """清空速度滤波状态及待处理观测，避免下一段使用上一段速度。"""
        self.kf_x = np.zeros((6, 1))
        self.kf_P = np.eye(6) * 1.0
        self.latest_z = None
        self.new_meas_available = False
        self.is_active = False

    def publish_zero_speed(self, reason: str = ""):
        """发布底盘零速度；reason 非空时记录 DEBUG 日志。"""
        stop_msg = SpeedHeading()
        stop_msg.linear_x = 0.0
        stop_msg.linear_y = 0.0
        stop_msg.angular_z = 0.0
        self.data_pub.publish(stop_msg)

        if reason:
            self.get_logger().debug(reason)

    def publish_twist_speed(self, twist_msg: Twist, reason: str = ""):
        """将车体 Twist 转为 SpeedHeading，直接发布至底盘速度话题。"""
        speed_msg = SpeedHeading()
        speed_msg.linear_x = float(twist_msg.linear.x)
        speed_msg.linear_y = float(twist_msg.linear.y)
        speed_msg.angular_z = float(twist_msg.angular.z)
        self.data_pub.publish(speed_msg)

        if reason:
            self.get_logger().debug(reason)

    def publish_manual_zero_speed(self, reason: str = ""):
        """清空滤波状态并发布零速度，用于手写运动收尾。"""
        self.reset_filter_state()
        self.publish_zero_speed(reason)

    def start_nav_cmd_tracking(self, desc: str = ""):
        """清空滤波状态并开启 Nav2 速度转发，由导航任务控制启停。"""
        self.reset_filter_state()
        self.nav_cmd_tracking_enabled = True
        if desc:
            self.get_logger().debug(f"✅ Nav2速度转发已开启：{desc}")
        else:
            self.get_logger().debug("✅ Nav2速度转发已开启")

    def stop_nav_cmd_tracking(self, reason: str = ""):
        """关闭 Nav2 速度转发，清空滤波状态并立即发布零速度。"""
        was_enabled = self.nav_cmd_tracking_enabled
        self.nav_cmd_tracking_enabled = False
        self.reset_filter_state()
        self.publish_zero_speed(reason if reason else "🛑 Nav2速度转发已显式结束，底盘速度清零")
        if was_enabled:
            self.get_logger().debug("🛑 Nav2速度转发已关闭，后续旧 /cmd_vel 将被忽略")

    def cmd_callback(self, msg):
        """接收 Nav2 速度观测；仅在导航转发启用且手写控制未接管时处理。"""
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
        """按固定周期更新滤波器，并在导航转发启用时输出底盘速度。"""
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
