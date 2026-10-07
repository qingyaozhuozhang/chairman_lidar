#!/usr/bin/env python3
"""
TF Relay Node

功能：
    监听 TF：<source_parent> → <source_child>（例如 map1 → odom1）
    以固定频率（默认 20 Hz）重复发布为：<target_parent> → <target_child>（例如 map → odom）

    如果源 TF 还没就绪，先发布"启动初始位姿"，
    等源 TF 到来后再切换到实时值。
"""

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TfRelayNode(Node):
    def __init__(self):
        super().__init__('tf_relay_node')

        # ============ 参数：源/目标坐标系 ============
        self.declare_parameter('source_parent', 'map1')
        self.declare_parameter('source_child', 'odom1')
        self.declare_parameter('target_parent', 'map')
        self.declare_parameter('target_child', 'odom')

        # ============ 参数：发布频率 / 时间戳偏移 ============
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('time_offset_sec', 0.1)

        # ============ 参数：初始位姿（对应 T_map_3dmap） ============
        # 格式 [x, y, z, roll, pitch, yaw]
        self.declare_parameter('init_pose', [-5.08, -1.5, 0.0, 0.0, 0.0, -1.5708])

        self.source_parent_ = self.get_parameter('source_parent').value
        self.source_child_ = self.get_parameter('source_child').value
        self.target_parent_ = self.get_parameter('target_parent').value
        self.target_child_ = self.get_parameter('target_child').value
        self.publish_rate_ = float(self.get_parameter('publish_rate_hz').value)
        self.time_offset_ = float(self.get_parameter('time_offset_sec').value)
        self.init_pose_ = list(self.get_parameter('init_pose').value)

        # ============ TF 监听 + 缓存 ============
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # ============ TF 广播 ============
        self.tf_broadcaster_ = TransformBroadcaster(self)

        # ============ 缓存最新变换 ============
        self.lock_ = threading.Lock()
        self.latest_transform_ = None
        self.received_source_ = False     # 是否收到过源 TF

        # ============ 初始位姿（作为 fallback） ============
        self.init_transform_ = self._make_init_transform()

        # ============ 定时器 ============
        self.lookup_timer_ = self.create_timer(0.02, self.lookup_source_tf)
        self.publish_timer_ = self.create_timer(
            1.0 / self.publish_rate_, self.publish_tf)

        self.get_logger().info(
            f'TF relay started: '
            f'{self.source_parent_} -> {self.source_child_}  ==>  '
            f'{self.target_parent_} -> {self.target_child_} '
            f'@ {self.publish_rate_} Hz'
        )
        self.get_logger().info(
            f'Fallback init_pose = {self.init_pose_} '
            f'(used until source TF is available)'
        )

    def _make_init_transform(self):
        """把 [x, y, z, roll, pitch, yaw] 转成 TransformStamped"""
        if len(self.init_pose_) < 6:
            self.get_logger().warn(
                f'init_pose length {len(self.init_pose_)} < 6, using identity')
            self.init_pose_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        x, y, z, roll, pitch, yaw = self.init_pose_

        # 欧拉角 → 四元数（ZYX 顺序：Rz * Ry * Rx）
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        t = TransformStamped()
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        return t

    def lookup_source_tf(self):
        """查询源 TF，更新缓存；失败时用初始位姿兜底"""
        try:
            transform = self.tf_buffer_.lookup_transform(
                self.source_parent_,
                self.source_child_,
                rclpy.time.Time())

            with self.lock_:
                self.latest_transform_ = transform
                if not self.received_source_:
                    self.received_source_ = True
                    self.get_logger().info(
                        'Source TF received, switching to live transform.')

        except (LookupException, ConnectivityException, ExtrapolationException):
            # 源 TF 还没就绪：如果还没收到过，用初始位姿兜底
            with self.lock_:
                if not self.received_source_:
                    self.latest_transform_ = self.init_transform_

    def publish_tf(self):
        """按固定频率重复发布缓存的 TF"""
        with self.lock_:
            if self.latest_transform_ is None:
                return
            transform = self.latest_transform_

        out = TransformStamped()
        stamp = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.time_offset_)
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self.target_parent_
        out.child_frame_id = self.target_child_

        out.transform.translation.x = transform.transform.translation.x
        out.transform.translation.y = transform.transform.translation.y
        out.transform.translation.z = transform.transform.translation.z
        out.transform.rotation.x = transform.transform.rotation.x
        out.transform.rotation.y = transform.transform.rotation.y
        out.transform.rotation.z = transform.transform.rotation.z
        out.transform.rotation.w = transform.transform.rotation.w

        self.tf_broadcaster_.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    node = TfRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()