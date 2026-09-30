"""闭环旋转 -90 度。run 是框架调用入口，下面是本功能的实际实现。"""
from framework.core.imports import *


def run(ctx, request):
    angle = float(ctx.config.get('angle_degrees', -90.0))
    return ctx.manual(lambda: rotate(ctx.node, angle))


def rotate(node, target_angle_degrees):
    node.get_logger().info('⏳ 正在同步最新里程计数据...')

    if not node.wait_for_odom():
        node.get_logger().warn('⚠️ 无法获取里程计，旋转任务取消')
        return False

    target_yaw = node.current_yaw + math.radians(target_angle_degrees)
    target_yaw = node.normalize_angle(target_yaw)

    twist_msg = Twist()
    node.get_logger().info(f'🚀 开始闭环精准旋转: {target_angle_degrees}度')

    start_time = node.get_clock().now()
    last_time = start_time
    current_omega = 0.0

    while rclpy.ok():
        if node.cancel_current_task:
            node.get_logger().warn("🚫 旋转任务被行为树中止")
            node.publish_manual_zero_speed()
            return False

        now = node.get_clock().now()
        dt = (now - last_time).nanoseconds / 1e9

        if dt <= 0:
            dt = 0.01

        last_time = now

        if (now - start_time).nanoseconds / 1e9 > 20.0:
            node.get_logger().warn('⚠️ 强制旋转超时！')
            node.publish_manual_zero_speed()
            return False

        error = node.normalize_angle(target_yaw - node.current_yaw)

        if abs(error) <= node.ERROR_TOLERANCE_YAW:
            break

        safe_omega = math.sqrt(2.0 * (0.8 * node.MAX_DECEL[2]) * abs(error))
        p_omega = node.KP_ANGULAR * abs(error)
        target_omega_abs = min(node.MAX_VEL_ANGULAR, safe_omega, p_omega)

        if error > 0:
            target_omega = max(node.MIN_VEL_ANGULAR, target_omega_abs)
        elif error < 0:
            target_omega = min(-node.MIN_VEL_ANGULAR, -target_omega_abs)
        else:
            target_omega = 0.0

        current_omega = node.apply_accel_limits(
            current_omega,
            target_omega,
            node.MAX_ACCEL[2],
            node.MAX_DECEL[2],
            dt
        )

        twist_msg.angular.z = current_omega
        node.publish_twist_speed(twist_msg)

        time.sleep(node.CONTROL_PERIOD)

    node.publish_manual_zero_speed()
    return True
