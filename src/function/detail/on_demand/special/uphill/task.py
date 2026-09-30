"""世界 X 方向上坡控制。run 是框架调用入口，下面是本功能的实际实现。"""
from framework.core.imports import *


def run(ctx, request):
    distance = ctx.config.get('distance')
    return ctx.manual(lambda: climb(ctx.node, distance))


def climb(node, target_distance=None):
    if target_distance is None:
        target_distance = node.UPHILL_GLOBAL_X_DISTANCE

    node.get_logger().info('⏳ -2 正在获取 map 全局位姿，准备沿全局 X 轴正方向直走...')
    node.flush_odom_queue()

    pose = None
    wait_start = time.time()

    while rclpy.ok():
        if node.cancel_current_task:
            node.publish_manual_zero_speed()
            return False

        pose = node.get_current_map_pose(timeout_sec=0.2, log_error=False)

        if pose is not None:
            break

        if time.time() - wait_start > 3.0:
            node.get_logger().error('❌ -2 启动失败：3 秒内无法获取 map -> base_footprint 位姿')
            node.publish_manual_zero_speed()
            return False

        time.sleep(0.05)

    start_x, start_y, start_yaw = pose
    target_x = start_x + float(target_distance)
    target_y = start_y
    target_yaw = start_yaw

    node.get_logger().info(
        f'vx:+0.000 m/s，'
        f'x:+0.0000m，'
        f'y偏移:+0.0000 m，'
        f'yaw偏移:+0.00°'
    )

    twist_msg = Twist()
    start_time = time.time()
    last_time = start_time
    last_debug_time = 0.0

    current_vx = 0.0
    current_vy = 0.0
    current_omega = 0.0
    task_success = False

    while rclpy.ok():
        if node.cancel_current_task:
            node.get_logger().warn('⚠️ -2 收到取消标志，正在停车退出')
            break

        now = time.time()
        dt = now - last_time

        if dt <= 0.0:
            dt = node.CONTROL_PERIOD

        dt = max(0.001, min(dt, 0.1))
        last_time = now

        pose = node.get_current_map_pose(timeout_sec=0.02, log_error=False)

        if pose is None:
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 0.0
            node.publish_twist_speed(twist_msg)
            time.sleep(node.CONTROL_PERIOD)
            continue

        current_x, current_y, current_yaw = pose

        forward_error = target_x - current_x
        x_traveled = current_x - start_x
        cross_error = current_y - target_y
        yaw_error = node.normalize_angle(target_yaw - current_yaw)

        if (
            abs(forward_error) <= node.UPHILL_ERROR_TOLERANCE_DIST and
            abs(cross_error) <= node.UPHILL_ERROR_TOLERANCE_CROSS and
            abs(yaw_error) <= node.UPHILL_ERROR_TOLERANCE_YAW
        ):
            node.get_logger().info(
                f'vx:+0.000 m/s，'
                f'x:{x_traveled:+.4f}m，'
                f'y偏移:{cross_error:+.4f} m，'
                f'yaw偏移:{math.degrees(yaw_error):+.2f}°'
            )
            task_success = True
            break

        if now - start_time > node.UPHILL_GLOBAL_X_TIMEOUT:
            node.get_logger().warn(
                f'vx:+0.000 m/s，'
                f'x:{x_traveled:+.4f}m，'
                f'y偏移:{cross_error:+.4f} m，'
                f'yaw偏移:{math.degrees(yaw_error):+.2f}°'
            )
            break

        if abs(forward_error) <= node.UPHILL_ERROR_TOLERANCE_DIST:
            vx_global = 0.0
        else:
            safe_speed = math.sqrt(
                2.0 * (0.8 * abs(node.UPHILL_MAX_DECEL[0])) * abs(forward_error)
            )
            p_speed = node.UPHILL_KP_LINEAR * abs(forward_error)
            speed_abs = min(node.UPHILL_MAX_VEL_LINEAR, safe_speed, p_speed)

            if forward_error > 0.0:
                vx_global = max(node.UPHILL_MIN_VEL_LINEAR, speed_abs)
            else:
                vx_global = min(-node.UPHILL_MIN_VEL_LINEAR, -speed_abs)

        vy_global = node.compute_uphill_cross_track_speed(cross_error)

        target_vx, target_vy = node.global_velocity_to_body(
            vx_global,
            vy_global,
            current_yaw
        )

        if abs(yaw_error) <= node.UPHILL_ERROR_TOLERANCE_YAW:
            target_omega = 0.0
        else:
            max_correct_omega = (
                node.UPHILL_MAX_VEL_ANGULAR * node.UPHILL_YAW_CORRECT_MAX_RATIO
            )
            target_omega = node.UPHILL_KP_YAW_CORRECT * yaw_error
            target_omega = node.clamp(target_omega, -max_correct_omega, max_correct_omega)

            if 0.0 < abs(target_omega) < node.UPHILL_MIN_VEL_ANGULAR:
                target_omega = math.copysign(node.UPHILL_MIN_VEL_ANGULAR, target_omega)

        current_vx = node.apply_accel_limits(
            current_vx,
            target_vx,
            node.UPHILL_MAX_ACCEL[0],
            node.UPHILL_MAX_DECEL[0],
            dt
        )

        current_vy = node.apply_accel_limits(
            current_vy,
            target_vy,
            node.UPHILL_MAX_ACCEL[1],
            node.UPHILL_MAX_DECEL[1],
            dt
        )

        current_omega = node.apply_accel_limits(
            current_omega,
            target_omega,
            node.UPHILL_MAX_ACCEL[2],
            node.UPHILL_MAX_DECEL[2],
            dt
        )

        twist_msg.linear.x = current_vx
        twist_msg.linear.y = current_vy
        twist_msg.angular.z = current_omega
        node.publish_twist_speed(twist_msg)

        if now - last_debug_time > 0.3:
            node.get_logger().info(
                f'vx:{vx_global:+.3f} m/s，'
                f'x:{x_traveled:+.4f}m，'
                f'y偏移:{cross_error:+.4f} m，'
                f'yaw偏移:{math.degrees(yaw_error):+.2f}°'
            )
            last_debug_time = now

        time.sleep(node.CONTROL_PERIOD)

    twist_msg.linear.x = 0.0
    twist_msg.linear.y = 0.0
    twist_msg.angular.z = 0.0
    node.publish_twist_speed(twist_msg)
    time.sleep(node.CONTROL_PERIOD)
    node.get_logger().info('✅ -2 已停车，返回主服务流程\n')

    return task_success
