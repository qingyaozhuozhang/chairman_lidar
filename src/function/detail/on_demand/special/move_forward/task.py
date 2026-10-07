"""带航向及横向纠偏的定距移动。run 是框架调用入口，下面是本功能的实际实现。"""
from framework.core.imports import *


def run(ctx, request):
    distance = float(ctx.config.get('distance', 1.2))
    return ctx.manual(lambda: move(ctx.node, 1.0, 0.0, distance))


def move(node, linear_x, linear_y, target_distance):
    node.get_logger().info('⏳ 正在同步最新里程计数据...')

    if not node.wait_for_odom():
        node.get_logger().warn('⚠️ 无法获取里程计，移动任务取消')
        return False

    start_x = node.current_x
    start_y = node.current_y
    start_yaw = node.current_yaw

    twist_msg = Twist()

    dir_norm = math.hypot(linear_x, linear_y)
    if dir_norm == 0:
        return False

    local_dir_x = linear_x / dir_norm
    local_dir_y = linear_y / dir_norm

    global_dir_x = local_dir_x * math.cos(start_yaw) - local_dir_y * math.sin(start_yaw)
    global_dir_y = local_dir_x * math.sin(start_yaw) + local_dir_y * math.cos(start_yaw)

    cross_dir_x = -global_dir_y
    cross_dir_y = global_dir_x

    node.get_logger().info(
        f'🚀 开始闭环精准移动: 目标距离={target_distance}m，'
        f'启用 yaw 锁定 + 横向/旁向偏移修正'
    )

    start_time = node.get_clock().now()
    last_time = start_time
    last_debug_time = start_time

    current_vx = 0.0
    current_vy = 0.0
    current_omega = 0.0

    while rclpy.ok():
        if node.cancel_current_task:
            node.get_logger().warn("🚫 移动任务被行为树中止")
            node.publish_manual_zero_speed()
            return False

        if node.current_x is None or node.current_y is None or node.current_yaw is None:
            node.get_logger().warn('⚠️ 移动过程中里程计丢失，任务取消')
            node.publish_manual_zero_speed()
            return False

        now = node.get_clock().now()
        dt = (now - last_time).nanoseconds / 1e9

        if dt <= 0:
            dt = node.CONTROL_PERIOD

        dt = max(0.001, min(dt, 0.1))
        last_time = now

        dx = node.current_x - start_x
        dy = node.current_y - start_y

        traveled_distance_in_dir = dx * global_dir_x + dy * global_dir_y
        forward_error = target_distance - traveled_distance_in_dir

        cross_error = dx * cross_dir_x + dy * cross_dir_y
        yaw_error = node.normalize_angle(start_yaw - node.current_yaw)

        if (
            abs(forward_error) <= node.ERROR_TOLERANCE_DIST and
            abs(cross_error) <= node.ERROR_TOLERANCE_CROSS and
            abs(yaw_error) <= node.ERROR_TOLERANCE_YAW
        ):
            node.get_logger().info(
                f'✅ 闭环移动完成: forward_error={forward_error:+.4f} m, '
                f'cross_error={cross_error:+.4f} m, '
                f'yaw_error={math.degrees(yaw_error):+.2f}°'
            )
            break

        if (now - start_time).nanoseconds / 1e9 > 30.0:
            node.get_logger().warn(
                f'⚠️ 强制移动超时！forward_error={forward_error:+.4f} m, '
                f'cross_error={cross_error:+.4f} m, '
                f'yaw_error={math.degrees(yaw_error):+.2f}°'
            )
            node.publish_manual_zero_speed()
            return False

        if abs(forward_error) <= node.ERROR_TOLERANCE_DIST:
            forward_speed = 0.0
        else:
            safe_speed = math.sqrt(2.0 * (0.8 * node.MAX_DECEL[0]) * abs(forward_error))
            p_speed = node.KP_LINEAR * abs(forward_error)
            target_speed_abs = min(node.MAX_VEL_LINEAR, safe_speed, p_speed)

            if forward_error > 0:
                forward_speed = max(node.MIN_VEL_LINEAR, target_speed_abs)
            elif forward_error < 0:
                forward_speed = min(-node.MIN_VEL_LINEAR, -target_speed_abs)
            else:
                forward_speed = 0.0

        cross_speed = node.compute_cross_track_speed(cross_error)

        target_vx_global = global_dir_x * forward_speed + cross_dir_x * cross_speed
        target_vy_global = global_dir_y * forward_speed + cross_dir_y * cross_speed

        target_vx, target_vy = node.global_velocity_to_body(
            target_vx_global,
            target_vy_global,
            node.current_yaw
        )

        target_omega = node.KP_YAW_CORRECT * yaw_error
        target_omega = node.clamp(
            target_omega,
            -node.MAX_VEL_ANGULAR * node.YAW_CORRECT_MAX_RATIO,
            node.MAX_VEL_ANGULAR * node.YAW_CORRECT_MAX_RATIO
        )

        current_vx = node.apply_accel_limits(
            current_vx,
            target_vx,
            node.MAX_ACCEL[0],
            node.MAX_DECEL[0],
            dt
        )

        current_vy = node.apply_accel_limits(
            current_vy,
            target_vy,
            node.MAX_ACCEL[1],
            node.MAX_DECEL[1],
            dt
        )

        current_omega = node.apply_accel_limits(
            current_omega,
            target_omega,
            node.MAX_ACCEL[2],
            node.MAX_DECEL[2],
            dt
        )

        twist_msg.linear.x = current_vx
        twist_msg.linear.y = current_vy
        twist_msg.angular.z = current_omega

        node.publish_twist_speed(twist_msg)

        if (now - last_debug_time).nanoseconds / 1e9 > 0.3:
            node.task_progress.update(
                f'📏 移动闭环: forward_error={forward_error:+.4f} m, '
                f'cross_error={cross_error:+.4f} m, '
                f'cross_v={cross_speed:+.3f} m/s, '
                f'yaw_error={math.degrees(yaw_error):+.2f}°'
            )
            last_debug_time = now

        time.sleep(node.CONTROL_PERIOD)

    node.publish_manual_zero_speed()
    return True
