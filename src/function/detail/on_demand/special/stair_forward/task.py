"""登阶控制：保持起始行进方向并纠偏，收到 /nav_topic=0 后结束。"""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from chairman_tasks.special import motion


def run(ctx, request):
    speed = float(ctx.config.get('speed', 0.3))
    return ctx.manual(lambda: start_corrected_stair_mode(ctx.node, 1, 0, speed, '向前')
                      and wait_stair_stop_signal(ctx.node, '向前'))


def start_corrected_stair_mode(node, local_x, local_y, target_speed, desc):
    """用起始航向将车体方向转换为里程计坐标系中的固定方向。

    定时器修正横向偏差与偏航误差，再转换为当前车体速度并直接发送到底盘。"""
    node.stair_cmd = -1

    node.task_progress.update(
        f'⏳ 准备持续移动：{desc}，正在读取里程计并初始化控制参数'
    )

    if not node.wait_for_odom(timeout=1.0):
        node.task_progress.fail(f'⚠️ {desc} 取消：无法获取里程计')
        return False

    dir_norm = math.hypot(local_x, local_y)
    if dir_norm <= 1e-9:
        node.task_progress.fail(f'❌ {desc} 取消：运动方向不能为 0')
        return False

    local_dir_x = float(local_x) / dir_norm
    local_dir_y = float(local_y) / dir_norm

    node.stair_start_x = node.current_x
    node.stair_start_y = node.current_y
    node.stair_start_yaw = node.current_yaw

    # 把车体局部方向固定到启动瞬间的 odom 全局方向。
    node.stair_global_dir_x = (
        local_dir_x * math.cos(node.stair_start_yaw)
        - local_dir_y * math.sin(node.stair_start_yaw)
    )
    node.stair_global_dir_y = (
        local_dir_x * math.sin(node.stair_start_yaw)
        + local_dir_y * math.cos(node.stair_start_yaw)
    )

    # 行进方向的左法向量，用于计算有符号横向偏差。
    node.stair_cross_dir_x = -node.stair_global_dir_y
    node.stair_cross_dir_y = node.stair_global_dir_x

    node.stair_target_speed = abs(float(target_speed))

    # 保存初始车体目标速度；实际输出由定时器结合当前姿态计算。
    node.stair_target_vx = local_dir_x * node.stair_target_speed
    node.stair_target_vy = local_dir_y * node.stair_target_speed

    node.stair_current_vx = 0.0
    node.stair_current_vy = 0.0
    node.stair_current_omega = 0.0

    node.stair_last_time = node.get_clock().now()
    node.in_stair_mode = True

    node.task_progress.update(
        f'🚀 开始持续移动：{desc}，'
        f'speed={node.stair_target_speed:.3f} m/s，等待 /nav_topic 收到 0 停止'
    )
    return True


def wait_stair_stop_signal(node, task_name):
    while rclpy.ok():
        if node.cancel_current_task:
            node.task_progress.fail(f"🚫 {task_name}被外部中止")
            stop_stair_mode(node)
            return False

        if node.stair_cmd == 0:
            stop_stair_mode(node)
            node.task_progress.update(f'✅ {task_name}持续移动结束：收到 /nav_topic=0，已刹车')
            return True

        time.sleep(0.02)

    stop_stair_mode(node)
    return False


def stair_timer_callback(node):
    if not node.in_stair_mode:
        return

    if (
        node.current_x is None or
        node.current_y is None or
        node.current_yaw is None or
        node.stair_start_x is None or
        node.stair_start_y is None or
        node.stair_start_yaw is None
    ):
        node.task_progress.fail('⚠️ 运动纠偏所需里程计丢失，立即刹车退出')
        stop_stair_mode(node)
        return

    now = node.get_clock().now()
    dt = (now - node.stair_last_time).nanoseconds / 1e9

    if dt <= 0:
        dt = node.CONTROL_PERIOD

    dt = max(0.001, min(dt, 0.1))
    node.stair_last_time = now

    dx = node.current_x - node.stair_start_x
    dy = node.current_y - node.stair_start_y

    # 位移在路径法向量上的投影即横向偏差。
    cross_error = dx * node.stair_cross_dir_x + dy * node.stair_cross_dir_y
    cross_speed = motion.compute_cross_track_speed(node, cross_error)

    # 全局速度 = 主方向速度 + 垂直方向纠偏速度。
    target_vx_global = (
        node.stair_global_dir_x * node.stair_target_speed
        + node.stair_cross_dir_x * cross_speed
    )
    target_vy_global = (
        node.stair_global_dir_y * node.stair_target_speed
        + node.stair_cross_dir_y * cross_speed
    )

    # 转换到当前车体坐标系。
    target_vx, target_vy = motion.global_velocity_to_body(
        target_vx_global,
        target_vy_global,
        node.current_yaw
    )

    yaw_error = motion.normalize_angle(node.stair_start_yaw - node.current_yaw)
    target_omega = node.KP_YAW_CORRECT * yaw_error
    target_omega = motion.clamp(
        target_omega,
        -node.MAX_VEL_ANGULAR * node.YAW_CORRECT_MAX_RATIO,
        node.MAX_VEL_ANGULAR * node.YAW_CORRECT_MAX_RATIO
    )

    node.stair_current_vx = motion.apply_accel_limits(
        node.stair_current_vx,
        target_vx,
        node.MAX_ACCEL[0],
        node.MAX_DECEL[0],
        dt
    )

    node.stair_current_vy = motion.apply_accel_limits(
        node.stair_current_vy,
        target_vy,
        node.MAX_ACCEL[1],
        node.MAX_DECEL[1],
        dt
    )

    node.stair_current_omega = motion.apply_accel_limits(
        node.stair_current_omega,
        target_omega,
        node.MAX_ACCEL[2],
        node.MAX_DECEL[2],
        dt
    )

    twist_msg = Twist()
    twist_msg.linear.x = node.stair_current_vx
    twist_msg.linear.y = node.stair_current_vy
    twist_msg.angular.z = node.stair_current_omega

    node.publish_twist_speed(twist_msg)


def stop_stair_mode(node):
    node.in_stair_mode = False

    node.stair_target_vx = 0.0
    node.stair_target_vy = 0.0
    node.stair_target_speed = 0.0

    node.stair_current_vx = 0.0
    node.stair_current_vy = 0.0
    node.stair_current_omega = 0.0

    node.stair_start_x = None
    node.stair_start_y = None
    node.stair_start_yaw = None

    node.stair_global_dir_x = 0.0
    node.stair_global_dir_y = 0.0
    node.stair_cross_dir_x = 0.0
    node.stair_cross_dir_y = 0.0

    node.publish_manual_zero_speed()
