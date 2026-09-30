"""登阶移动的方向、速度、启动条件与结束条件。run 是框架调用入口，下面是本功能的实际实现。"""
from framework.core.imports import *


def run(ctx, request):
    speed = float(ctx.config.get('speed', 0.3))
    return ctx.manual(lambda: start_corrected_stair_mode(ctx.node, 1, 0, speed, '向前')
                      and wait_stair_stop_signal(ctx.node, '向前'))


def start_corrected_stair_mode(node, local_x, local_y, target_speed, desc):
    """
    持续运动模式。
      - 主运动方向固定在启动瞬间的 odom 全局方向；
      - cross_error 修正垂直方向漂移；
      - yaw_error 修正偏航角漂移；
      - 输出前先把全局速度转换回当前车体坐标系 cmd_vel。
    """
    node.stair_cmd = -1

    node.get_logger().info(
        f'⏳ 准备持续移动：{desc}，正在读取里程计并初始化控制参数'
    )

    if not node.wait_for_odom(timeout=1.0):
        node.get_logger().warn(f'⚠️ {desc} 取消：无法获取里程计')
        return False

    dir_norm = math.hypot(local_x, local_y)
    if dir_norm <= 1e-9:
        node.get_logger().error(f'❌ {desc} 取消：运动方向不能为 0')
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

    # 目标路线垂直方向修正
    node.stair_cross_dir_x = -node.stair_global_dir_y
    node.stair_cross_dir_y = node.stair_global_dir_x

    node.stair_target_speed = abs(float(target_speed))

    # 这两个值只作为语义记录，真实目标速度在 timer 里实时计算。
    node.stair_target_vx = local_dir_x * node.stair_target_speed
    node.stair_target_vy = local_dir_y * node.stair_target_speed

    node.stair_current_vx = 0.0
    node.stair_current_vy = 0.0
    node.stair_current_omega = 0.0

    node.stair_last_time = node.get_clock().now()
    node.in_stair_mode = True

    node.get_logger().info(
        f'🚀 开始持续移动：{desc}，'
        f'speed={node.stair_target_speed:.3f} m/s，等待 /nav_topic 收到 0 停止'
    )
    return True


def wait_stair_stop_signal(node, task_name):
    while rclpy.ok():
        if node.cancel_current_task:
            node.get_logger().warn(f"🚫 {task_name}被外部中止")
            node.stop_stair_mode()
            return False

        if node.stair_cmd == 0:
            node.stop_stair_mode()
            node.get_logger().info(f'✅ {task_name}持续移动结束：收到 /nav_topic=0，已刹车')
            return True

        time.sleep(0.02)

    node.stop_stair_mode()
    return False
