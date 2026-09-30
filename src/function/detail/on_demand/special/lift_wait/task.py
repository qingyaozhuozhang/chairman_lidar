"""等待正向抬升达到阈值。run 是框架调用入口，下面是本功能的实际实现。"""
from framework.core.imports import *


def run(ctx, request):
    return bool(wait_lift(ctx.node)) and not ctx.cancelled


def wait_lift(node):
    """
    -11 抬升自检测：
      1. 功能启动后等待一帧新的 /odom_map.z；
      2. 记录该 z 为 start_z；
      3. 只判断正向抬升：current_z - start_z >= lift_check_height 时返回成功。

    注意：
      - 不使用 abs，下降不会误判成功；
      - 不主动发布底盘零速度；
      - 循环中不反复打印检测日志，避免终端刷屏。
    """
    threshold = float(node.LIFT_CHECK_HEIGHT)

    node.get_logger().info(
        f"⏳ -11 抬升自检测启动：等待新的 /odom_map.z，目标抬升高度={threshold:.3f} m"
    )

    start_request_time = time.time()
    start_z = node.wait_for_fresh_odom_map_z(after_time=start_request_time, timeout=3.0)

    if start_z is None:
        return False

    node.get_logger().info(
        f"🚀 -11 开始检测抬升：起始 z0={start_z:.4f} m，"
        f"成功条件：当前 z - z0 >= {threshold:.3f} m"
    )

    while rclpy.ok():
        if node.cancel_current_task:
            node.get_logger().warn("🚫 -11 抬升检测被新任务或急停中断")
            return False

        current_z = node.odom_map_z
        if current_z is None:
            time.sleep(node.CONTROL_PERIOD)
            continue

        current_z = float(current_z)
        dz = current_z - start_z

        if dz >= threshold:
            node.get_logger().info(
                f"✅ -11 抬升检测成功：z0={start_z:.4f} m，"
                f"z={current_z:.4f} m，抬升={dz:+.4f} m，"
                f"阈值={threshold:.3f} m"
            )
            return True

        time.sleep(node.CONTROL_PERIOD)

    return False
