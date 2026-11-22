"""基于 /odom_map 的高度增量检测正向抬升。"""
import time
import rclpy


def run(ctx, request):
    return bool(wait_lift(ctx.node)) and not ctx.cancelled


def wait_lift(node):
    """以任务开始后收到的首个高度值为基准，检测正向增量。

    current_z - start_z 达到 LIFT_CHECK_HEIGHT 时成功，下降不计入抬升量。
    本函数只读取位姿；任务结束时的停车由调度层负责。"""
    threshold = float(node.LIFT_CHECK_HEIGHT)

    node.task_progress.update(
        f"⏳ -11 抬升自检测启动：等待新的 /odom_map.z，目标抬升高度={threshold:.3f} m"
    )

    start_request_time = time.time()
    start_z = node.wait_for_fresh_odom_map_z(after_time=start_request_time, timeout=3.0)

    if start_z is None:
        return False

    node.task_progress.update(
        f"🚀 -11 开始检测抬升：起始 z0={start_z:.4f} m，"
        f"成功条件：当前 z - z0 >= {threshold:.3f} m"
    )

    while rclpy.ok():
        if node.cancel_current_task:
            node.task_progress.fail("🚫 -11 抬升检测被新任务或急停中断")
            return False

        current_z = node.odom_map_z
        if current_z is None:
            time.sleep(node.CONTROL_PERIOD)
            continue

        current_z = float(current_z)
        dz = current_z - start_z

        if dz >= threshold:
            node.task_progress.update(
                f"✅ -11 抬升检测成功：z0={start_z:.4f} m，"
                f"z={current_z:.4f} m，抬升={dz:+.4f} m，"
                f"阈值={threshold:.3f} m"
            )
            return True

        time.sleep(node.CONTROL_PERIOD)

    return False
