"""恢复行为：开环后退指定时长，带打断/超时/零速保护。"""
import time
import rclpy
from geometry_msgs.msg import Twist


def run(ctx, request):
    duration = float(ctx.config.get('backward_duration', 2.0))  # 后退时长(s)
    speed    = float(ctx.config.get('backward_speed', -0.05))    # 后退线速度(m/s, 取绝对值)
    return ctx.manual(lambda: backward_recovery(ctx.node, duration, speed))


def backward_recovery(node, duration=2.0, speed=-0.05):

    if duration <= 0.0 or speed == 0.0:
        node.task_progress.fail('⚠️ 后退恢复参数无效，任务取消')
        return False

    node.task_progress.update(
        f'🔙 开环后退恢复: 时长={duration:.2f}s, 速度={speed:.3f} m/s'
    )

    twist_msg = Twist()
    twist_msg.linear.y  = 0.0
    twist_msg.angular.z = 0.0
    twist_msg.linear.x = speed

    start_time     = time.monotonic()
    last_debug_time = start_time

    try:
        while rclpy.ok():
            if node.cancel_current_task:
                node.task_progress.update('🚫 后退恢复被行为树中止')
                return False

            now = time.monotonic()

            elapsed = now - start_time
            if elapsed >= duration:
                node.task_progress.update(
                    f'✅ 后退恢复完成: elapsed={elapsed:.2f}s'
                )
                return True
            
            node.publish_twist_speed(twist_msg)

            if now - last_debug_time > 0.3:
                node.task_progress.update(
                    f'🔙 后退中: elapsed={elapsed:.2f}s / {duration:.2f}s '
                )
                last_debug_time = now
            time.sleep(node.CONTROL_PERIOD)

    finally:
        node.publish_manual_zero_speed()
    return False