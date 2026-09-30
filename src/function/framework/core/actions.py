"""原功能的共享实现；新增任务通常不需要修改这里。"""
import time

import rclpy

class TrackedActionClient:
    def __init__(self, owner, client, timeout=3.0):
        self.owner = owner
        self.client = client
        self.timeout = timeout
        self.pending = []
        self.settling = False

    def wait_for_server(self):
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not self.owner.cancel_current_task:
            if self.client.wait_for_server(timeout_sec=0.1):
                return True
            if time.monotonic() >= deadline:
                raise TimeoutError('navigate_to_pose 动作服务未就绪')
        raise RuntimeError('等待动作服务期间任务已取消')

    def send_goal_async(self, goal):
        if self.owner.cancel_current_task:
            raise RuntimeError('发送目标前任务已取消')
        future = self.client.send_goal_async(goal)
        self.pending.append(future)
        future.add_done_callback(self.cancel_late_goal)
        return future

    def cancel_late_goal(self, future):
        if self.settling:
            try:
                handle = future.result()
                if handle and handle.accepted:
                    handle.cancel_goal_async()
            except Exception as exc:
                self.owner.get_logger().error(f'迟到目标取消失败: {exc}')

    def wait(self, future):
        deadline = time.monotonic() + self.timeout
        while not future.done():
            if time.monotonic() >= deadline or not rclpy.ok():
                raise TimeoutError('Nav2目标尚未确认结束；禁止下一任务，请调用参数恢复服务重试')
            time.sleep(0.01)
        return future.result()

    def settle(self):
        self.settling = True
        for future in list(self.pending):
            handle = self.wait(future)
            if handle and handle.accepted:
                result = handle.get_result_async()
                if not result.done():
                    self.wait(handle.cancel_goal_async())
                self.wait(result)
            self.pending.remove(future)
        self.settling = False
