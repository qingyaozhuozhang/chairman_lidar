"""固定点导航模式的实现。"""
from framework.core.imports import *

class PreAlignMixin:
    def is_nav2_profile_3(self, speed_profile):
        return self.normalize_nav2_speed_profile_key(speed_profile) == 3

    def execute_nav2_goal_until_profile3_first_stage_ready(self, x, y, qz, qw, desc):
        """
        模式 3 第一段专用 Nav2 导航：
          - 不再等待 Nav2 对提前点返回完全成功；
          - 到提前点附近后，yaw 差不多就放行；
          - 如果已经到提前点附近但 yaw 还没完全调好，最多再等
            nav2_profile_3_pre_align_near_adjust_timeout 秒，然后直接进入真实目标点。
          - 放行后先停止速度转发，并确认第一段 Action 返回终态，才允许发送第二段。
        """
        self.task_progress.stage(f'模式3 1/2 提前转正【{desc}】', '准备参数，等待 Nav2 接受目标')
        actual_speed_profile = self.select_initial_nav2_profile(3, x, y, desc)
        self.apply_nav2_speed_profile(actual_speed_profile)
        self.nav2_dynamic_last_check_time = None

        self._nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        target_x = float(x)
        target_y = float(y)
        target_yaw = self.yaw_from_qz_qw(qz, qw)

        release_xy = max(0.0, float(self.nav2_profile_3_pre_align_release_xy_tolerance))
        release_yaw = max(0.0, float(self.nav2_profile_3_pre_align_release_yaw_tolerance))
        near_adjust_timeout = max(0.0, float(self.nav2_profile_3_pre_align_near_adjust_timeout))

        self.current_nav_goal_handle = None
        tracker_started = False
        near_start_time = None
        stage_result = '第一段未完成'

        try:
            self.start_nav_cmd_tracking(desc)
            tracker_started = True

            future = self._nav_client.send_goal_async(goal_msg)

            while not future.done():
                if self.cancel_current_task:
                    stage_result = '发送阶段收到中止请求'
                    return False
                time.sleep(0.01)

            goal_handle = future.result()
            self.current_nav_goal_handle = goal_handle

            if goal_handle is None:
                stage_result = 'Nav2 未返回第一段目标句柄'
                self.task_progress.fail(stage_result)
                return False

            if not goal_handle.accepted:
                stage_result = 'Nav2 拒绝了第一段导航目标'
                self.task_progress.fail(stage_result)
                return False

            self.task_progress.update('Nav2 已接受第一段目标')
            result_future = goal_handle.get_result_async()

            while not result_future.done():
                if self.cancel_current_task:
                    stage_result = '第一段收到中止请求'
                    return False

                pose = self.get_current_map_pose(timeout_sec=0.01, log_error=False)
                if pose is not None:
                    current_x, current_y, current_yaw = pose
                    dist_error = math.hypot(target_x - current_x, target_y - current_y)
                    yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))

                    now_s = time.monotonic()
                    self.task_progress.update(
                        f'距提前点 {dist_error:.3f}m | 朝向误差 {math.degrees(yaw_error):.1f}°')

                    if dist_error <= release_xy:
                        if near_start_time is None:
                            near_start_time = now_s

                        near_elapsed = now_s - near_start_time

                        if yaw_error <= release_yaw:
                            stage_result = (f'达到放行条件：距离 {dist_error:.3f}m，'
                                            f'朝向误差 {math.degrees(yaw_error):.1f}°；第一段动作已结束')
                            return True

                        if near_adjust_timeout <= 1e-6 or near_elapsed >= near_adjust_timeout:
                            stage_result = (f'近点调整达到 {near_adjust_timeout:.2f}s 上限，按配置放行；'
                                            f'朝向误差仍为 {math.degrees(yaw_error):.1f}°；第一段动作已结束')
                            return True

                    else:
                        near_start_time = None

                else:
                    self.task_progress.update('第一段导航中；暂时无法读取 map 位姿')

                time.sleep(0.01)

            result = result_future.result()
            status = result.status if result is not None else None

            if status == GoalStatus.STATUS_SUCCEEDED:
                stage_result = 'Nav2 确认已到达第一段目标'
                return True

            stage_result = self.nav2_result_description(status)
            self.task_progress.fail(stage_result)
            return False

        finally:
            try:
                if tracker_started or self.nav_cmd_tracking_enabled:
                    self.stop_nav_cmd_tracking(f'🛑 模式3第一段结束/放行：{desc}，显式停止底盘')
                # 取消请求的响应仅表示已受理，不表示旧行为树已经停止。
                # Nav2 在取消收尾时会 terminate_all()，过早发送的第二段也可能被 ABORT。
                # settle() 等待旧目标 result（包括迟到的接受响应）；超时抛错并保留
                # pending，交给框架恢复流程处理，禁止继续发送第二段。
                if self._nav_client.pending:
                    self.task_progress.update('阶段交接：等待第一段 Nav2 动作结束确认')
                self._nav_client.settle()
            except Exception:
                stage_result = '第一段动作结束未能确认，停止后续导航'
                raise
            finally:
                self.current_nav_goal_handle = None
                self.task_progress.end_stage(stage_result)

    def execute_profile3_pre_yaw_align_goal(self, x, y, qz, qw, desc, timeout_sec=None):
        """模式 3 专用提前转正两段式导航。

        规则只使用 nav2_profile_3_pre_align_distance 这一个距离阈值：
          - 当前距离目标点 > 阈值：先到目标点前方阈值距离处，并在该点转到最终目标姿态；
          - 当前距离目标点 <= 阈值：先在当前位置原地转到最终目标姿态；
          - 转正后再保持最终目标姿态平移进入真实目标点。

        不再使用第一段提前转正 timeout，也不再使用 min_distance。
        """
        if not self.nav2_profile_3_pre_align_enabled:
            return self.execute_nav2_goal(
                x, y, qz, qw, desc, timeout_sec=timeout_sec, speed_profile=3
            )

        pose = self.get_current_map_pose(timeout_sec=0.20, log_error=False)
        if pose is None:
            self.get_logger().warn(
                f"⚠️ 模式3提前转正：无法获取当前 map 位姿，直接导航到真实目标点：{desc}"
            )
            return self.execute_nav2_goal(
                x, y, qz, qw, desc, timeout_sec=timeout_sec, speed_profile=3
            )

        current_x, current_y, current_yaw = pose
        target_x = float(x)
        target_y = float(y)
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        align_distance = max(0.0, float(self.nav2_profile_3_pre_align_distance))

        if align_distance <= 1e-6:
            self.get_logger().warn(
                f"⚠️ nav2_profile_3_pre_align_distance={align_distance:.3f} 无效，"
                f"模式3直接导航到真实目标点：{desc}"
            )
            return self.execute_nav2_goal(
                x, y, qz, qw, desc, timeout_sec=timeout_sec, speed_profile=3
            )

        # 距离在配置阈值以内：不再生成提前点，直接原地先转正。
        if distance <= align_distance:
            turn_ok = self.execute_nav2_goal_until_profile3_first_stage_ready(
                current_x,
                current_y,
                qz,
                qw,
                f"{desc}（原地）"
            )

            if not turn_ok:
                return False

            if self.cancel_current_task:
                return False

            return self.execute_nav2_goal(
                x,
                y,
                qz,
                qw,
                f"{desc}-模式3转正后进点",
                timeout_sec=timeout_sec,
                speed_profile=3,
                stage_name=f'模式3 2/2 进入真实目标【{desc}】'
            )

        # 距离在配置阈值以外：先到提前点，并使用最终目标姿态。
        unit_x = dx / distance
        unit_y = dy / distance
        pre_x = target_x - unit_x * align_distance
        pre_y = target_y - unit_y * align_distance

        first_ok = self.execute_nav2_goal_until_profile3_first_stage_ready(
            pre_x,
            pre_y,
            qz,
            qw,
            f"{desc}（提前点）"
        )

        if not first_ok:
            return False

        if self.cancel_current_task:
            return False

        return self.execute_nav2_goal(
            x,
            y,
            qz,
            qw,
            f"{desc}-模式3转正后进点",
            timeout_sec=timeout_sec,
            speed_profile=3,
            stage_name=f'模式3 2/2 进入真实目标【{desc}】'
        )
