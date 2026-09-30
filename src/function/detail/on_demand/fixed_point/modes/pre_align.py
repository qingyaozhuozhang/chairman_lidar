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
        """
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

        self.get_logger().info(
            f'🚀 模式3第一段快速调整开始【{desc}】：'
            f'提前点xy放行={release_xy:.3f}m，'
            f'yaw放行={math.degrees(release_yaw):.1f}°，'
            f'近点最多调整={near_adjust_timeout:.2f}s'
        )

        self.current_nav_goal_handle = None
        tracker_started = False
        near_start_time = None
        last_debug_time = 0.0

        try:
            self.start_nav_cmd_tracking(desc)
            tracker_started = True

            future = self._nav_client.send_goal_async(goal_msg)

            while not future.done():
                if self.cancel_current_task:
                    self.get_logger().warn(f'⚠️ 模式3第一段【{desc}】发送阶段收到取消标志')
                    return False
                time.sleep(0.01)

            goal_handle = future.result()
            self.current_nav_goal_handle = goal_handle

            if goal_handle is None:
                self.get_logger().error('❌ 模式3第一段 Nav2 goal_handle 为空')
                return False

            if not goal_handle.accepted:
                self.get_logger().error(f'❌ 模式3第一段导航请求被拒绝：{desc}')
                return False

            self.get_logger().info(f'✅ 模式3第一段 Nav2 已接受目标【{desc}】')
            result_future = goal_handle.get_result_async()

            while not result_future.done():
                if self.cancel_current_task:
                    self.get_logger().warn('⚠️ 收到取消标志，正在取消模式3第一段 Nav2 目标！')
                    try:
                        cancel_future = goal_handle.cancel_goal_async()
                        self.wait_future_done(cancel_future, timeout_sec=0.5)
                    except Exception as e:
                        self.get_logger().error(f'❌ 取消模式3第一段 Nav2 goal 失败: {e}')
                    return False

                pose = self.get_current_map_pose(timeout_sec=0.01, log_error=False)
                if pose is not None:
                    current_x, current_y, current_yaw = pose
                    dist_error = math.hypot(target_x - current_x, target_y - current_y)
                    yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))

                    now_s = time.time()

                    if dist_error <= release_xy:
                        if near_start_time is None:
                            near_start_time = now_s
                            self.get_logger().info(
                                f'📍 模式3第一段已到提前点附近：'
                                f'xy误差={dist_error:.3f}m <= {release_xy:.3f}m，开始快速放行判断'
                            )

                        near_elapsed = now_s - near_start_time

                        if yaw_error <= release_yaw:
                            self.get_logger().info(
                                f'✅ 模式3第一段提前放行：'
                                f'xy误差={dist_error:.3f}m，'
                                f'yaw误差={math.degrees(yaw_error):.2f}° <= {math.degrees(release_yaw):.2f}°，'
                                f'直接进入真实目标点'
                            )
                            try:
                                cancel_future = goal_handle.cancel_goal_async()
                                self.wait_future_done(cancel_future, timeout_sec=0.5)
                            except Exception as e:
                                self.get_logger().warn(f'⚠️ 模式3第一段放行时取消 Nav2 goal 异常: {e}')
                            return True

                        if near_adjust_timeout <= 1e-6 or near_elapsed >= near_adjust_timeout:
                            self.get_logger().warn(
                                f'⏩ 模式3第一段近点调整到时放行：'
                                f'已在提前点附近调整 {near_elapsed:.2f}s，'
                                f'xy误差={dist_error:.3f}m，'
                                f'yaw误差={math.degrees(yaw_error):.2f}°，'
                                f'直接进入真实目标点'
                            )
                            try:
                                cancel_future = goal_handle.cancel_goal_async()
                                self.wait_future_done(cancel_future, timeout_sec=0.5)
                            except Exception as e:
                                self.get_logger().warn(f'⚠️ 模式3第一段超时放行时取消 Nav2 goal 异常: {e}')
                            return True

                    else:
                        near_start_time = None

                    if now_s - last_debug_time > 0.30:
                        self.get_logger().info(
                            f'🧭 模式3第一段调整中：'
                            f'xy误差={dist_error:.3f}m，'
                            f'yaw误差={math.degrees(yaw_error):.2f}°'
                        )
                        last_debug_time = now_s

                time.sleep(0.01)

            result = result_future.result()
            status = result.status if result is not None else None

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'🎯 模式3第一段 Nav2 已自然成功：{desc}')
                return True

            self.get_logger().warn(f'⚠️ 模式3第一段任务异常结束，状态码: {status}')
            return False

        finally:
            if tracker_started or self.nav_cmd_tracking_enabled:
                self.stop_nav_cmd_tracking(f'🛑 模式3第一段结束/放行：{desc}，显式停止底盘')
            self.current_nav_goal_handle = None

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
        target_yaw = self.yaw_from_qz_qw(qz, qw)
        yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))

        self.get_logger().info(
            f"🧭 模式3提前转正两段式导航：{desc}，"
            f"距离={distance:.3f}m，阈值={align_distance:.3f}m，"
            f"目标yaw差={math.degrees(yaw_error):.2f}°"
        )

        if align_distance <= 1e-6:
            self.get_logger().warn(
                f"⚠️ nav2_profile_3_pre_align_distance={align_distance:.3f} 无效，"
                f"模式3直接导航到真实目标点：{desc}"
            )
            return self.execute_nav2_goal(
                x, y, qz, qw, desc, timeout_sec=timeout_sec, speed_profile=3
            )

        # 距离目标点已经在 0.8m 阈值以内：不再生成提前点，直接原地先转正。
        if distance <= align_distance:
            self.get_logger().info(
                f"📍 当前距离目标 {distance:.3f}m <= {align_distance:.3f}m，"
                f"模式3先在当前位置原地转正，再进入目标点"
            )

            turn_ok = self.execute_nav2_goal_until_profile3_first_stage_ready(
                current_x,
                current_y,
                qz,
                qw,
                f"{desc}-模式3原地提前转正"
            )

            if not turn_ok:
                return False

            if self.cancel_current_task:
                return False

            self.get_logger().info(
                f"➡️ 模式3转正完成，保持目标姿态平移进入真实目标点：{desc}"
            )
            return self.execute_nav2_goal(
                x,
                y,
                qz,
                qw,
                f"{desc}-模式3转正后进点",
                timeout_sec=timeout_sec,
                speed_profile=3
            )

        # 距离目标点在 0.8m 阈值以外：先到真实目标点前方 0.8m 的提前点，并使用最终目标姿态。
        unit_x = dx / distance
        unit_y = dy / distance
        pre_x = target_x - unit_x * align_distance
        pre_y = target_y - unit_y * align_distance

        self.get_logger().info(
            f"📍 当前距离目标 {distance:.3f}m > {align_distance:.3f}m，"
            f"模式3先到提前转正点 ({pre_x:.3f}, {pre_y:.3f})，"
            f"该点距离真实目标约 {align_distance:.3f}m，使用最终目标姿态"
        )

        first_ok = self.execute_nav2_goal_until_profile3_first_stage_ready(
            pre_x,
            pre_y,
            qz,
            qw,
            f"{desc}-模式3提前转正点"
        )

        if not first_ok:
            return False

        if self.cancel_current_task:
            return False

        self.get_logger().info(
            f"➡️ 模式3提前转正完成，保持目标姿态平移进入真实目标点：{desc}"
        )
        return self.execute_nav2_goal(
            x,
            y,
            qz,
            qw,
            f"{desc}-模式3转正后进点",
            timeout_sec=timeout_sec,
            speed_profile=3
        )
