"""固定点导航：点位读取、目标发送、跟踪、完成判断及取消处理。"""
from framework.core.imports import *


def run(ctx, request):
    return ctx.go_to_point(request.target)


class FixedPointMixin:
    def get_preset_goal(self, choice):
        """读取预设点；第 6 位为 Nav2 速度/PID档位，缺省时使用档位 1。"""
        goal = self.PRESET_GOALS[choice]
        if len(goal) >= 6:
            target_x, target_y, target_qz, target_qw, desc, speed_profile = goal[:6]
        else:
            target_x, target_y, target_qz, target_qw, desc = goal[:5]
            speed_profile = 1

        return target_x, target_y, target_qz, target_qw, desc, speed_profile

    def execute_nav2_goal(self, x, y, qz, qw, desc, timeout_sec=None, speed_profile=1):
        """
        发送 Nav2 目标点，并用显式开始/显式结束控制本节点内置 tracker。
        注意：这里不依赖 Nav2 status 话题、不依赖 /cmd_vel=0、不依赖 /cmd_vel 断流超时。
        """
        actual_speed_profile = self.select_initial_nav2_profile(speed_profile, x, y, desc)
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

        self.get_logger().info(f'🚀 开始导航至【{desc}】...')

        start_time = self.get_clock().now()
        self.current_nav_goal_handle = None
        tracker_started = False

        try:
            # 显式开启 Nav2 速度转发。旧速度状态先清空，避免上一轮残留。
            self.start_nav_cmd_tracking(desc)
            tracker_started = True

            future = self._nav_client.send_goal_async(goal_msg)

            while not future.done():
                if self.cancel_current_task:
                    self.get_logger().warn(f'⚠️ 导航【{desc}】发送阶段收到取消标志')
                    return False
                time.sleep(0.01)

            goal_handle = future.result()
            self.current_nav_goal_handle = goal_handle

            if goal_handle is None:
                self.get_logger().error('❌ Nav2 goal_handle 为空')
                return False

            if not goal_handle.accepted:
                self.get_logger().error('❌ 导航请求被拒绝')
                return False

            self.get_logger().info(f'✅ Nav2 已接受目标【{desc}】')

            result_future = goal_handle.get_result_async()

            while not result_future.done():
                self.maybe_switch_nav2_profile_2_by_distance(speed_profile, x, y, desc)

                if self.cancel_current_task:
                    self.get_logger().warn('⚠️ 收到取消标志，正在取消 Nav2 导航目标！')
                    try:
                        goal_handle.cancel_goal_async()
                    except Exception as e:
                        self.get_logger().error(f'❌ 取消 Nav2 goal 失败: {e}')
                    return False

                if timeout_sec is not None:
                    elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9

                    if elapsed_time > timeout_sec:
                        self.get_logger().warn(
                            f'⏳ 导航至【{desc}】超时 ({timeout_sec}s)！强行取消并向行为树返回失败'
                        )
                        try:
                            goal_handle.cancel_goal_async()
                        except Exception as e:
                            self.get_logger().error(f'❌ 取消 Nav2 goal 失败: {e}')
                        return False

                time.sleep(0.01)

            status = result_future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'🎯 Nav2 result 已返回成功：{desc}')
                self.get_logger().info('🎉 成功到达目标点！\n')
                return True

            self.get_logger().warn(f'⚠️ 任务异常结束，状态码: {status}\n')
            return False

        finally:
            # 显式结束：无论成功、失败、取消、异常，都只从这里关闭 Nav2 速度转发并发 0。
            if tracker_started or self.nav_cmd_tracking_enabled:
                self.stop_nav_cmd_tracking(f'🛑 Nav2任务结束/中止：{desc}，显式停止底盘')
            self.current_nav_goal_handle = None

    def get_current_map_pose(self, timeout_sec=1.0, log_error=True):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec)
            )

            map_x = trans.transform.translation.x
            map_y = trans.transform.translation.y

            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            map_yaw = math.atan2(siny_cosp, cosy_cosp)

            return map_x, map_y, map_yaw

        except Exception as e:
            if log_error:
                self.get_logger().error(f'❌ 无法获取 TF 地图坐标转换 map -> base_footprint: {e}')
            return None

    def yaw_from_qz_qw(self, qz, qw):
        """预设点只给 z/w 四元数时，计算目标 yaw。"""
        qz = float(qz)
        qw = float(qw)
        return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

