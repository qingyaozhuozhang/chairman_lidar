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

    def execute_nav2_goal(self, x, y, qz, qw, desc, timeout_sec=None, speed_profile=1,
                          stage_name=None):
        """
        发送 Nav2 目标点，并用显式开始/显式结束控制本节点内置 tracker。
        注意：这里不依赖 Nav2 status 话题、不依赖 /cmd_vel=0、不依赖 /cmd_vel 断流超时。
        """
        actual_speed_profile = self.select_initial_nav2_profile(speed_profile, x, y, desc)
        if stage_name is None:
            if actual_speed_profile == '2_sprint':
                phase = '模式2 冲刺'
            elif actual_speed_profile == 2:
                phase = '模式2 精调'
            elif actual_speed_profile == 3:
                phase = '模式3 直接导航'
            else:
                phase = '模式1 基础导航'
            stage_name = f'{phase}【{desc}】'
        self.task_progress.stage(stage_name, '准备参数，等待 Nav2 接受目标')
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

        start_time = self.get_clock().now()
        self.current_nav_goal_handle = None
        tracker_started = False
        last_progress_time = float('-inf')
        stage_result = '导航阶段未完成'

        try:
            # 显式开启 Nav2 速度转发。旧速度状态先清空，避免上一轮残留。
            self.start_nav_cmd_tracking(desc)
            tracker_started = True

            future = self._nav_client.send_goal_async(goal_msg)

            while not future.done():
                if self.cancel_current_task:
                    stage_result = '发送阶段收到中止请求，等待目标结束确认'
                    self.task_progress.update(stage_result)
                    return False
                time.sleep(0.01)

            goal_handle = future.result()
            self.current_nav_goal_handle = goal_handle

            if goal_handle is None:
                stage_result = 'Nav2 未返回目标句柄'
                self.task_progress.fail(stage_result)
                return False

            if not goal_handle.accepted:
                stage_result = 'Nav2 拒绝了导航目标'
                self.task_progress.fail(stage_result)
                return False

            self.task_progress.update('Nav2 已接受目标，正在导航')

            result_future = goal_handle.get_result_async()

            while not result_future.done():
                self.maybe_switch_nav2_profile_2_by_distance(speed_profile, x, y, desc)

                if self.cancel_current_task:
                    stage_result = '收到中止请求，等待 Nav2 动作结束确认'
                    self.task_progress.update(stage_result)
                    try:
                        goal_handle.cancel_goal_async()
                    except Exception as e:
                        self.get_logger().error(f'❌ 取消 Nav2 goal 失败: {e}')
                    return False

                if timeout_sec is not None:
                    elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9

                    if elapsed_time > timeout_sec:
                        stage_result = f'导航至【{desc}】超时（{timeout_sec}s）'
                        self.task_progress.fail(stage_result)
                        try:
                            goal_handle.cancel_goal_async()
                        except Exception as e:
                            self.get_logger().error(f'❌ 取消 Nav2 goal 失败: {e}')
                        return False

                now_s = time.monotonic()
                if now_s - last_progress_time >= 0.2:
                    pose = self.get_current_map_pose(timeout_sec=0.0, log_error=False)
                    if pose is None:
                        self.task_progress.update('Nav2 导航中；暂时无法读取 map 位姿')
                    else:
                        px, py, yaw = pose
                        distance = math.hypot(float(x) - px, float(y) - py)
                        yaw_error = abs(self.normalize_angle(self.yaw_from_qz_qw(qz, qw) - yaw))
                        self.task_progress.update(
                            f'距目标 {distance:.3f}m | 朝向误差 {math.degrees(yaw_error):.1f}°')
                    last_progress_time = now_s
                time.sleep(0.01)

            status = result_future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                stage_result = 'Nav2 确认已到达目标'
                return True

            stage_result = self.nav2_result_description(status)
            self.task_progress.fail(stage_result)
            return False

        except Exception as exc:
            stage_result = f'导航阶段异常：{exc}'
            raise
        finally:
            # 显式结束：无论成功、失败、取消、异常，都只从这里关闭 Nav2 速度转发并发 0。
            if tracker_started or self.nav_cmd_tracking_enabled:
                self.stop_nav_cmd_tracking(f'🛑 Nav2任务结束/中止：{desc}，显式停止底盘')
            self.current_nav_goal_handle = None
            self.task_progress.end_stage(stage_result)

    @staticmethod
    def nav2_result_description(status):
        names = {GoalStatus.STATUS_CANCELED: 'CANCELED：Nav2 目标被取消',
                 GoalStatus.STATUS_ABORTED: 'ABORTED：Nav2 执行失败'}
        return names.get(status, 'Nav2 返回非成功状态') + f'（状态码 {status}）'

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
