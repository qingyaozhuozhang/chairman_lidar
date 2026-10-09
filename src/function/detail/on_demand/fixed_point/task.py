"""定点导航：选择模式 → 执行目标/提前转正 → 等待结果 → 恢复参数。

模式 1 为基础导航；模式 2 根据距离切换冲刺与精调；模式 3 先转正再进点。
"""
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.parameter import Parameter

from chairman_tasks.config import dynamic_parameters
from chairman_tasks.special import motion


def run(ctx, request):
    return ctx.go_to_point(request.target)


def navigate_pre_align(node, x, y, qz, qw, desc, timeout_sec=None):
    """执行模式 3：提前对齐朝向，再导航至最终目标。

    距离大于 pre_align_distance 时，第一段目标位于当前位置至目标的连线上，
    距最终目标 pre_align_distance 米；否则先在当前位置调整朝向。
    第一段使用最终目标姿态，按位置、朝向容差及近点调整时限放行。"""
    if not node.pre_align_enabled:
        return navigate(node,
            x, y, qz, qw, desc, timeout_sec=timeout_sec, speed_profile=3
        )

    pose = node.get_current_map_pose(timeout_sec=0.20, log_error=False)
    if pose is None:
        node.task_progress.update(
            f"⚠️ 模式3提前转正：无法获取当前 map 位姿，直接导航到真实目标点：{desc}"
        )
        return navigate(node,
            x, y, qz, qw, desc, timeout_sec=timeout_sec, speed_profile=3
        )

    current_x, current_y, _ = pose
    target_x = float(x)
    target_y = float(y)
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.hypot(dx, dy)

    align_distance = max(0.0, float(node.pre_align_distance))

    if align_distance <= 1e-6:
        node.task_progress.update(
            f"⚠️ pre_align_distance={align_distance:.3f} 无效，"
            f"模式3直接导航到真实目标点：{desc}"
        )
        return navigate(node,
            x, y, qz, qw, desc, timeout_sec=timeout_sec, speed_profile=3
        )

    # 距离在配置阈值以内：不再生成提前点，直接原地先转正。
    if distance <= align_distance:
        turn_ok = navigate_alignment(node,
            current_x,
            current_y,
            qz,
            qw,
            f"{desc}（原地）"
        )

        if not turn_ok:
            return False

        if node.cancel_current_task:
            return False

        return navigate(node,
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

    first_ok = navigate_alignment(node,
        pre_x,
        pre_y,
        qz,
        qw,
        f"{desc}（提前点）"
    )

    if not first_ok:
        return False

    if node.cancel_current_task:
        return False

    return navigate(node,
        x,
        y,
        qz,
        qw,
        f"{desc}-模式3转正后进点",
        timeout_sec=timeout_sec,
        speed_profile=3,
        stage_name=f'模式3 2/2 进入真实目标【{desc}】'
    )


def navigate(node, x, y, qz, qw, desc, timeout_sec=None, speed_profile=1,
                      stage_name=None):
    """发送地图目标并等待 Nav2 结果；进入和退出时显式控制速度转发。"""
    actual_speed_profile = initial_profile(node, speed_profile, x, y, desc)
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
    node.task_progress.stage(stage_name, '准备参数，等待 Nav2 接受目标')
    apply_profile(node, actual_speed_profile)
    node.dynamic_last_check_time = None

    node._nav_client.wait_for_server()

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()

    goal_msg.pose.pose.position.x = float(x)
    goal_msg.pose.pose.position.y = float(y)
    goal_msg.pose.pose.orientation.z = float(qz)
    goal_msg.pose.pose.orientation.w = float(qw)

    start_time = node.get_clock().now()
    node.current_nav_goal_handle = None
    tracker_started = False
    last_progress_time = float('-inf')
    stage_result = '导航阶段未完成'

    try:
        # 发送目标前开启速度转发，同时清空上一段的滤波状态。
        node.start_nav_cmd_tracking(desc)
        tracker_started = True

        future = node._nav_client.send_goal_async(goal_msg)

        while not future.done():
            if node.cancel_current_task:
                stage_result = '发送阶段收到中止请求，等待目标结束确认'
                node.task_progress.update(stage_result)
                return False
            time.sleep(0.01)

        goal_handle = future.result()
        node.current_nav_goal_handle = goal_handle

        if goal_handle is None:
            stage_result = 'Nav2 未返回目标句柄'
            node.task_progress.fail(stage_result)
            return False

        if not goal_handle.accepted:
            stage_result = 'Nav2 拒绝了导航目标'
            node.task_progress.fail(stage_result)
            return False

        node.task_progress.update('Nav2 已接受目标，正在导航')

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            update_dynamic_profile(node, speed_profile, x, y, desc)

            if node.cancel_current_task:
                stage_result = '收到中止请求，等待 Nav2 动作结束确认'
                node.task_progress.update(stage_result)
                try:
                    goal_handle.cancel_goal_async()
                except Exception as e:
                    node.task_progress.fail(f'❌ 取消 Nav2 goal 失败: {e}')
                return False

            if timeout_sec is not None:
                elapsed_time = (node.get_clock().now() - start_time).nanoseconds / 1e9

                if elapsed_time > timeout_sec:
                    stage_result = f'导航至【{desc}】超时（{timeout_sec}s）'
                    node.task_progress.fail(stage_result)
                    try:
                        goal_handle.cancel_goal_async()
                    except Exception as e:
                        node.task_progress.fail(f'❌ 取消 Nav2 goal 失败: {e}')
                    return False

            now_s = time.monotonic()
            if now_s - last_progress_time >= 0.2:
                pose = node.get_current_map_pose(timeout_sec=0.0, log_error=False)
                if pose is None:
                    node.task_progress.update('Nav2 导航中；暂时无法读取 map 位姿')
                else:
                    px, py, yaw = pose
                    distance = math.hypot(float(x) - px, float(y) - py)
                    yaw_error = abs(motion.normalize_angle(yaw_from_qz_qw(node, qz, qw) - yaw))
                    node.task_progress.update(
                        f'距目标 {distance:.3f}m | 朝向误差 {math.degrees(yaw_error):.1f}°')
                last_progress_time = now_s
            time.sleep(0.01)

        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            stage_result = 'Nav2 确认已到达目标'
            return True

        stage_result = result_description(status)
        node.task_progress.fail(stage_result)
        return False

    except Exception as exc:
        stage_result = f'导航阶段异常：{exc}'
        raise
    finally:
        # 所有退出路径均关闭速度转发并发送零速度。
        if tracker_started or node.nav_cmd_tracking_enabled:
            node.stop_nav_cmd_tracking(f'🛑 Nav2任务结束/中止：{desc}，显式停止底盘')
        node.current_nav_goal_handle = None
        node.task_progress.end_stage(stage_result)


def navigate_alignment(node, x, y, qz, qw, desc):
    """执行模式 3 的第一段，达到位置容差后检查朝向和近点调整时限。

    朝向满足容差或调整超时后允许放行；发送第二段前须确认第一段 Action 终止。"""
    node.task_progress.stage(f'模式3 1/2 提前转正【{desc}】', '准备参数，等待 Nav2 接受目标')
    actual_speed_profile = initial_profile(node, 3, x, y, desc)
    apply_profile(node, actual_speed_profile)
    node.dynamic_last_check_time = None

    node._nav_client.wait_for_server()

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = float(x)
    goal_msg.pose.pose.position.y = float(y)
    goal_msg.pose.pose.orientation.z = float(qz)
    goal_msg.pose.pose.orientation.w = float(qw)

    target_x = float(x)
    target_y = float(y)
    target_yaw = yaw_from_qz_qw(node, qz, qw)

    release_xy = max(0.0, float(node.pre_align_release_xy_tolerance))
    release_yaw = max(0.0, float(node.pre_align_release_yaw_tolerance))
    near_adjust_timeout = max(0.0, float(node.pre_align_near_adjust_timeout))

    node.current_nav_goal_handle = None
    tracker_started = False
    near_start_time = None
    stage_result = '第一段未完成'

    try:
        node.start_nav_cmd_tracking(desc)
        tracker_started = True

        future = node._nav_client.send_goal_async(goal_msg)

        while not future.done():
            if node.cancel_current_task:
                stage_result = '发送阶段收到中止请求'
                return False
            time.sleep(0.01)

        goal_handle = future.result()
        node.current_nav_goal_handle = goal_handle

        if goal_handle is None:
            stage_result = 'Nav2 未返回第一段目标句柄'
            node.task_progress.fail(stage_result)
            return False

        if not goal_handle.accepted:
            stage_result = 'Nav2 拒绝了第一段导航目标'
            node.task_progress.fail(stage_result)
            return False

        node.task_progress.update('Nav2 已接受第一段目标')
        result_future = goal_handle.get_result_async()

        while not result_future.done():
            if node.cancel_current_task:
                stage_result = '第一段收到中止请求'
                return False

            pose = node.get_current_map_pose(timeout_sec=0.01, log_error=False)
            if pose is not None:
                current_x, current_y, current_yaw = pose
                dist_error = math.hypot(target_x - current_x, target_y - current_y)
                yaw_error = abs(motion.normalize_angle(target_yaw - current_yaw))

                now_s = time.monotonic()
                node.task_progress.update(
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
                node.task_progress.update('第一段导航中；暂时无法读取 map 位姿')

            time.sleep(0.01)

        result = result_future.result()
        status = result.status if result is not None else None

        if status == GoalStatus.STATUS_SUCCEEDED:
            stage_result = 'Nav2 确认已到达第一段目标'
            return True

        stage_result = result_description(status)
        node.task_progress.fail(stage_result)
        return False

    finally:
        try:
            if tracker_started or node.nav_cmd_tracking_enabled:
                node.stop_nav_cmd_tracking(f'🛑 模式3第一段结束/放行：{desc}，显式停止底盘')
            # 取消响应不代表动作终止；须等待第一段最终结果后才能发送第二段。
            # 超时保留 pending 并抛出异常，由调度层继续收尾。
            if node._nav_client.pending:
                node.task_progress.update('阶段交接：等待第一段 Nav2 动作结束确认')
            node._nav_client.settle()
        except Exception:
            stage_result = '第一段动作结束未能确认，停止后续导航'
            raise
        finally:
            node.current_nav_goal_handle = None
            node.task_progress.end_stage(stage_result)


def initial_profile(node, speed_profile, target_x, target_y, desc):
    """导航开始前选择实际应用的 Nav2 档位。模式 2 会根据起点距离决定冲刺/精调。"""
    selected = profile_key(node, speed_profile)

    if selected != 2:
        return selected

    distance = distance_to_goal(node, target_x, target_y, timeout_sec=0.05)
    if distance is not None and distance <= node.dynamic_switch_distance:
        return 2

    if distance is None:
        node.task_progress.update(
            f'[模式2] 暂时无法获取 map 位姿，先使用冲刺档；'
            f'位姿恢复后按 {node.dynamic_switch_distance:.2f}m 阈值切换精调'
        )

    return node.dynamic_sprint_key


def update_dynamic_profile(node, requested_speed_profile, target_x, target_y, desc):
    """模式 2 进入切换距离后应用精调参数，后续不再切回冲刺。"""
    requested_key = profile_key(node, requested_speed_profile)
    if requested_key != 2:
        return

    # 已经切到精调档后不再切回冲刺，避免阈值附近来回切换。
    if node.current_profile == 2:
        return

    now = node.get_clock().now()
    if node.dynamic_last_check_time is not None:
        elapsed = (now - node.dynamic_last_check_time).nanoseconds / 1e9
        if elapsed < node.dynamic_switch_check_period:
            return
    node.dynamic_last_check_time = now

    distance = distance_to_goal(node, target_x, target_y, timeout_sec=0.01)
    if distance is None:
        return

    if distance <= node.dynamic_switch_distance:
        node.task_progress.update('已进入精调距离，正在切换速度参数')
        apply_profile(node, 2)
        node.task_progress.end_stage(
            f'距目标 {distance:.3f}m <= {node.dynamic_switch_distance:.3f}m，精调参数已生效')
        node.task_progress.stage(f'模式2 精调【{desc}】', '继续当前 Nav2 目标，进行到点精调')


def begin_parameters(node, mode):
    mode = profile_key(node, mode)
    if mode not in node.mode_configs:
        raise ValueError(f'未定义速度模式 {mode}')
    base = node.mode_configs[mode]['base_mode']
    keys = (2, '2_sprint') if base == 2 else (base,)
    names = {}
    for key in keys:
        for remote, values in profile_values(node, key, mode).items():
            names.setdefault(remote, set()).update(values)
    node.parameter_transaction.begin(names)
    node.active_mode = mode
    node.current_profile = None


def apply_profile(node, speed_profile):
    key = profile_key(node, speed_profile)
    if key not in node.speed_profiles:
        raise ValueError(f'未配置模式 {key}')
    if node.current_profile == key:
        return True
    # 参数拒绝时抛出异常，由调度层恢复已修改的节点。
    node.parameter_transaction.apply(profile_values(node, key))
    node.current_profile = key
    node.get_logger().debug(f'临时应用速度模式 {node.active_mode}，内部阶段 {key}')
    return True


def restore_parameters(node):
    node.parameter_transaction.restore()
    node.current_profile = None
    node.active_mode = None


def profile_values(node, key, mode_id=None):
    requested = node.active_mode if mode_id is None else mode_id
    if requested is None:
        requested = 2 if key == '2_sprint' else key
    base = node.mode_configs[requested]['base_mode']
    section = 'sprint_parameters' if key == '2_sprint' else 'parameters'
    values = {}
    for mode in dict.fromkeys((base, requested)):
        config = node.mode_configs[mode]
        sections = ('parameters', section) if mode != base and section == 'sprint_parameters' else (section,)
        for selected in sections:
            for remote, parameters in dynamic_parameters(config.get(selected, {})).items():
                values.setdefault(remote, {}).update(parameters)
    return {remote: {name: Parameter(name, value=value).get_parameter_value()
                     for name, value in parameters.items()} for remote, parameters in values.items()}


def distance_to_goal(node, target_x, target_y, timeout_sec=0.02):
    """使用 TF 的 map -> base_footprint 坐标计算当前距离目标点的平面距离。"""
    pose = node.get_current_map_pose(timeout_sec=timeout_sec, log_error=False)
    if pose is None:
        return None

    current_map_x, current_map_y, _ = pose
    return math.hypot(float(target_x) - current_map_x, float(target_y) - current_map_y)


def yaw_from_qz_qw(node, qz, qw):
    """由平面四元数 qz/qw 计算目标航向，返回弧度。"""
    qz = float(qz)
    qw = float(qw)
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def result_description(status):
    names = {GoalStatus.STATUS_CANCELED: 'CANCELED：Nav2 目标被取消',
             GoalStatus.STATUS_ABORTED: 'ABORTED：Nav2 执行失败'}
    return names.get(status, 'Nav2 返回非成功状态') + f'（状态码 {status}）'


def profile_key(node, speed_profile):
    """统一模式键的表示：数值编号转为整数，保留冲刺阶段等字符串键。"""
    if speed_profile is None:
        return 1

    if isinstance(speed_profile, str):
        value = speed_profile.strip()
        if value in node.speed_profiles:
            return value
        try:
            return int(value)
        except Exception:
            return value

    try:
        return int(speed_profile)
    except Exception:
        return speed_profile
