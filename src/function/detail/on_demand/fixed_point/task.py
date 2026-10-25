"""定点导航：应用所选参数 → 发送最终目标 → 等待 Nav2 结果 → 恢复参数。"""
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.parameter import Parameter

from chairman_tasks.special import motion


def run(ctx, request):
    return ctx.go_to_point(request.target)


def navigate(node, x, y, qz, qw, desc, timeout_sec=None, speed_profile=1,
                      stage_name=None):
    """发送地图目标并等待 Nav2 结果；进入和退出时显式控制速度转发。"""
    selected = profile_key(node, speed_profile)
    if stage_name is None:
        stage_name = f'模式{selected} 直接导航【{desc}】'
    node.task_progress.stage(stage_name, '准备参数，等待 Nav2 接受目标')
    apply_profile(node, selected)

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


def begin_parameters(node, mode):
    mode = profile_key(node, mode)
    if mode not in node.mode_configs:
        raise ValueError(f'未定义速度模式 {mode}')
    names = {remote: set(values) for remote, values in node.speed_profiles[mode].items()}
    node.parameter_transaction.begin(names)
    node.active_mode = mode
    node.current_profile = None


def apply_profile(node, speed_profile):
    key = profile_key(node, speed_profile)
    if key not in node.speed_profiles:
        raise ValueError(f'未配置模式 {key}')
    if key != node.active_mode:
        raise ValueError(f'模式 {key} 不属于当前导航片段')
    if node.current_profile == key:
        return True
    # 参数拒绝时抛出异常，由调度层恢复已修改的节点。
    node.parameter_transaction.apply(profile_values(node, key))
    node.current_profile = key
    node.get_logger().debug(f'临时应用速度模式 {key}')
    return True


def restore_parameters(node):
    node.parameter_transaction.restore()
    node.current_profile = None
    node.active_mode = None


def profile_values(node, key):
    """只转换当前参数组，不合并其他模式的配置。"""
    return {remote: {name: Parameter(name, value=value).get_parameter_value()
                     for name, value in parameters.items()}
            for remote, parameters in node.speed_profiles[key].items()}


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
    """模式编号仅用于选择参数表，所有编号执行相同的导航流程。"""
    if speed_profile is None:
        return 1
    if isinstance(speed_profile, str):
        speed_profile = int(speed_profile.strip())
    if type(speed_profile) is not int or speed_profile < 1:
        raise ValueError(f'模式编号必须为正整数: {speed_profile}')
    return speed_profile
