"""计算目标并调用定点导航。run 是框架调用入口，下面是本功能的实际实现。"""
from framework.core.imports import *


def run(ctx, request):
    mode = int(ctx.config.get('mode', ctx.point_mode(request.target)))
    return ctx.navigation(mode, lambda base: navigate(ctx.node, request.kfs_offset, speed_profile=base))


def navigate(node, kfs_offset, speed_profile=1):
    pose = node.get_current_map_pose()

    if pose is None:
        return False

    current_x, current_y, current_yaw = pose

    local_x = float(kfs_offset.x)
    local_y = float(kfs_offset.y)

    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)

    map_offset_x = local_x * cos_yaw - local_y * sin_yaw
    map_offset_y = local_x * sin_yaw + local_y * cos_yaw

    target_x = current_x + map_offset_x
    target_y = current_y + map_offset_y

    target_yaw = node.snap_yaw_to_nearest_90(current_yaw)
    target_qz = math.sin(target_yaw / 2.0)
    target_qw = math.cos(target_yaw / 2.0)

    offset_distance = math.hypot(local_x, local_y)

    node.get_logger().info(
        f'🎯 16号KFS局部偏置导航：'
        f'偏置距离 local_x={local_x:+.3f} m，'
        f'local_y={local_y:+.3f} m，'
        f'合成距离={offset_distance:.3f} m'
    )

    return node.execute_nav2_goal(
        target_x,
        target_y,
        target_qz,
        target_qw,
        (
            f'KFS局部偏置导航 '
            f'local_x={local_x:+.3f}, local_y={local_y:+.3f}'
        ),
        speed_profile=speed_profile
    )
