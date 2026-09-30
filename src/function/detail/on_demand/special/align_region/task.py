"""计算目标并调用定点导航。run 是框架调用入口，下面是本功能的实际实现。"""
from framework.core.imports import *


def run(ctx, request):
    mode = int(ctx.config.get('mode', ctx.point_mode(request.target)))
    return ctx.navigation(mode, lambda base: navigate(ctx.node, speed_profile=base))


def navigate(node, speed_profile=1):
    node.get_logger().info('正在通过 TF 树获取小车绝对地图坐标...')

    pose = node.get_current_map_pose()

    if pose is None:
        return False

    map_x, map_y, map_yaw = pose

    target_region = None

    for rid, bounds in node.merlin_regions.items():
        if bounds[0] <= map_x <= bounds[1] and bounds[2] <= map_y <= bounds[3]:
            target_region = (rid, bounds)
            break

    if target_region is None:
        node.get_logger().warn(
            f'小车当前坐标 ({map_x:.3f}, {map_y:.3f}) 不在任何梅林方块区域内！'
        )
        return False

    rid, bounds = target_region

    center_x = (bounds[0] + bounds[1]) / 2.0
    center_y = (bounds[2] + bounds[3]) / 2.0

    target_yaw_rad = node.snap_yaw_to_nearest_90(map_yaw)

    qz = math.sin(target_yaw_rad / 2.0)
    qw = math.cos(target_yaw_rad / 2.0)

    node.get_logger().info(
        f'🎯 目标: ID {rid} ({bounds[4]}) 中心点 ({center_x:.3f}, {center_y:.3f})'
    )

    return node.execute_nav2_goal(
        center_x,
        center_y,
        qz,
        qw,
        f"方块{rid - 1}中心对齐",
        timeout_sec=10.0,
        speed_profile=speed_profile
    )
