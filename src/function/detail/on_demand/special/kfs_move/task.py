"""将车体坐标系中的 KFS 平面偏置转换为闭环移动方向与距离。"""
import math
from chairman_tasks.special.move_forward.task import move


def run(ctx, request):
    return ctx.manual(lambda: move_offset(ctx.node, request.kfs_offset))


def move_offset(node, kfs_offset):
    offset_x = float(kfs_offset.x)
    offset_y = float(kfs_offset.y)
    target_distance = math.hypot(offset_x, offset_y)

    node.task_progress.update(
        f'🎯 KFS偏置：x={offset_x:+.3f} m，'
        f'y={offset_y:+.3f} m，合成距离={target_distance:.3f} m'
    )

    return move(node, offset_x, offset_y, target_distance)
