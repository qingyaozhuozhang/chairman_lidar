"""沿车体负 Y 方向定距移动，复用 move_forward 的闭环控制。"""


def run(ctx, request):
    return ctx.move(0.0, -1.0, float(ctx.config.get("distance", 0.2)))
