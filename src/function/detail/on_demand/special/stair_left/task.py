"""沿车体正 Y 方向登阶，复用 stair_forward 的周期纠偏与停止控制。"""


def run(ctx, request):
    return ctx.stair(0, 1, float(ctx.config.get("speed", 0.1)), "向左")
