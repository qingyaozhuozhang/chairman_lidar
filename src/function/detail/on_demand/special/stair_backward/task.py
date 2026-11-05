"""沿车体负 X 方向登阶，复用 stair_forward 的周期纠偏与停止控制。"""


def run(ctx, request):
    return ctx.stair(-1, 0, float(ctx.config.get("speed", 0.3)), "向后")
