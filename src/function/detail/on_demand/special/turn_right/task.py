"""按配置角度右转，复用 turn_left 的旋转控制。"""


def run(ctx, request):
    return ctx.rotate(float(ctx.config.get("angle_degrees", -90.0)))
