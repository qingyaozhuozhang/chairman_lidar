"""恢复行为：闭环后退 5cm。"""


def run(ctx, request):
    distance = float(ctx.config.get('distance', 0.05))
    return ctx.move(-1.0, 0.0, distance)
