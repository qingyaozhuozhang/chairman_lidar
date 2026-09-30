"""新跑点流程模板：多个点按顺序执行，每段恢复临时参数。"""


def run(ctx, request):
    for point_id in ctx.config['points']:
        if ctx.cancelled:
            return False
        if not ctx.go_to_point(point_id, mode=ctx.config.get('mode')):
            return False
    return True
