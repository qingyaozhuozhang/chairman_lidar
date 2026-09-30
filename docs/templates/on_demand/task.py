"""按需任务模板：复制此文件夹到 detail/on_demand/special/<功能名>/。"""


def run(ctx, request):
    """收到一次请求执行一次；成功True，失败/取消False。"""
    ctx.log('开始执行我的任务')
    if ctx.cancelled:
        return False
    # parameters中的参数通过ctx.config获取。
    # 用ctx.wait代替长时间time.sleep，以便响应急停。
    return ctx.wait(float(ctx.config.get('wait_seconds', 0.5)))
