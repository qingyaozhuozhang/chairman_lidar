"""Task registration is data; adding a task requires no dispatch-code edits."""
from importlib import import_module

from .task_context import TaskContext


def validate_task_modules(tasks):
    for task in tasks.values():
        module = import_module('chairman_tasks.' + task['module'])
        if not callable(getattr(module, 'run', None)):
            raise ValueError(f'{task["module"]} 缺少 run(ctx, request)')


def dispatch(node, request):
    task = node.task_configs.get(request.target)
    if task is None:
        if request.target not in node.PRESET_GOALS:
            raise ValueError(f'未知功能编号: {request.target}')
        task = {'module': 'fixed_point.task'}
    module = import_module('chairman_tasks.' + task['module'])
    result = module.run(TaskContext(node, task.get('parameters')), request)
    if type(result) is not bool:
        raise TypeError(f'{task["module"]}.run必须返回True或False')
    return result
