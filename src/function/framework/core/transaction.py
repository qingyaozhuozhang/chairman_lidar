"""Per-task live parameter snapshot. Independent of ROS for failure-path tests."""
from copy import deepcopy


class ParameterTransaction:
    def __init__(self, backend):
        self.backend = backend
        self.snapshot = {}
        self.touched = []
        self.active = False

    def begin(self, names_by_node):
        if self.active:
            raise RuntimeError('上一任务参数尚未恢复，请先调用 /restore_navigation_parameters')
        snapshot = {node: self.backend.get(node, list(names)) for node, names in names_by_node.items()}
        self.snapshot = deepcopy(snapshot)
        self.active = True

    def apply(self, values_by_node):
        if not self.active:
            raise RuntimeError('参数修改必须在任务事务内执行')
        for node, values in values_by_node.items():
            if not set(values).issubset(self.snapshot.get(node, {})):
                raise ValueError(f'{node}: 包含未保存原值的参数')
        for node, values in values_by_node.items():
            # A timeout can happen after the remote node accepted a request.
            if node not in self.touched:
                self.touched.append(node)
            self.backend.set(node, values)

    def restore(self):
        if not self.active:
            return
        # Never let a late mode-setting request overtake the restoration.
        self.backend.settle()
        failures = []
        for node in list(reversed(self.touched)):
            try:
                values = self.snapshot[node]
                self.backend.set(node, values)
                if self.backend.get(node, list(values)) != values:
                    raise RuntimeError('恢复后读回值不一致')
                self.touched.remove(node)
            except Exception as exc:
                failures.append(f'{node}: {exc}')
        if failures:
            raise RuntimeError('参数恢复未完成；禁止下一任务：' + '; '.join(failures))
        self.snapshot = {}
        self.active = False
