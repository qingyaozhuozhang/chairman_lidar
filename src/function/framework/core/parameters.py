"""公共参数通信：读取、原子赋值、保存原值和恢复确认。"""
import time
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from copy import deepcopy


class RosParameterBackend:
    def __init__(self, node, timeout=3.0):
        self.node = node
        self.timeout = timeout
        self.clients = {}
        self.pending = []

    def client(self, node, kind):
        key = (node, kind)
        if key not in self.clients:
            service_type, suffix = (GetParameters, 'get_parameters') if kind == 'get' else (SetParametersAtomically, 'set_parameters_atomically')
            self.clients[key] = self.node.create_client(service_type, f'/{node.strip("/")}/{suffix}', callback_group=self.node.reentrant_group)
        return self.clients[key]

    def wait(self, future):
        deadline = time.monotonic() + self.timeout
        # 取消任务后仍须等待参数恢复，超时使用单调时钟。
        while not future.done():
            if time.monotonic() >= deadline or not rclpy.ok():
                raise TimeoutError('参数服务响应超时；保持恢复状态')
            time.sleep(0.01)
        result = future.result()
        if result is None:
            raise RuntimeError('参数服务没有返回结果')
        return result

    def call(self, node, kind, request):
        client = self.client(node, kind)
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f'{node} 参数服务不可用')
        future = client.call_async(request)
        if kind == 'set':
            self.pending.append(future)
        try:
            return self.wait(future)
        finally:
            if future.done() and future in self.pending:
                self.pending.remove(future)

    def settle(self):
        for future in list(self.pending):
            self.wait(future)
            self.pending.remove(future)

    def get(self, node, names):
        request = GetParameters.Request(names=names)
        values = self.call(node, 'get', request).values
        if len(values) != len(names) or any(v.type == ParameterType.PARAMETER_NOT_SET for v in values):
            raise RuntimeError(f'{node}: 请求了不存在或未声明的动态参数')
        return dict(zip(names, values))

    def set(self, node, values):
        request = SetParametersAtomically.Request(parameters=[Parameter(name=name, value=value) for name, value in values.items()])
        result = self.call(node, 'set', request).result
        if not result.successful:
            raise RuntimeError(f'{node}: {result.reason}')


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
            # 请求超时时远端可能已经赋值，仍需保留原值供恢复。
            if node not in self.touched:
                self.touched.append(node)
            self.backend.set(node, values)

    def restore(self):
        if not self.active:
            return
        # 先等待未完成的赋值请求，避免迟到响应覆盖恢复值。
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
