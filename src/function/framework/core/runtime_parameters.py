"""ROS parameter transport and temporary profiles. Never writes YAML files."""
import time
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from .transaction import ParameterTransaction


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
        # Restoration must continue after emergency_stop, using wall time.
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


class RuntimeParametersMixin:
    def initialize_transactions(self):
        self.parameter_transaction = ParameterTransaction(RosParameterBackend(self))
        self.active_mode = None

    def profile_values(self, key, mode_id=None):
        profile = self.NAV2_SPEED_PROFILES[key]
        values = {'controller_server': dict(profile['controller']), 'velocity_smoother': dict(profile['velocity_smoother'])}
        requested = mode_id if mode_id is not None else self.active_mode
        if requested is None:
            requested = 2 if key == '2_sprint' else key
        base = self.mode_configs[requested]['base_mode']
        # Derived modes retain the original stage algorithm and override its values.
        for mode in dict.fromkeys((base, requested)):
            for node, overrides in self.mode_configs[mode].get('overrides', {}).items():
                values.setdefault(node.strip('/'), {}).update(overrides)
        return {node: {name: self.make_parameter_msg(name, value).value for name, value in params.items()} for node, params in values.items()}

    def begin_temporary_parameters(self, mode):
        mode = self.normalize_nav2_speed_profile_key(mode)
        if mode not in self.mode_configs:
            raise ValueError(f'未定义速度模式 {mode}')
        base = self.mode_configs[mode]['base_mode']
        keys = (2, '2_sprint') if base == 2 else (base,)
        names = {}
        for key in keys:
            for node, values in self.profile_values(key, mode).items():
                names.setdefault(node, set()).update(values)
        self.parameter_transaction.begin(names)
        self.active_mode = mode
        self.current_nav2_speed_profile = None

    def apply_nav2_speed_profile(self, speed_profile):
        key = self.normalize_nav2_speed_profile_key(speed_profile)
        if key not in self.NAV2_SPEED_PROFILES:
            raise ValueError(f'未配置模式 {key}')
        if self.current_nav2_speed_profile == key:
            return True
        # Rejection aborts the task; the dispatcher restores all touched nodes.
        self.parameter_transaction.apply(self.profile_values(key))
        self.current_nav2_speed_profile = key
        self.get_logger().info(f'临时应用速度模式 {self.active_mode}，内部阶段 {key}')
        return True

    def restore_temporary_parameters(self):
        self.parameter_transaction.restore()
        self.current_nav2_speed_profile = None
        self.active_mode = None
