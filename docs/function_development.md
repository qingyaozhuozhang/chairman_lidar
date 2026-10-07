# 功能使用与新增功能

所有路径相对于 `chairman_navigation/`；命令从工程根目录执行。本文说明功能结构、`sum` 的实际交互，以及新增固定点、速度模式、特殊功能、持续功能的步骤。已有参数如何改见 [parameter_modification.md](parameter_modification.md)。

## 1. 功能文件框架

```text
src/function/
├── config/on_demand/
│   ├── points.yaml                         # 固定点：编号、名称、坐标、模式
│   ├── functions.yaml                      # 特殊任务：编号、说明、模块、参数
│   └── modes/
│       ├── base.yaml                       # 模式 1
│       ├── dynamic.yaml                    # 模式 2
│       ├── pre_align.yaml                  # 模式 3
│       └── <新模式>.yaml
├── detail/
│   ├── continuous/                         # 启动后持续工作：ROS 包或功能包集合
│   │   ├── odometry/                       # ros2 run odometry odometry
│   │   └── micro_ros/                      # ros2 run micro_ros agent
│   │       ├── launcher/                  # ROS 包 micro_ros 的构建文件
│   │       ├── agent.py
│   │       ├── config/boot.yaml
│   │       └── src/                       # Agent、消息和 setup 包，统一构建
│   └── on_demand/                          # 收到一次请求执行一次
│       ├── fixed_point/
│       │   ├── task.py                     # 实际定点导航逻辑
│       │   └── modes/
│       │       ├── dynamic.py              # 按距离切换快慢模式
│       │       └── pre_align.py            # 提前对齐再导航
│       └── special/
│           ├── turn_left/task.py           # 左转的实际控制逻辑
│           ├── uphill/task.py              # 上坡的实际控制逻辑
│           ├── align_region/task.py        # 区域中心目标计算
│           ├── ...
│           └── <新功能>/
│               ├── __init__.py             # 空文件，使目录成为 Python 模块
│               └── task.py                 # 统一入口 run(ctx, request)
└── framework/                              # ROS 包 framework
    ├── sum.py                              # 菜单显示、读取功能编号、等待结果
    └── core/
        ├── runtime.py                      # ROS 服务、任务运行、取消、退出清理
        ├── task_context.py                 # ctx 提供给任务的调用接口
        ├── registry.py                     # 按 functions.yaml 导入任务
        └── ...                             # 共享通信、参数恢复、数学工具
```

`on_demand` 的 `run(ctx, request)` 必须返回 `True` 或 `False`，分别表示成功或失败/中止。具体算法写在本功能的 `task.py` 或同目录辅助文件，不放回 `framework`。

`continuous` 的功能通过订阅和定时器持续运行，用自己的 `ros2 run` 命令启动，没有 `continuous.py` 统一出口。登阶虽然持续输出速度，但仍属于按需任务：选择一次，等待 `/nav_topic=0` 或中止后返回。

现有三个登阶任务的方向、启动条件、完成条件在各自 `task.py`；共用纠偏控制在 `src/function/detail/on_demand/special/stair_forward/controller.py`。修改共用控制会影响三个登阶任务。

## 2. sum.py 怎么用

### 2.1 正常启动与参数

```bash
source install/setup.bash
ros2 run configuration main_boot
```

在总启动终端选场地后，`main_boot` 调用 `fishbot_navigation2/navigation2.launch.py` 并另开终端运行 `sum`。场地通过 `SELECTED_POSE` 传入：1/2 用红方点位与区域，3/4 用蓝方点位与区域。`sum` 不再询问场地。

```bash
# 不启动 sum，也不启动替代功能服务
ros2 run configuration main_boot --selected-pose 4 --no-sum
```

单独调试或上述跳过菜单后，可在另一个终端启动：

```bash
source install/setup.bash
SELECTED_POSE=4 ros2 run framework sum
```

`SELECTED_POSE=4` 是临时环境变量，不是 `sum` 的后置参数；它应与正在运行的导航和 odometry 使用同一个编号。若当前终端已设置该环境变量，只需 `ros2 run framework sum`。未设置时缺省 1；另一个终端不会自动继承总启动的子进程环境。

| sum 的输入方式 | 说明 |
|---|---|
| 命令行不加参数 | 正常菜单用法 |
| 自定义后置参数 | 没有；不使用 `--selected-pose`、`--call`、`--offset` |
| `--ros-args ...` | ROS2 标准调试参数，例如 `-p max_vel_linear:=0.3`；不用于选择场地或功能 |
| 菜单输入整数 | 执行该编号功能 |
| 菜单输入 `m` | 重新显示完整菜单 |
| 偏置提示后输入 `x y z` | 本次 KFS 功能的偏置，单位米 |
| 菜单输入 `q` | 退出 |

没有 `sum --list/--check/--serve`。需要无菜单服务时，单独使用 `ros2 run framework preset_nav_node`；它与 `sum` 提供同一个服务，二者只运行一个。

### 2.2 启动后显示什么

红方、默认配置下的显示节选如下，实际会列出全部点位及特殊功能：

```text
chairman_navigation 功能菜单
定点导航：
    0: 原点（模式 3：pre_align）
    1: 矛头1（模式 3：pre_align）
    2: 矛头2（模式 3：pre_align）
    ...
特殊功能：
   -1: 旋转180°
   -2: 世界X方向上坡
   -3: 右移0.2m
   -4: 前移1.2m
   -5: 旋转-90°
   -6: 旋转+90°
   -7: 前进0.3m/s等待停止信号
   -8: 后退0.3m/s等待停止信号
   -9: KFS偏置闭环移动
  -10: 左移0.1m/s等待停止信号
  -11: 等待抬升高度变化
   15: 对准当前梅林区域中心
   16: KFS偏置导航
输入功能编号；执行时 Ctrl+C 中止当前任务；输入 m 重看菜单；菜单中 q 或 Ctrl+C 退出。
选择功能编号>
```

选择编号后，`simple_nav_node` 的日志会明确记录任务与阶段的边界，三种模式统一使用 `[任务开始]`、`[阶段开始]`、`[阶段结束]` 和最终结果提示：

| 模式 | 阶段提示 |
|---|---|
| 1 | `模式1 基础导航【目标名称】` |
| 2 | `模式2 冲刺【目标名称】` → `模式2 精调【目标名称】`；起点已在切换距离内时直接显示精调 |
| 3 | `模式3 1/2 提前转正【目标名称（提前点/原地）】` → `模式3 2/2 进入真实目标【目标名称】`；未启用提前转正或走直接导航分支时显示 `模式3 直接导航` |

模式 2 只有在精调参数成功生效后才报告阶段切换；参数更新失败会报告失败及所在阶段。模式 3 的示例（省略 ROS 时间戳）：

```text
[任务开始] 编号 1：矛头1（模式 3）
[阶段开始] 模式3 1/2 提前转正【矛头1（提前点）】 | 准备参数，等待 Nav2 接受目标
[===         ] 模式3 1/2 提前转正【矛头1（提前点）】 | 距提前点 0.300m | 朝向误差 2.1° | 1.2s
[阶段结束] 模式3 1/2 提前转正【矛头1（提前点）】 | 达到放行条件：……；第一段动作已结束
[阶段开始] 模式3 2/2 进入真实目标【矛头1】 | 准备参数，等待 Nav2 接受目标
[阶段结束] 模式3 2/2 进入真实目标【矛头1】 | Nav2 确认已到达目标
[任务完成] 编号 1：矛头1（模式 3） | 结束位置：模式3 2/2 进入真实目标【矛头1】 | 执行成功；动作已结束，已发送零速度，临时参数已恢复 | 用时 …s

chairman_navigation 功能菜单
定点导航：
    ...
特殊功能：
    ...
选择功能编号>
```

运行中的流动状态条每秒最多刷新 5 次，在同一行显示当前阶段、距离/朝向误差和耗时；它表示任务仍在执行，不是完成百分比。终端较窄时会截断显示，避免自动换行。重定向到文件或管道时只保留阶段与结果日志，不写入刷新控制字符。

只有动作结束且临时参数恢复后才报告整次任务结果：`[任务完成]`、`[任务中止]` 或 `[任务失败]`，并标出结束阶段。Nav2 状态码 6 显示为 `ABORTED：Nav2 执行失败`；停止/恢复未确认会单独说明，不能当作正常结束。模式 3 到近点调整时限后放行会明确写“按配置放行”和剩余角度，不声称转正完成。第一段的 `[阶段结束]` 不代表整次任务终止。

整次任务完成、中止或失败后，都会自动重新打印完整菜单，等待下一次编号；执行中的阶段切换不会重印菜单。空闲时也可输入 `m` 重看。框架仍等本次请求返回后才接受下一项，避免同时执行两个运动任务。底层速度转发、零速度发送等细节保留为 DEBUG 日志。

选择 -9 或 16 后，还会出现：

```text
输入偏置 x y z（单位 m，空格分隔）> 0.2 0.0 0.0
```

当前 KFS 平面移动/导航算法使用 x/y，接口保留 z 字段。输入格式错误会返回菜单；这不是命令行 `--offset` 参数。

执行中 Ctrl+C 请求中止，等动作停止、临时参数恢复后返回菜单。菜单空闲时 Ctrl+C 退出。登阶等待 `/nav_topic=0`，抬升检测等待达到高度阈值；这些任务在结束条件满足前不会自动返回。

## 3. 增加一个普通固定点

坐标格式、红蓝方选择及 `mode` 字段的填写方式，参考[参数配置文档：点位参数](parameter_modification.md#31-改点位坐标或点位所用模式)。

1. 打开 **`src/function/config/on_demand/points.yaml`**。
2. 在已有 `red:` 下增加一个未使用的非负编号，例如 33：

```yaml
  33:
    name: 新取料点
    pose: [1.20, 2.30, 0.0, 1.0]
    mode: 1
```

3. 蓝方也需要该点时，在已有 `blue:` 下增加同编号和蓝方坐标。只添加一方时只在该方菜单显示。
4. 按[参数配置文档第 3 节的生效步骤](parameter_modification.md#3-按需功能参数)构建 `framework` 并重启 `sum`，菜单自动出现 33，输入 `33` 即可运行。

`pose` 顺序是地图系 `[x,y,qz,qw]`；可用 `ros2 run tool get_pose` 获取。`mode` 必须是已有模式 id。普通点不需要新增 Python 文件或登记 `functions.yaml`，复用 `fixed_point/task.py`。

编号须在 0～127 且未占用；15/16 已被动态目标任务使用。不要重复创建另一个 `red:` 或 `blue:` 顶层键。

## 4. 增加新的速度模式

调节已有模式时参考[参数配置文档：内置速度模式](parameter_modification.md#33-改内置速度模式)；新增模式的 `base_mode`、`overrides` 及临时参数恢复规则参考[参数配置文档：新模式参数](parameter_modification.md#34-新模式的临时-nav2-参数)。

1. 从模板复制：

```bash
cp docs/templates/speed_mode.yaml src/function/config/on_demand/modes/precise.yaml
```

2. 编辑新文件 **`src/function/config/on_demand/modes/precise.yaml`**，例如：

```yaml
id: 4
name: 精确低速
base_mode: 1
overrides:
  controller_server:
    FollowPath.v_linear_max: 0.25
    FollowPath.v_linear_min: -0.25
    FollowPath.translation_kp: 3.5
  velocity_smoother:
    max_velocity: [0.25, 0.25, 0.5]
    min_velocity: [-0.25, -0.25, -0.5]
```

3. 在 **`src/function/config/on_demand/points.yaml`** 把需要使用新模式的点设为 `mode: 4`。
4. 构建并重启 `framework`；选择该点时，会自动应用新参数。

`id` 唯一，新增模式使用 4 以上编号。`base_mode` 决定继承哪一种已有流程：

| base_mode | 流程 | 实现路径（相对于 `src/function/detail/on_demand/`） |
|---|---|---|
| 1 | 基础定点流程（base） | `fixed_point/task.py` |
| 2 | 距离远时快档、距离近时细调 | `fixed_point/modes/dynamic.py` |
| 3 | 提前对齐朝向，再到实际目标 | `fixed_point/modes/pre_align.py` |

新模式通过 `overrides` 调节节点现有且支持动态修改的参数，不另建 `simple_nav_node` 参数表。内置 1/2/3 的 `simple_nav_node` 保留在各自 YAML。模式 2 阶段切换时也会继续应用新模式的 `overrides`。

任务开始保存当前运行值，完成、异常或取消后恢复并读回确认；磁盘 `nav2_params.yaml` 不被修改。模式文件增加的是参数组合；新的运动流程通过下一节的任务代码实现。

## 5. 增加特殊功能

在 `functions.yaml` 中填写 `parameters` 时，参考[参数配置文档：特殊任务参数](parameter_modification.md#32-改某个特殊任务的运行参数)。任务代码通过 `ctx.config` 读取这些字段。

### 5.1 一个完整的任务示例

复制统一模板：

```bash
cp -r docs/templates/on_demand src/function/detail/on_demand/special/my_task
```

保留空的 `__init__.py`，编辑 **`src/function/detail/on_demand/special/my_task/task.py`**：

```python
def run(ctx, request):
    ctx.log('开始：去目标点、旋转、等待')
    if ctx.cancelled:
        return False
    if not ctx.go_to_point(int(ctx.config['point'])):
        return False
    if not ctx.rotate(float(ctx.config.get('angle', 45.0))):
        return False
    return ctx.wait(float(ctx.config.get('wait_seconds', 0.5)))
```

在 **`src/function/config/on_demand/functions.yaml`** 已有 `functions:` 列表末尾追加：

```yaml
- id: -12
  name: my_task
  description: 去1号点、旋转45度、等待
  module: special.my_task.task
  parameters:
    point: 1
    angle: 45.0
    wait_seconds: 0.5
```

构建 `framework` 并重启 `sum`，菜单出现 -12，输入该编号执行。`description` 是菜单里的小说明，参数改变后应同步更新说明；`module` 不写 `.py`，从 `on_demand/` 下开始写模块路径。

任务编号必须唯一且在 -128～127；建议新特殊任务使用空闲负数。示例 -12 若已被占用，换一个编号。需要 x/y/z 偏置提示时，加 `uses_offset: true` 并在任务内读取 `request.kfs_offset`。`uses_point: true` 是已有 15/16 动态目标关联点位的设置，普通特殊任务不需要。

### 5.2 自己写闭环算法

例如新增另一种转向控制，复制 `src/function/detail/on_demand/special/turn_left/` 到 `special/my_turn/`，修改新目录 `task.py` 中的实际 `rotate(node, target_angle_degrees)` 控制循环，保持入口：

```python
def run(ctx, request):
    angle = float(ctx.config.get('angle_degrees', 90.0))
    return ctx.manual(lambda: rotate(ctx.node, angle))
```

这里的 `rotate` 必须是本任务文件内真正实现的函数；所复制的文件已有完整闭环逻辑。然后在 `functions.yaml` 登记 `module: special.my_turn.task`，按前述流程构建。新增功能不修改 `sum.py` 或 `registry.py`。

自行编写控制循环时读取 `ctx.node` 的里程计，检查 `node.cancel_current_task`，定义结束条件和超时，成功/失败返回布尔值。`ctx.manual()` 负责速度通道切换及退出清零；导航片段用 `ctx.navigation(mode, callback)` 负责参数生命周期。已有 `ctx.go_to_point/rotate/move` 内部已包装对应生命周期，不要再嵌套包装同一个动作。

常用接口位于 **`src/function/framework/core/task_context.py`**：

| 接口 | 用途 |
|---|---|
| `ctx.config` | 读取本任务 `parameters` |
| `ctx.cancelled` | 是否收到中止 |
| `ctx.node` | ROS 节点、里程计、共用数学和发布接口 |
| `ctx.go_to_point(id, mode=None)` | 等待一个登记点的导航完成 |
| `ctx.go_to_pose(x,y,qz,qw,mode=1)` | 直接给定地图目标 |
| `ctx.rotate(angle_degrees)` | 闭环旋转，度 |
| `ctx.move(local_x,local_y,distance)` | 沿起始车体方向纠偏移动，米 |
| `ctx.move_offset(request.kfs_offset)` | 使用偏置做闭环移动 |
| `ctx.wait(seconds)` | 可响应中止的等待 |
| `ctx.log(text)` | 打印日志 |

组合接口 `ctx.rotate()` 当前复用 `turn_left/task.py` 的旋转实现，`ctx.move()` 复用 `move_forward/task.py`。现有编号功能各自保留自己的实现；修改其他同类任务文件不会自动修改组合接口的行为。

## 6. 增加持续调用功能

每个新功能在 `src/function/detail/continuous/` 下建立独立 ROS 包，用该包自己的命令启动，不在 `functions.yaml` 登记，不会出现在按需菜单。

1. 复制可运行的定时器节点模板：

```bash
cp -r docs/templates/continuous/chairman_status_example src/function/detail/continuous/
```

2. 编辑 **`src/function/detail/continuous/chairman_status_example/status_node.py`**，在里面写订阅、定时器等处理逻辑。模板每秒输出一次状态。需要改包名时同步修改本目录 `package.xml`、`CMakeLists.txt`。
3. 若需要自己的参数，在该包下新建 `config/`，由 CMake 安装并由节点/launch 显式读取。micro-ROS 配置的具体路径、字段和生效步骤见[参数配置文档：micro-ROS 参数](parameter_modification.md#4-micro-ros-参数)；odometry 的初始位姿与区域设置见[参数配置文档：odometry 参数](parameter_modification.md#6-odometry-参数)。单纯创建 YAML 不会自动加载。
4. 构建并启动：

```bash
colcon build --symlink-install --base-paths src configuration tool --packages-select chairman_status_example
source install/setup.bash
ros2 run chairman_status_example status_node
```

5. 若希望随总启动一起运行，在 **`configuration/main_boot.py`** 的 `build_commands()` 中 `return commands` 之前加入：

```python
commands.append(('Status', common + 'exec ros2 run chairman_status_example status_node'))
```

该命令会继承总启动选定的 `SELECTED_POSE` 和工作空间环境。重新构建 `configuration` 后再启动。`src/function/` 本身不要增加 `package.xml`，避免挡住下层独立包的发现。

已有持续功能为 `ros2 run odometry odometry`、`ros2 run micro_ros agent`，默认总启动已运行它们，不要重复启动。
