# 功能使用与新增功能

本文路径相对于 `chairman_lidar/`，命令从工程根目录执行。内容按“文件结构 → 功能调用 → 新增点位与模式 → 开发任务”的顺序组织。已有参数的编辑位置和生效步骤见[参数修改说明](parameter_modification.md)。

## 1. 功能文件框架

```text
src/function/
├── framework/                            # ROS 包 framework：进程入口与公共通信
│   ├── sum.py                            # 菜单、状态条、信号处理与退出
│   └── core/
│       ├── communication.py              # 话题、服务、TF 和底盘速度输出
│       ├── actions.py                    # Nav2 目标发送、取消、终态确认
│       └── parameters.py                 # 参数读取、原子设置和恢复校验
├── detail/
│   ├── on_demand/                        # Python 包 chairman_tasks
│   │   ├── task.py                       # TaskServer、dispatch、TaskContext
│   │   ├── config.py                     # 配置加载、名称展开与有效性校验
│   │   ├── fixed_point/task.py           # 三种定点导航流程及参数应用
│   │   └── special/
│   │       ├── motion.py                 # 角度、坐标转换和速度限制计算
│   │       ├── move_forward/task.py      # 定距移动闭环控制
│   │       ├── turn_left/task.py         # 有符号角度的旋转控制
│   │       ├── stair_forward/task.py     # 登阶周期控制与停止信号处理
│   │       ├── align_region/task.py      # 当前区域中心目标计算
│   │       ├── uphill/task.py            # 沿地图 X 方向上坡
│   │       └── <功能名>/task.py          # 功能入口 run(ctx, request)
│   └── continuous/                       # 独立运行的持续功能
│       ├── odometry/                     # ros2 run odometry odometry
│       └── micro_ros/                    # ros2 run micro_ros agent
│           ├── launcher/                 # 启动包构建与安装定义
│           ├── agent.py
│           ├── config/boot.yaml
│           └── src/                      # Agent、消息和 setup 源码包
└── config/on_demand/                      # framework 构建、安装使用的配置
    ├── points.yaml                       # 点位编号、地图位姿与模式
    ├── functions.yaml                    # 特殊功能登记与任务参数
    └── modes/
        ├── speed_1.yaml                  # 基础导航
        ├── speed_2.yaml                  # 冲刺与精调
        ├── speed_3.yaml                  # 提前对齐
        └── special.yaml                  # 手写运动与检测参数
```

日常配置编辑入口为 `src/config/framework/on_demand/`；`sync` 将其同步至上图中的包内配置和已有安装副本。具体功能代码位于 `detail`，`framework/core` 只保留公共 ROS 通信服务。

建议按一次请求的执行顺序阅读代码：

| 顺序 | 入口 | 职责 |
|---|---|---|
| 1 | `framework/sum.py: main()`、`menu_loop()` | 启动 ROS 执行器，读取编号并提交服务请求 |
| 2 | `detail/on_demand/task.py: TaskServer.srv_callback()` | 获取任务锁，记录开始状态，执行任务并统一收尾 |
| 3 | 同文件的 `dispatch()` | 从登记模块导入 `run(ctx, request)`；普通点位使用 `fixed_point.task` |
| 4 | 具体功能的 `run()` | 读取任务参数，执行导航或手写控制，返回布尔结果 |
| 5 | `TaskServer.restore_parameters()` | 确认 Action 结束，恢复临时参数并读回校验 |

`TaskContext` 与调度代码在同一个 `task.py` 中，提供组合任务接口。右移和 KFS 移动复用 `move_forward`；三个旋转入口复用 `turn_left`；三个登阶入口复用 `stair_forward`。修改这些共享控制函数会影响对应的全部调用者。

持续功能通过订阅和定时器独立运行。登阶属于按需任务：一次调用后持续运动，收到停止信号或中止请求时返回结果。

## 2. sum.py 怎么用

### 2.1 正常启动与参数

```bash
source install/setup.bash
ros2 run configuration main_boot
```

总启动选择场地后，将 `SELECTED_POSE` 传给功能、导航及 odometry 进程：1/2 使用红方点位与区域，3/4 使用蓝方。Function sum 不再单独选择场地。

需要在另一个终端独立运行菜单时，先避免总启动重复创建功能进程，再沿用同一场地编号：

```bash
ros2 run configuration main_boot --selected-pose 4 --no-sum
```

另一个终端执行：

```bash
source install/setup.bash
SELECTED_POSE=4 ros2 run framework sum
```

`SELECTED_POSE=4` 是当前命令的环境变量。未设置时，功能进程默认使用场地 1；总启动不会改变其他已打开终端的环境。

| 输入方式 | 说明 |
|---|---|
| 不加命令行参数 | 启动交互菜单 |
| `--ros-args ...` | ROS 标准启动参数，例如 `-p controller_server.FollowPath.v_linear_max:=0.3`，仅覆盖本进程手写控制参数 |
| 菜单输入整数 | 执行该编号对应的任务 |
| 菜单输入 `m` | 显示完整菜单 |
| 偏置提示后输入 `x y z` | 设置本次 KFS 请求的偏置，单位 m |
| 执行中 Ctrl+C | 请求中止当前任务，收尾后返回菜单 |
| 菜单输入 `q` 或 Ctrl+C | 退出功能进程 |

`sum` 不提供 `--selected-pose`、`--call` 等自定义选项。无交互服务入口为 `ros2 run framework preset_nav_node`，它与 `sum` 提供相同服务，两者只运行一个。`main_boot --headless` 使用此服务入口。

### 2.2 启动后显示什么

以下为红方菜单节选，实际条目由 `points.yaml` 和 `functions.yaml` 生成：

```text
chairman_navigation 功能菜单
定点导航：
    0: 原点（模式 3：speed_3）
    1: 矛头1（模式 3：speed_3）
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

一次任务按“开始 → 运行状态 → 最终结果”显示。运行状态在同一行更新，不为每次误差变化单独打印日志。

| 模式 | 状态条中的阶段 |
|---|---|
| 1 | 基础导航 |
| 2 | 冲刺 → 精调；起点在切换距离内时直接精调 |
| 3 | 1/2 提前转正 → 2/2 进入真实目标；关闭提前对齐或满足直接导航分支时显示直接导航 |

模式 3 的显示示意如下；两行状态条代表同一行在不同时刻的内容：

```text
[任务开始] 编号 1：矛头1（模式 3）
[===         ] 模式3 1/2 提前转正【矛头1（提前点）】 | 距提前点 0.300m | 朝向误差 2.1° | 1.2s
[   ===      ] 模式3 2/2 进入真实目标【矛头1】 | 等待 Nav2 执行结果 | 2.0s
[任务完成] 编号 1：矛头1（模式 3） | 结束位置：模式3 2/2 进入真实目标【矛头1】 | 执行成功；动作已结束，已发送零速度，临时参数已恢复 | 用时 …s

chairman_navigation 功能菜单
定点导航：
    ...
特殊功能：
    ...
选择功能编号>
```

状态条最多每秒刷新 5 次，显示阶段、可用误差及耗时，不代表完成百分比。输出重定向到文件或管道时，保留任务开始、最终结果与必要告警，不输出刷新控制字符。整次请求返回后自动重新显示菜单，阶段切换不会重印菜单。

成功、中止或失败均先尝试停止动作和恢复参数，再报告 `[任务完成]`、`[任务中止]` 或 `[任务失败]`。结果中的“结束位置”指执行阶段。若停止或恢复尚未确认，结果会说明原因，后续任务在恢复完成前不能继续执行。

模式 2 在精调参数设置成功后更新阶段。模式 3 第一段达到位置容差后，朝向满足容差或近点调整达到时限均可放行；超时放行不代表朝向已对齐。第二段必须等待第一段 Action 返回终态后才能发送。

选择 -9 或 16 后继续输入：

```text
输入偏置 x y z（单位 m，空格分隔）> 0.2 0.0 0.0
```

KFS 平面运动使用车体坐标系中的 x/y，接口保留 z 字段。登阶等待 `/nav_topic=0`；抬升检测以任务开始后的新高度为基准，等待正向高度增量达到阈值。

### 2.3 服务与速度接口

| 接口 | 类型 | 用途 |
|---|---|---|
| `/set_nav_target` | `custom_msg/srv/SetNavTarget` | 请求包含 `int8 target` 和 `geometry_msgs/Point kfs_offset`；响应包含 `success` 和 `message` |
| `/emergency_stop` | `std_msgs/msg/Empty` | 请求取消当前任务并停止速度转发 |
| `/restore_navigation_parameters` | `std_srvs/srv/Trigger` | 空闲时重试动作结束确认和参数恢复 |
| `/nav_topic` | `std_msgs/msg/Int8` | 登阶任务收到 0 后停止 |
| `/nav_speed_heading_data` | `custom_msg/msg/SpeedHeading` | 底盘输出：`linear_x`、`linear_y` 为 m/s，`angular_z` 为 rad/s |

服务与菜单调用同一调度逻辑。同一时刻只执行一个请求，忙碌时新请求返回失败。Nav2 的 `/cmd_vel` 只在导航转发启用时处理；手写运动接管速度输出，结束时直接发送零速度。

## 3. 增加一个普通固定点

1. 编辑 **`src/config/framework/on_demand/points.yaml`**，在已有 `red:` 下增加未占用的非负编号，例如 34：

```yaml
red:
  34:
    name: 中场等待点
    pose: [1.20, 2.30, 0.0, 1.0]
    mode: 3
```

2. 蓝方也需要此点时，在已有 `blue:` 下填写同编号及蓝方坐标。只登记一方的点，仅在对应方菜单显示。
3. 同步配置并重启 Function sum：

```bash
ros2 run chairman_config sync --files framework/on_demand/points.yaml
```

`pose` 是地图坐标系中的 `[x, y, qz, qw]`，位置单位 m，朝向为平面四元数；`mode` 必须引用已定义模式。编号范围为 0～127，15/16 已用于动态目标。普通点无需新增 Python 文件或登记 `functions.yaml`，由定点导航入口统一处理。

实车采集使用 `ros2 run tool point`，具体采样和保存规则见 [README 的工具说明](../README.md#实车快速标定点位)。

## 4. 增加新的速度模式

已有模式使用 `speed_1.yaml`、`speed_2.yaml`、`speed_3.yaml`。修改已有字段见[内置速度模式](parameter_modification.md#33-改内置速度模式)。增加参数组合时，从完整模板创建中央文件和包内副本：

```bash
cp docs/templates/speed_mode.yaml src/config/framework/on_demand/modes/precise.yaml
```

编辑新文件的 `id`、`name`、`base_mode` 及所需值。以下仅展示需要修改的字段，其余参数保留模板中的完整节点块：

```yaml
id: 4
name: precise
base_mode: 1
parameters:
  controller_server:
    ros__parameters:
      FollowPath:
        translation_kp: 3.5
        v_linear_min: -0.25
        v_linear_max: 0.25
  velocity_smoother:
    ros__parameters:
      max_velocity: [0.25, 0.25, 0.5]
      min_velocity: [-0.25, -0.25, -0.5]
```

编辑完成后创建包内文件：

```bash
cp src/config/framework/on_demand/modes/precise.yaml src/function/config/on_demand/modes/precise.yaml
```

在 `src/config/manifest.json` 中追加映射，注意保留相邻条目间的逗号：

```json
"framework/on_demand/modes/precise.yaml": "src/function/config/on_demand/modes/precise.yaml"
```

模式编号使用未占用的 4 及以上整数。`base_mode` 选择执行流程，三种实现均位于 `detail/on_demand/fixed_point/task.py`：

| base_mode | 流程 | 主要函数 |
|---|---|---|
| 1 | 基础导航 | `navigate()` |
| 2 | 冲刺与精调 | `navigate()`、`initial_profile()`、`update_dynamic_profile()` |
| 3 | 提前对齐后进入目标 | `navigate_pre_align()`、`navigate_alignment()` |

新增模式的 `parameters` 覆盖基础模式同名运行参数，缺省字段继承基础模式。继承模式 2 时，该组覆盖也用于冲刺阶段；需要独立冲刺值时，复制 `speed_2.yaml` 的完整 `sprint_parameters` 并修改。切换距离和提前对齐规则由内置 2/3 号文件配置，新增模式用于组合参数，不定义新的导航算法。

把目标点的 `mode` 改为 4，同步点位，再构建并重启功能进程：

```bash
ros2 run chairman_config sync --files framework/on_demand/points.yaml
colcon build --packages-select framework
source install/setup.bash
```

任务开始前读取将被修改的运行值，结束后恢复并读回确认；磁盘 `nav2_params.yaml` 保留启动基准。标记“启动项”的字段不参与任务期间的动态修改，详见[参数生命周期](parameter_modification.md#34-新模式的临时-nav2-参数)。

## 5. 增加特殊功能

特殊功能在 `functions.yaml` 登记，由 `run(ctx, request)` 执行。任务参数放在登记项的 `parameters` 下，通过 `ctx.config` 读取。

### 5.1 一个完整的任务示例

复制模板并保留 `__init__.py`：

```bash
cp -r docs/templates/on_demand src/function/detail/on_demand/special/my_task
```

编辑 `special/my_task/task.py`：

```python
def run(ctx, request):
    if ctx.cancelled:
        return False
    if not ctx.go_to_point(int(ctx.config['point'])):
        return False
    if not ctx.rotate(float(ctx.config.get('angle', 45.0))):
        return False
    return ctx.wait(float(ctx.config.get('wait_seconds', 0.5)))
```

在 **`src/config/framework/on_demand/functions.yaml`** 的已有 `functions:` 列表末尾追加：

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

同步登记文件，构建并重启 Function sum：

```bash
ros2 run chairman_config sync --files framework/on_demand/functions.yaml
colcon build --packages-select framework
source install/setup.bash
```

`description` 用于菜单，应与任务行为一致。`module` 从 `on_demand/` 下的模块路径开始写，不带 `.py` 后缀。编号必须唯一且在 -128～127 内；普通特殊任务建议使用空闲负数。

需要偏置输入时增加 `uses_offset: true`，由 `request.kfs_offset` 读取。`uses_point: true` 用于关联动态点位，该编号须在红蓝点位中同时登记。普通特殊任务不需要此字段。

入口必须返回 `True` 或 `False`。组合任务检查每一步结果，前一步失败或中止时立即返回；多点序列可参考 `docs/templates/point_sequence/task.py`。常规过程信息使用 `ctx.node.task_progress.update()` 更新状态条，错误原因使用 `fail()` 记录，任务边界由调度层统一输出。

### 5.2 自己写闭环算法

例如复制 `special/turn_left/` 为 `special/my_turn/`，在新文件中实现旋转控制函数，保留入口：

```python
def run(ctx, request):
    angle = float(ctx.config.get('angle_degrees', 90.0))
    return ctx.manual(lambda: rotate(ctx.node, angle))
```

`rotate()` 是本文件中的控制实现；在 `functions.yaml` 登记 `module: special.my_turn.task` 后按上一节构建。新任务无需修改菜单或调度代码。

控制循环应明确坐标系、结束容差与超时，持续检查 `node.cancel_current_task`。`ctx.manual()` 管理手写速度通道及退出清零；`ctx.navigation(mode, callback)` 管理导航片段的参数应用、Action 结束确认与恢复。已有高层接口内部包含这些处理，不应重复嵌套同一动作的生命周期。

组合接口位于 **`src/function/detail/on_demand/task.py`** 的 `TaskContext`：

| 接口 | 用途 |
|---|---|
| `ctx.config` | 读取当前任务的 `parameters` |
| `ctx.cancelled` | 查询中止标记 |
| `ctx.node` | 访问 ROS 节点、位姿缓存和速度发布接口 |
| `ctx.go_to_point(id, mode=None)` | 导航至登记点；省略模式时使用点位模式 |
| `ctx.go_to_pose(x, y, qz, qw, mode=1, name='自定义目标')` | 导航至地图位姿 |
| `ctx.rotate(angle_degrees)` | 原地闭环旋转，角度单位为度 |
| `ctx.move(local_x, local_y, distance)` | 沿起始车体方向定距移动，距离单位 m |
| `ctx.move_offset(offset)` | 根据 KFS 平面偏置闭环移动 |
| `ctx.align_region(mode=1)` | 导航至当前区域中心 |
| `ctx.navigate_offset(offset, mode=1)` | 将车体偏置转换为地图导航目标 |
| `ctx.uphill()` | 使用上坡配置执行运动 |
| `ctx.stair(x, y, speed, name='登阶')` | 沿指定车体方向登阶，等待停止信号 |
| `ctx.wait_lift()` | 等待正向高度增量达到阈值 |
| `ctx.wait(seconds)` | 可响应中止的等待 |
| `ctx.log(text)` | 输出低频信息日志；连续进度使用状态条 |

普通移动使用 `/Odometry` 的位置与航向；区域、上坡和导航目标使用 TF 的 `map → base_footprint`；抬升检测使用 `/odom_map.z`。读取或发布数据时，应保持这些坐标系和单位一致。

每个导航片段返回前均确认动作停止并恢复参数，下一步只在返回成功后执行。恢复异常应交由调度层处理，不能捕获后继续运动。

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
colcon build --packages-select chairman_status_example
source install/setup.bash
ros2 run chairman_status_example status_node
```

5. 若希望随总启动一起运行，在 **`configuration/main_boot.py`** 的 `build_commands()` 中 `return commands` 之前加入：

```python
commands.append(('Status', common + 'exec ros2 run chairman_status_example status_node'))
```

该命令会继承总启动选定的 `SELECTED_POSE` 和工作空间环境。重新构建 `configuration` 后再启动。`src/function/` 本身不要增加 `package.xml`，避免挡住下层独立包的发现。

已有持续功能为 `ros2 run odometry odometry`、`ros2 run micro_ros agent`，默认总启动已运行它们，不要重复启动。
