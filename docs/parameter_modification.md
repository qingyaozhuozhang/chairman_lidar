# 参数文件与配置修改

本文路径相对于 `chairman_lidar/`，执行命令前先加载 `source install/setup.bash`。示例说明参数位置与修改方法，实际数值应依据地图、设备标定和控制要求设置。YAML 片段合并到已有节点或条目中，不重复创建同名键。

## 1. 参数部分的文件框架

```text
src/config/                              # 中央编辑入口，使用 sync 同步
├── sync.py
├── manifest.json                        # 中央文件到源码目标的映射
├── framework/on_demand/
│   ├── points.yaml                      # 红蓝点位、名称与模式
│   ├── functions.yaml                   # 特殊功能登记及任务参数
│   └── modes/
│       ├── speed_1.yaml                 # 基础导航
│       ├── speed_2.yaml                 # 冲刺与精调
│       ├── speed_3.yaml                 # 提前对齐
│       └── special.yaml                 # 手写运动和检测参数
├── fast_lio/                            # mid360.yaml 及其他雷达配置
├── fishbot_navigation2/
│   ├── nav2_params.yaml                 # Nav2 启动基准
│   └── rviz/*.rviz
├── livox_ros_driver2/                    # MID360 网络配置及其他型号配置
└── odometry/
    ├── initial_poses.yaml               # 四个场地的初始位姿
    └── regions.yaml                     # 红蓝区域边界
src/function/
├── config/on_demand/                    # 中央功能配置的包内副本
└── detail/continuous/
    ├── micro_ros/config/boot.yaml        # 串口及 Agent 环境，本包直接管理
    └── odometry/config/                 # 中央 odometry 配置的包内副本
src/fishbot/fishbot_navigation2/
├── config/nav2_params.yaml              # 中央 Nav2 配置的包内副本
├── launch/navigation2.launch.py         # 节点关系、TF、地图及扫描设置
├── maps/room.yaml                       # 二维地图描述
└── PCD/scans.pcd                        # 当前 launch 引用的点云地图
configuration/main_boot.py               # 场地选择与启动编排
```

配置按三个层次生效：中央文件用于编辑和同步；包内副本用于构建安装；节点启动时读取当前工作空间的安装配置。任务期间的运行参数还可由模式切换临时覆盖。

| 类型 | 编辑入口 | 生效方式 |
|---|---|---|
| 雷达、FAST-LIO、Nav2、odometry | `src/config/` 下对应目录 | `sync --files ...`，重启对应节点 |
| 点位、功能登记、已有模式及手写控制 | `src/config/framework/on_demand/` | 同步相应文件，重启 Function sum |
| 新增模式文件或功能 Python 代码 | 中央/包内模式文件、`detail/on_demand/` | 登记映射或模块，构建 `framework` 并重启 |
| micro-ROS 启动参数 | `src/function/detail/continuous/micro_ros/config/boot.yaml` | 构建 `micro_ros`，重启 Agent |
| 导航 launch、地图、TF、扫描设置 | `src/fishbot/fishbot_navigation2/` | 构建 `fishbot_navigation2`，重启导航 |
| 场地编号 | `main_boot` 交互或 `--selected-pose` | 通过 `SELECTED_POSE` 传给本次子进程 |

直接修改包内配置时，应同步保留中央文件中的同一修改，再构建对应包。否则后续 `sync` 会使用中央内容覆盖包内文件。普通增量构建不会自动将中央文件同步到其他包。

## 2. sync.py 的使用说明

### 2.1 全部覆盖

```bash
ros2 run chairman_config sync --all
```

同步 `src/config/manifest.json` 登记的全部配置，包括点位、功能和模式。未登记的新文件不会被自动扫描加入。

### 2.2 指定文件覆盖

```bash
ros2 run chairman_config sync --files fast_lio/mid360.yaml
ros2 run chairman_config sync --files framework/on_demand/points.yaml framework/on_demand/modes/speed_2.yaml
ros2 run chairman_config sync --files fishbot_navigation2/nav2_params.yaml odometry/regions.yaml
```

`--files` 后填写相对于 `src/config/` 的名称，多个文件用空格分隔。

| 参数 | 含义 |
|---|---|
| `--all` | 同步全部已登记文件 |
| `--files 文件1 [文件2 ...]` | 同步指定文件 |
| `-h` / `--help` | 显示帮助 |

`--all` 与 `--files` 必须二选一。工具检查映射和路径范围，原样复制文件，不校验参数语义，也不提供预览步骤。修改前的文件保存在 `.configuration_backups/`；写入失败时恢复已写入的目标。未知配置名、缺失目标文件或越过工程边界的路径会报错。

### 2.3 实际覆盖到哪里

| `--files` 名称 | 源码目标 |
|---|---|
| `fast_lio/mid360.yaml` | `src/fishbot/fast_lio/config/mid360.yaml` |
| `fishbot_navigation2/nav2_params.yaml` | `src/fishbot/fishbot_navigation2/config/nav2_params.yaml` |
| `livox_ros_driver2/MID360_config.json` | `src/fishbot/livox_ros_driver2/config/MID360_config.json` |
| `odometry/initial_poses.yaml` | `src/function/detail/continuous/odometry/config/initial_poses.yaml` |
| `odometry/regions.yaml` | `src/function/detail/continuous/odometry/config/regions.yaml` |
| `framework/on_demand/points.yaml` | `src/function/config/on_demand/points.yaml` |
| `framework/on_demand/functions.yaml` | `src/function/config/on_demand/functions.yaml` |
| `framework/on_demand/modes/speed_3.yaml` | `src/function/config/on_demand/modes/speed_3.yaml` |
| `framework/on_demand/modes/special.yaml` | `src/function/config/on_demand/modes/special.yaml` |

其他模式、雷达和 RViz 配置以清单为准。同步同时更新本工程已有的安装副本，兼容独立安装与合并安装。运行中的节点不会自动重新读取文件；同步后须重启相关进程。

## 3. 按需功能参数

点位、任务、模式和手写控制的中央文件均可同步。以下示例更新模式 3：

```bash
ros2 run chairman_config sync --files framework/on_demand/modes/speed_3.yaml
```

停止旧 Function sum，再由总启动或同场地终端重新启动。已有配置仅修改数值时无需重编译；新增 Python 模块或模式文件后执行：

```bash
colcon build --packages-select framework
source install/setup.bash
```

### 3.1 改点位坐标或点位所用模式

文件：**`src/config/framework/on_demand/points.yaml`**。

例如修改已有红方 1 号点（合并到 `red` 下）：

```yaml
red:
  1:
    name: 新的取料位置
    pose: [1.20, 2.30, 0.0, 1.0]
    mode: 1
```

`pose` 为地图系 `[x, y, qz, qw]`；x/y 单位米，qz/qw 为朝向四元数。`mode` 对应 `modes/*.yaml` 的 `id`。红方使用 `red`，蓝方使用 `blue`；两方坐标分别填写。不要修改 15/16 的空坐标来代替它们的动态目标算法。

### 3.2 改某个特殊任务的运行参数

文件：**`src/config/framework/on_demand/functions.yaml`**。

修改已有 -6 项的 `parameters`，让该任务转 45°；同时改说明以匹配实际行为：

```yaml
- id: -6
  name: turn_left
  description: 左转45°
  module: special.turn_left.task
  parameters:
    angle_degrees: 45.0
```

实现位于 `src/function/detail/on_demand/special/turn_left/task.py`，通过 `ctx.config.get('angle_degrees', 90.0)` 读取。

其他已经支持的字段：

| 任务 | `parameters` 字段示例 | 实现文件 |
|---|---|---|
| -1/-5/-6 旋转 | `angle_degrees: 45.0`（右转用负值） | `special/turn_180/task.py`、`special/turn_right/task.py`、`special/turn_left/task.py` |
| -3/-4 定距移动 | `distance: 0.40`，单位米 | `special/shift_right/task.py`、`special/move_forward/task.py` |
| -2 上坡 | `distance: 2.0`，单位米 | `special/uphill/task.py` |
| -7/-8/-10 登阶 | `speed: 0.20`，单位 m/s | `special/stair_forward/task.py`、`special/stair_backward/task.py`、`special/stair_left/task.py` |
| 15/16 动态目标导航 | `mode: 1` | `special/align_region/task.py`、`special/kfs_navigation/task.py` |

表中实现路径均以 `src/function/detail/on_demand/` 开头。`parameters` 只会影响任务代码实际读取的字段；添加一个无人读取的键不会自动改变控制行为。-9/16 的 x/y/z 偏置是本次调用输入，来源为菜单或 `request.kfs_offset`，不是 `parameters` 中任意命名的字段。

15/16 的动态目标入口采用单段导航；`mode` 选择参数组，模式 2 仍按距离切换冲刺与精调。模式 3 的两段提前对齐由普通定点及 `ctx.go_to_pose()` 流程调用，15/16 使用其参数时不执行提前点流程。

### 3.3 改内置速度模式

| 模式 | 中央文件 | 流程 |
|---|---|---|
| 1 | `src/config/framework/on_demand/modes/speed_1.yaml` | 基础导航 |
| 2 | `src/config/framework/on_demand/modes/speed_2.yaml` | 远处冲刺、近点精调 |
| 3 | `src/config/framework/on_demand/modes/speed_3.yaml` | 提前对齐、进入真实目标 |

三份文件均完整列出 `controller_server` 与 `velocity_smoother`，名称、节点层级及数组格式与 `nav2_params.yaml` 一致。`id/name/base_mode` 用于功能侧选择模式；`parameters` 下才是对应 Nav2 节点的参数树。

例如把模式 1 的线速度上限改为 0.4 m/s，应在现有 `FollowPath` 块调整 `v_linear_min: -0.4`、`v_linear_max: 0.4`，同时修改该文件的完整平滑器块：

```yaml
parameters:
  velocity_smoother:
    ros__parameters:
      use_sim_time: True  # 启动项
      smoothing_frequency: 20.0
      scale_velocities: False  # 启动项
      feedback: "OPEN_LOOP"
      max_velocity: [0.4, 0.4, 1.0]
      min_velocity: [-0.4, -0.4, -1.0]
      max_accel: [0.5, 0.5, 1.0]
      max_decel: [-0.5, -0.5, -1.0]
      odom_topic: "odom"
      odom_duration: 0.1
      deadband_velocity: [0.0, 0.0, 0.0]
      velocity_timeout: 1.0
```

此示例位于已有 `parameters` 中，与完整 `controller_server` 块并列。平滑器向量顺序为 `[x, y, yaw]`：速度单位为 m/s、m/s、rad/s；加速度单位为 m/s²、m/s²、rad/s²。`max_decel` 使用负值。

常用调节项如下，插件字段均位于 `controller_server.ros__parameters`：

| 字段 | 含义 |
|---|---|
| `FollowPath.translation_kp/ki/kd` | 平移 PID 增益 |
| `FollowPath.rotation_kp/ki/kd` | 旋转 PID 增益 |
| `FollowPath.v_linear_min/max` | 控制器线速度范围 |
| `FollowPath.v_angular_min/max` | 控制器角速度范围 |
| `general_goal_checker.xy_goal_tolerance` | 到点位置容差，m |
| `general_goal_checker.yaw_goal_tolerance` | 到点航向容差，rad |
| `velocity_smoother` 的 `max_velocity/min_velocity` | 平滑器各轴速度限制 |
| `velocity_smoother` 的 `max_accel/max_decel` | 平滑器各轴加速与制动限制 |

**模式 2：两组参数分别配置。** `sprint_parameters` 用于远处冲刺，`parameters` 用于近点精调，两组控制同一批参数，因此字段名称相同。

| 配置项 | 冲刺组 | 精调组 |
|---|---|---|
| `FollowPath.translation_kp` | 10.0 | 4.0 |
| `FollowPath.v_linear_max` | 3.0 | 1.5 |
| `velocity_smoother.max_velocity` | [3.0, 3.0, 2.5] | [1.5, 1.5, 1.5] |

顶层 `switch_distance` 默认为 1.5 m，`switch_check_period` 默认为 0.05 s。起点距离大于阈值时冲刺，进入阈值后精调；起点已在阈值内则直接精调。切换后不再返回冲刺，参数设置成功后才更新显示阶段。

**模式 3：提前对齐条件单独配置。** 以下字段位于顶层 `pre_align`：

```yaml
pre_align:
  enabled: true
  distance: 0.5
  release_xy_tolerance: 0.1
  release_yaw_tolerance: 0.15
  near_adjust_timeout: 0.5
```

`distance` 是提前点与真实目标间的距离，单位 m。当前位置已在该距离内时先原地调整朝向，否则先导航至目标连线上的提前点。第一段使用最终目标朝向；位置进入 `release_xy_tolerance` 后，航向满足 `release_yaw_tolerance`（rad）或近点调整达到 `near_adjust_timeout`（s）时放行。第一段 Action 终止确认完成后发送第二段，超时放行不等于已满足航向容差。

关闭 `enabled`、距离阈值接近零或无法获取初始地图位姿时，流程直接导航至真实目标。

### 3.4 新模式的临时 Nav2 参数

新增模式使用 `docs/templates/speed_mode.yaml` 的完整节点块，设置唯一 `id`、显示用 `name` 和 `base_mode: 1/2/3`。`parameters` 覆盖基础模式同名运行参数，缺省项继承基础模式；模式 2 可额外提供 `sprint_parameters`。创建文件、登记同步映射和点位引用的步骤见[新增速度模式](function_development.md#4-增加新的速度模式)。

模式切换只更新运行期间可修改的参数：

| 参数类别 | 处理方式 |
|---|---|
| `use_sim_time`、插件类型 `.plugin` | 启动项，不在任务中修改 |
| `controller_server` 的节点级字段，如 `controller_frequency`、插件列表 | 启动项，控制器运行时不下发 |
| `velocity_smoother.scale_velocities` | 启动项，不参与动态切换 |
| 控制器插件、目标/进度检查器及平滑器的其他已列字段 | 通过参数服务设置，结束后恢复 |

标为 `# 启动项` 的字段保留在模式文件中用于完整对照，应在中央 `nav2_params.yaml` 修改后同步并重启导航。实际时钟由 launch 的 `use_sim_time` 参数控制。其他字段也需远端节点支持对应类型和动态更新，否则任务会报告失败。

一次导航片段按以下顺序处理参数：

1. 读取本次将修改的运行参数并保存原值。
2. 设置所选模式；模式 2 的后续阶段继续使用同一份原值记录。
3. 目标完成、失败或取消后，确认 Nav2 动作结束。
4. 恢复原值并读回核对，成功后允许组合任务继续下一步。

参数服务对单个节点使用原子设置，多个节点仍需顺序调用。磁盘 `nav2_params.yaml` 始终保存启动基准；恢复目标是执行前读取的运行值，不是重新读取磁盘文件。

若停止或恢复尚未确认，后续任务被阻止。任务空闲且相关服务可用后，可重试恢复：

```bash
ros2 service call /restore_navigation_parameters std_srvs/srv/Trigger '{}'
```

### 3.5 手写运动与检测参数

`modes/special.yaml` 与速度文件使用相同的 `节点名 → ros__parameters → 插件名 → 参数名` 层级。`parameters` 用于普通移动、旋转和登阶，`uphill_parameters` 用于沿地图 X 方向上坡。每组完整列出 `controller_server` 和 `velocity_smoother`，并用 `simple_nav_node` 块配置本功能特有的条件。

这些配置由 `simple_nav_node` 的手写控制读取，不向 Nav2 下发。标记 `# 对照项` 的字段只保留完整 Nav2 结构，本手写控制器不读取；其他字段实际参与控制。不要把对照项当作手写控制的调节入口。

| 配置位置（省略 `ros__parameters`） | 用途 |
|---|---|
| `controller_server.FollowPath.translation_kp` | 直线位置误差的比例增益 |
| `controller_server.FollowPath.rotation_kp` | 普通组原地旋转的比例增益 |
| `controller_server.FollowPath.v_linear_max`、`v_angular_max` | 线速度与角速度幅值上限 |
| `controller_server.FollowPath.min_approach_linear_velocity`、`min_approach_angular_velocity` | 接近目标时的最小速度幅值 |
| `controller_server.general_goal_checker.xy_goal_tolerance`、`yaw_goal_tolerance` | 位置与航向结束容差 |
| `velocity_smoother.max_accel`、`max_decel` | 手写控制的加速与制动限制，顺序为 `[x, y, yaw]` |
| 普通组 `controller_server.controller_frequency` | 阻塞式手写控制循环的周期来源，默认 50 Hz；登阶定时器由通信节点独立按 50 Hz 调度 |
| `simple_nav_node.yaw_correction` | 移动时保持起始朝向；`rotation_kp` 为增益，`max_velocity_ratio` 为角速度上限比例 |
| `simple_nav_node.cross_track` | 横向纠偏，使用 `translation_kp`、`v_linear_max`、`min_approach_linear_velocity`、`xy_goal_tolerance` |
| 普通组 `simple_nav_node.lift_check_height` | 抬升检测高度阈值 |
| 上坡组 `simple_nav_node.global_x_distance`、`timeout` | 上坡默认距离（m）与超时（s） |

`max_decel` 与 Nav2 一样填写负值，内部制动公式使用其幅值。最小接近速度是正的幅值，不能替换为表示反向速度下限的 `v_linear_min` 或 `v_angular_min`。

修改后同步 `framework/on_demand/modes/special.yaml`，重启 Function sum 生效。ROS 标准启动参数也可覆盖本次进程的值：普通组使用 `controller_server.FollowPath.translation_kp` 等名称；上坡组在名称前加 `uphill.`。本节点专用项省略 `simple_nav_node` 前缀，例如 `yaw_correction.rotation_kp` 和 `uphill.cross_track.translation_kp`。

### 3.6 检查运行值

先确认终端使用本工作空间，再读取相关节点：

```bash
ros2 pkg prefix framework
ros2 pkg prefix fishbot_navigation2
ros2 param get /controller_server FollowPath.translation_kp
ros2 param get /velocity_smoother max_velocity
ros2 param get /simple_nav_node controller_server.FollowPath.v_linear_max
ros2 param get /simple_nav_node uphill.controller_server.FollowPath.v_linear_max
```

检查启动基准时应在没有导航任务执行的状态读取；检查速度模式时应在任务期间读取，结束后应恢复到任务前的值。手写控制参数在 Function sum 启动时加载，修改文件后需要同步并重启。

## 4. micro-ROS 参数

唯一编辑文件：**`src/function/detail/continuous/micro_ros/config/boot.yaml`**。

```yaml
workspace: ""
device: /dev/ttyUSB0
baud: 921600
```

| 字段 | 说明 |
|---|---|
| `workspace` | 默认留空 `""`，使用项目统一构建的 Agent。只有使用外部工作空间时才填写相对项目根目录的路径，外部目录需有 `install/setup.bash` |
| `device` | 当前设备串口名，例如 `/dev/ttyUSB0`；设备路径本身是系统路径 |
| `baud` | 整数波特率，须与下位机一致；当前配置为 921600 |

Agent 源码保留在 `src/function/detail/continuous/micro_ros/src/`，启动包的 `package.xml` 和 `CMakeLists.txt` 位于 `micro_ros/launcher/`，使 colcon 能从根目录同时发现启动包和 Agent。`agent.py`、`config/boot.yaml` 的编辑位置保持不变。

首次运行或修改 Agent 源码时，在项目根目录执行即可：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

默认统一使用根目录的 `install/`；外部工作空间仅在显式配置 `workspace` 时加载。首次编译需要准备系统依赖，并联网下载 Agent 的构建依赖。

该配置文件不在 `configuration/`，也不经过 `sync`。修改后，在项目根目录执行：

```bash
colcon build --packages-select micro_ros
source install/setup.bash
ros2 run micro_ros agent
```

先停止旧 Agent，再单独启动或重新运行总启动，避免重复占用串口。

## 5. FAST-LIO 与 Livox 驱动参数

### 5.1 FAST-LIO

文件：**`src/config/fast_lio/mid360.yaml`**。例如在原 `/**.ros__parameters` 下修改滤波尺寸：

```yaml
/**:
  ros__parameters:
    filter_size_surf: 0.15
    filter_size_map: 0.15
```

同文件中的 `common.lid_topic`、`common.imu_topic` 是输入话题，`preprocess.blind` 为近距过滤参数，`mapping.extrinsic_T/extrinsic_R` 为外参。保留嵌套层次，数值以标定结果为准。

```bash
ros2 run chairman_config sync --files fast_lio/mid360.yaml
```

重启 FAST-LIO/导航服务生效。其他雷达型号文件使用同样的覆盖方式，但修改它们不会改变当前使用 MID360 的启动链路；换型号还需要对应 launch 选择该配置。

### 5.2 MID360 网络

文件：**`src/config/livox_ros_driver2/MID360_config.json`**。

若主机网卡 IP 为 `192.168.1.60`，在 `MID360.host_net_info` 中把以下现有键一起改为该地址：`cmd_data_ip`、`push_msg_ip`、`point_data_ip`、`imu_data_ip`。雷达地址改在 `lidar_configs[0].ip`，例如 `192.168.1.186`。这些是设备网络地址，不是工程文件路径。

```bash
ros2 run chairman_config sync --files livox_ros_driver2/MID360_config.json
```

重启驱动生效。端口字段、`lidar_type` 和外参字段不是主机 IP，不要随地址一起替换。HAP 等配置也需同步对应文件，并由对应驱动 launch 加载。

## 6. odometry 参数

### 6.1 四个场地的初始位姿

文件：**`src/config/odometry/initial_poses.yaml`**。例如修改 1 号场地，保留 2/3/4 的条目：

```yaml
poses:
  1:
    lidar_init_x: -5.10
    lidar_init_y: -1.50
    lidar_init_qz: -0.7071068
    lidar_init_qw: 0.7071068
```

通过 `tool get_init_pose` 获取的四个值按上述字段填写。odometry 启动时根据 `SELECTED_POSE` 加载该组参数，在 TF 稳定后按配置次数发布 `/initialpose`。`/odom_map` 的平面位置与航向来自地图 TF，初始位姿文件不会直接改写 TF。

```bash
ros2 run chairman_config sync --files odometry/initial_poses.yaml
```

同步后重启 odometry。当前 GICP 节点未启用 `/initialpose` 订阅，因此该发布不会直接重置配准结果；配准中的地图偏移和 TF 转发节点的初始变换需分别核对。导航 launch 的 `PRESET_POSES` 生成的字符串未接入发布动作，不作为初始化生效入口。

### 6.2 红蓝区域边界

文件：**`src/config/odometry/regions.yaml`**。例如修改 `regions_red` 中已有 id=1 的边界，保留其他区域：

```yaml
regions_red:
  - id: 1
    name: 梅林入口
    x_min: -3.60
    x_max: -2.40
    y_min: -4.80
    y_max: -1.20
```

蓝方改 `regions_blue`。坐标为地图系米，保持 min < max 和区域编号的既有含义。15 号任务使用 id 1～13 查找当前区域并计算中心；odometry 在区域类型切换时更新代价地图膨胀参数。

```bash
ros2 run chairman_config sync --files odometry/regions.yaml
```

odometry 和 `sum` 都会读取区域配置，因此都要重启。

## 7. 导航 launch、地图与 RViz

文件：**`src/fishbot/fishbot_navigation2/launch/navigation2.launch.py`**。

这里修改节点启动关系、TF 安装偏移、两个点云转扫描节点的高度/距离范围及地图文件路径。例如要换 2D 地图：把新地图和其描述放入 `src/fishbot/fishbot_navigation2/maps/`，将 `map_yaml_path` 的 `'room.yaml'` 改为新文件名。地图 YAML 内的 `image` 应指向该地图对应图像，采用相对路径。

若要调整扫描高度，在此 launch 的 `pointcloud_to_laserscan_local`、`pointcloud_to_laserscan_global` 对应 `parameters` 中修改 `min_height/max_height`；局部和全局是两份配置，需要分别确认。

```bash
colcon build --packages-select fishbot_navigation2
source install/setup.bash
```

随后重启导航服务。launch 本身不属于 `sync` 管理的文件。

RViz 的中央副本是 **`src/config/fishbot_navigation2/rviz/nav2_default_view.rviz`**。例如在该文件的 `Visualization Manager → Global Options → Fixed Frame` 设置显示坐标系为 `map`，再同步：

```bash
ros2 run chairman_config sync --files fishbot_navigation2/rviz/nav2_default_view.rviz
```

重启 RViz 后加载。Livox 的 RViz 配置同理，使用 `manifest.json` 中对应键。

导航 launch 同时配置 GICP 与 TF 转发：

| 项目 | 位置与含义 |
|---|---|
| 点云地图 | `pcd_file_path` 当前指向 `PCD/scans.pcd`，用于 GICP 配准 |
| GICP 输出 | `small_gicp_relocalization_node` 的 `map_frame/odom_frame` 为 `map1/odom1` |
| TF 转发 | C++ `tf_relay_node` 将源坐标系对转发为 `map/odom` |
| 转发频率与时间戳 | `publish_rate_hz: 20.0`；`time_offset_sec: 0.1` 表示输出时间戳相对当前 ROS 时间的偏移 |
| 初始变换 | `init_pose` 顺序为 `[x, y, z, roll, pitch, yaw]`，位置单位 m、角度单位 rad；仅在尚未收到源 TF 时使用 |

源 TF 暂时不可用时，转发节点继续发布最近一次成功读取的变换；它不执行点云配准。独立入口为 `ros2 run small_gicp_relocalization tf_relay_node`，总导航运行期间不要重复启动同一 TF 发布者。

GICP 的地图固定偏移 `T_map_3dmap` 定义在 `src/fishbot/small_gicp_relocalization/src/small_gicp_relocalization.cpp`。修改该变换涉及 C++ 代码，需构建 `small_gicp_relocalization` 后重启定位节点。地图、配准输出与转发初始变换应使用一致的场地坐标约定。
