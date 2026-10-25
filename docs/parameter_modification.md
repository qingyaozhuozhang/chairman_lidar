# 参数文件与配置修改

本文路径相对于 `chairman_lidar/`，执行命令前先加载 `source install/setup.bash`。示例说明参数位置与修改方法，实际数值应依据地图、设备标定和控制要求设置。

## 1. 参数部分的文件框架

```text
src/config/                              # 中央参数配置入口，使用 sync 同步
├── sync.py
├── manifest.json                        # 中央文件到源码目标的映射
├── framework/on_demand/
│   ├── points.yaml                      # 红蓝区点位、名称与模式
│   ├── functions.yaml                   # 特殊功能登记及任务参数
│   └── modes/
│       ├── speed_1.yaml                 # 基础导航
│       ├── speed_2.yaml                 # 冲刺与精调
│       ├── speed_3.yaml                 # 提前对齐
│       └── special.yaml                 # 特殊运动和检测参数
├── fast_lio/                            # mid360.yaml 及其他雷达配置
├── fishbot_navigation2/
│   ├── nav2_params.yaml                 # Nav2 启动基准
│   └── rviz/*.rviz
├── livox_ros_driver2/                    # MID360 网络配置及其他型号配置
└── odometry/
    ├── initial_poses.yaml               # 四个启动位置的初始位姿
    └── regions.yaml                     # 红蓝区域边界
src/function/
├── config/on_demand/                    
└── detail/continuous/
    ├── micro_ros/config/boot.yaml        # 串口及 Agent 环境
    └── odometry/config/                 # 中央 odometry 配置的包内副本
src/fishbot/fishbot_navigation2/
├── config/nav2_params.yaml              # 中央 Nav2 配置的包内副本
├── launch/navigation2.launch.py         # 节点关系、TF、地图及扫描设置
├── maps/room.yaml                       # 栅格地图描述
└── PCD/scans.pcd                        # 当前 launch 引用的点云地图
configuration/main_boot.py               # 场地选择与启动入口
```

配置按三个层次生效：中央文件用于编辑和同步；包内副本用于构建安装；节点启动时读取当前工作空间的安装配置。任务期间的运行参数还可由模式切换临时覆盖。

| 类型 | 编辑入口 | 生效方式 |
|---|---|---|
| 雷达、FAST-LIO、Nav2、odometry | `src/config/` 下对应目录 | `sync --files ...`，重启对应节点 |
| 点位、功能登记、已有模式及特殊运动控制 | `src/config/framework/on_demand/` | 同步相应文件，重启 Function sum |
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


## 3. 按需功能参数

点位、任务、模式和特殊运动控制的中央文件均可同步。以下示例更新模式 3：

```bash
ros2 run chairman_config sync --files framework/on_demand/modes/speed_3.yaml
```

新增 Python 模块或模式文件后执行：

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

使用 `ros2 run tool point` 实车标定时，工具同时修改中央文件与 `src/function/config/on_demand/points.yaml` 中选中点位的 x/y，保留各自的朝向、名称、模式和注释。

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

表中实现路径均以 `src/function/detail/on_demand/` 开头。

15/16 的动态目标入口采用单段导航；`mode` 选择参数组，模式 2 仍按距离切换冲刺与精调。模式 3 的两段提前对齐由普通定点及 `ctx.go_to_pose()` 流程调用，15/16 使用其参数时不执行提前点流程。

### 3.3 改内置速度模式

| 模式 | 中央文件 | 流程 |
|---|---|---|
| 1 | `src/config/framework/on_demand/modes/speed_1.yaml` | 基础导航 |
| 2 | `src/config/framework/on_demand/modes/speed_2.yaml` | 远处冲刺、近点精调 |
| 3 | `src/config/framework/on_demand/modes/speed_3.yaml` | 提前对齐、进入真实目标 |

三份文件均完整列出 `controller_server` 与 `velocity_smoother`，名称、节点层级及数组格式与 `nav2_params.yaml` 一致。`id` 供点位的 `mode` 引用，`name` 用于菜单显示；`parameters` 下是本模式独立使用的 Nav2 参数树。每个文件的参数及流程设置只作用于自身。

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

### 3.4 新增跑定点模式参数

新增模式可复制 `docs/templates/speed_mode.yaml` 或所需流程的现有模式文件，设置唯一 `id` 和显示用 `name`，保留完整节点块后调整参数。点位选择 `mode: 4` 时，只使用编号 4 文件中的配置；修改其他模式不会改变本模式的值。创建文件、登记同步映射和点位引用的步骤见[新增速度模式](function_development.md#4-增加新的速度模式)。

| 本模式的配置 | 导航行为 |
|---|---|
| 仅有 `parameters` | 全程使用本组参数直接导航 |
| 同时有 `parameters`、`sprint_parameters` | 按本文件的 `switch_distance` 和 `switch_check_period` 切换冲刺与精调 |
| 有 `parameters`，且 `pre_align.enabled: true` | 按本文件的 `pre_align` 设置提前对齐，再进入最终目标 |

冲刺和精调两组必须包含相同的动态参数字段，数值分别配置。提前对齐与冲刺精调使用不同的模式文件。新增已有 Nav2 参数时，在对应节点的完整参数块中填写原参数名及正确类型；未列出的参数保持当前运行值。

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
2. 设置所选模式；冲刺、精调及提前对齐的后续阶段继续使用本次导航的同一份原值记录。
3. 目标完成、失败或取消后，确认 Nav2 动作结束。
4. 恢复原值并读回核对，成功后允许组合任务继续下一步。

参数服务对单个节点使用原子设置，多个节点仍需顺序调用。磁盘 `nav2_params.yaml` 始终保存启动基准；恢复目标是执行前读取的运行值，不是重新读取磁盘文件。

若停止或恢复尚未确认，后续任务被阻止。任务空闲且相关服务可用后，可重试恢复：

```bash
ros2 service call /restore_navigation_parameters std_srvs/srv/Trigger '{}'
```

### 3.5 特殊运动与检测参数

`modes/special.yaml` 与速度文件使用相同的 `节点名 → ros__parameters → 插件名 → 参数名` 层级。`parameters` 用于普通移动、旋转和登阶，`uphill_parameters` 用于沿地图 X 方向上坡。每组完整列出 `controller_server` 和 `velocity_smoother`，并用 `simple_nav_node` 块配置本功能特有的条件。

这些配置由 `simple_nav_node` 的特殊运动控制读取，不向 Nav2 下发。标记 `# 对照项` 的字段只保留完整 Nav2 结构，本特殊运动控制器不读取；其他字段实际参与控制。不要把对照项当作特殊运动控制的调节入口。

| 配置位置（省略 `ros__parameters`） | 用途 |
|---|---|
| `controller_server.FollowPath.translation_kp` | 直线位置误差的比例增益 |
| `controller_server.FollowPath.rotation_kp` | 普通组原地旋转的比例增益 |
| `controller_server.FollowPath.v_linear_max`、`v_angular_max` | 线速度与角速度幅值上限 |
| `controller_server.FollowPath.min_approach_linear_velocity`、`min_approach_angular_velocity` | 接近目标时的最小速度幅值 |
| `controller_server.general_goal_checker.xy_goal_tolerance`、`yaw_goal_tolerance` | 位置与航向结束容差 |
| `velocity_smoother.max_accel`、`max_decel` | 特殊运动控制的加速与制动限制，顺序为 `[x, y, yaw]` |
| 普通组 `controller_server.controller_frequency` | 阻塞式特殊运动控制循环的周期来源，默认 50 Hz；登阶定时器由通信节点独立按 50 Hz 调度 |
| `simple_nav_node.yaw_correction` | 移动时保持起始朝向；`rotation_kp` 为增益，`max_velocity_ratio` 为角速度上限比例 |
| `simple_nav_node.cross_track` | 横向纠偏，使用 `translation_kp`、`v_linear_max`、`min_approach_linear_velocity`、`xy_goal_tolerance` |
| 普通组 `simple_nav_node.lift_check_height` | 抬升检测高度阈值 |
| 上坡组 `simple_nav_node.global_x_distance`、`timeout` | 上坡默认距离（m）与超时（s） |

`max_decel` 与 Nav2 一样填写负值，内部制动公式使用其幅值。最小接近速度是正的幅值，不能替换为表示反向速度下限的 `v_linear_min` 或 `v_angular_min`。

修改后同步 `framework/on_demand/modes/special.yaml`，重启 Function sum 生效。ROS 标准启动参数也可覆盖本次进程的值：普通组使用 `controller_server.FollowPath.translation_kp` 等名称；上坡组在名称前加 `uphill.`。本节点专用项省略 `simple_nav_node` 前缀，例如 `yaw_correction.rotation_kp` 和 `uphill.cross_track.translation_kp`。

### 3.6 检查运行值

先确认终端使用本工作空间，再读取相关节点：

```bash
ros2 param get /controller_server FollowPath.translation_kp
ros2 param get /velocity_smoother max_velocity
ros2 param get /simple_nav_node controller_server.FollowPath.v_linear_max
ros2 param get /simple_nav_node uphill.controller_server.FollowPath.v_linear_max
```

检查启动基准时应在没有导航任务执行的状态读取；检查速度模式时应在任务期间读取，结束后应恢复到任务前的值。

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
colcon build
source install/setup.bash
```

首次编译需要准备系统依赖，并联网下载 Agent 的构建依赖。

该配置文件不在 `configuration/`，也不经过 `sync`。修改后，在项目根目录执行：

```bash
colcon build --packages-select micro_ros
source install/setup.bash
ros2 run micro_ros agent
```


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


### 5.2 MID360 网络

文件：**`src/config/livox_ros_driver2/MID360_config.json`**。

雷达地址主要修改 `lidar_configs[0].ip`，例如 `192.168.1.186`。这些是设备网络地址。Livox_mid360雷达具体配置可参考[网站说明](https://qingyaozhuozhang.github.io/navigation/lidar/Mid-360%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E9%85%8D%E7%BD%AE%E6%95%99%E7%A8%8B/)

```bash
ros2 run chairman_config sync --files livox_ros_driver2/MID360_config.json
```

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

四个字段表示所选启动点处 `base_footprint` 在栅格地图 `map` 中的位置与朝向：x/y 单位 m，qz/qw 为平面朝向四元数。可通过 RViz 的 `2D Pose Estimate` 配合 `tool get_init_pose` 获取这些值；定位运行期间，该操作也会重新初始化 GICP。

同一文件中的 `mapping_pose` 保存建图点，即这份 PCD 的坐标系原点在 `map` 中的位姿：

```yaml
mapping_pose:
  x: -5.082
  y: -1.5
  z: 0.0
  roll: 0.0
  pitch: 0.0
  yaw: -1.5707963267948966
```

位置单位 m，角度单位 rad。建图点初值采用 1 号点的位置和朝向，将该点定义为 PCD 的零位姿；`mapping_pose` 保存这个原点在导航 `map` 中的坐标。建图点独立保存，不随 `poses` 中的点位标定自动改变。每份 PCD 使用一组建图点标定；在 1 号点建图后从 2 号点启动时，保留 `mapping_pose`，选择 `poses` 中的 2 号。重新建图或转换 PCD 坐标系时才需重新核对建图点。`point.py` 的启动点标定只修改 1～4 号位姿，不修改 `mapping_pose`。

使用 `ros2 run tool point` 实车标定时，先选菜单 `3：启动点标定`，再选启动编号 1～4。机器人停稳且朝向正确后确认采集，工具读取 10 条新 `/odom_map` 消息，去极值平均 x/y 与解环绕后的 yaw，将 yaw 转为 qz/qw，同时写入中央文件和 `src/function/detail/continuous/odometry/config/initial_poses.yaml`。四个字段保留 6 位小数，其他编号、字段和注释保持不变；不写 `install`，不创建备份目录。保存后构建 `odometry`，重新加载工作空间，再重启 Navigation2 与 odometry。

应先确认 GICP 已配准成功、地图位姿正确，再标定启动点。例如从已知启动位置完成定位后，将车移动到其他启动位置采集。工具记录的是当前定位结果；未配准时仅由旧启动值生成的位姿，不能作为重新标定的依据。

`main_boot` 将所选编号通过 `SELECTED_POSE` 传给导航与 odometry。GICP launch 从 odometry 包内同一份文件读取 `mapping_pose`，转换为 `map_to_pcd`；读取所选启动点，转换为 `initial_map_pose`。两项传入参数的顺序均为 `[x, y, z, roll, pitch, yaw]`，平面启动点的 z、roll、pitch 为 0。定位节点结合当前 `odom → base_footprint` 和地图固定变换，分别计算匹配初值与初始 `map → odom`，因此启动点中无需人工扣除雷达安装偏移。

odometry 默认不自动发布 `/initialpose`，其 `/odom_map` 的平面位置与航向直接来自地图 TF。启动点负责初始化，后续位置由局部里程计与 GICP 匹配共同确定。

```bash
ros2 run chairman_config sync --files odometry/initial_poses.yaml
```

同步后重启 Navigation2 与 odometry，并使用同一场地编号。启动时应将小车放在对应点位且朝向一致，等待 `[定位就绪] GICP 首次配准收敛` 后核对 RViz 位姿。仅修改磁盘文件不会重置已经运行的定位节点。

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

使用 `ros2 run tool point` 标定区域中心时，工具同时修改中央文件与 `src/function/detail/continuous/odometry/config/regions.yaml`，将选中区域的边界设为平均位置 x/y 各向外扩展 0.6 m。

## 7. 导航 launch、地图与 RViz

文件：**`src/fishbot/fishbot_navigation2/launch/navigation2.launch.py`**。

这里修改节点启动关系、TF 安装偏移、两个点云转扫描节点的高度/距离范围及地图文件路径。例如要换 2D 地图：把新地图和其描述放入 `src/fishbot/fishbot_navigation2/maps/`，将 `map_yaml_path` 的 `'room.yaml'` 改为新文件名。地图 YAML 内的 `image` 应指向该地图对应图像，采用相对路径。

若要调整扫描高度，在此 launch 的 `pointcloud_to_laserscan_local`、`pointcloud_to_laserscan_global` 对应 `parameters` 中修改 `min_height/max_height`；局部和全局是两份配置，需要分别确认。

```bash
colcon build --packages-select fishbot_navigation2
source install/setup.bash
```

RViz 的中央配置文件是 **`src/config/fishbot_navigation2/rviz/nav2_default_view.rviz`**。例如在该文件的 `Visualization Manager → Global Options → Fixed Frame` 设置显示坐标系为 `map`，再同步：

```bash
ros2 run chairman_config sync --files fishbot_navigation2/rviz/nav2_default_view.rviz
```

重启 RViz 后加载。Livox 的 RViz 配置同理，使用 `manifest.json` 中对应键。

### 7.1 GICP 配准与地图坐标

建图点与启动点统一配置在 `odometry/initial_poses.yaml`；其余配准、坐标系与发布参数采用定位节点默认值。导航 launch 启动一个 GICP 节点，统一负责初始化、配准和 `map → odom` 发布：

| 项目 | 位置与含义 |
|---|---|
| 点云地图 | `pcd_file_path` 当前指向 `PCD/scans.pcd`，用于 GICP 配准 |
| GICP 输出 | `map_frame: map`、`odom_frame: odom`，由定位节点直接发布该 TF |
| 启动点参考 | `robot_base_frame: base_footprint`；`initial_map_pose` 由 launch 根据四个启动点生成 |
| 地图坐标标定 | `initial_poses.yaml` 的 `mapping_pose` 由 launch 转换为 `map_to_pcd`，表示点云地图坐标系在栅格地图 `map` 中的位姿，单位 m/rad；不随启动编号变化 |
| 点云输入 | `input_cloud_topic: /cloud_registered`；根据消息的 frame 与时间戳变换到 `odom` 后配准 |
| 配准频率 | `registration_rate_hz: 2.0`，每轮处理累积点云 |
| TF 频率与时间戳 | `publish_rate_hz: 20.0`；`time_offset_sec: 0.1` 表示输出时间戳相对当前 ROS 时间的偏移 |
| 地图降采样 | `global_leaf_size: 0.05`，单位 m；定位节点加载 PCD 后进行体素降采样 |
| 实时点云处理 | FAST-LIO 输出 `/cloud_registered`；GICP 转换到 `odom`、过滤非有限点，再按 `registered_leaf_size: 0.05`（m）进行体素降采样并计算协方差 |
| 匹配参数 | 默认 `num_threads: 16`、`num_neighbors: 10`、`max_dist_sq: 4.0`；最后一项为对应点距离平方上限，单位 m² |

用 `T_A_B` 表示将 B 系坐标转换为 A 系坐标的变换，初始化关系为：

```text
A = T_map_pcd                 ← initial_poses.yaml 的 mapping_pose 建图点
B = T_map_start               ← 所选启动点，作为本次输出的固定参考
O = T_odom_base_footprint      ← 初始化时的局部里程计 TF，包含安装偏移

启动点相对建图点：S = T_pcd_start = inverse(A) × B
GICP 初值：T_pcd_odom = S × inverse(O)
初始 TF： T_map_odom = B × inverse(O)
匹配后先换到启动参考系：T_start_odom = inverse(S) × GICP 的 T_pcd_odom 匹配结果
输出到导航地图：T_map_odom = B × T_start_odom
```

例如建图原点取 1 号点 `(-5.082, -1.5, yaw=-π/2)`，选择 2 号启动点 `(5.708, -5.257, yaw=π/2)`，底盘在原 PCD 中的初始位置为 `(3.757, 10.790)`，朝向为 `π`。随后再结合当前局部里程计和安装偏移得到 `T_pcd_odom`，作为第一轮 GICP 配准的初值；无需移动 PCD 地图。从建图点本身启动时，底盘相对 PCD 的位姿为零，配准初值仍按实际局部里程计和安装偏移换算。

输出端的 `map_to_start` 使用所选启动点的位置与朝向；`result_t_` 保留 GICP 的原始 `PCD ← odom` 结果。先用 `inverse(S)` 将结果转换到启动参考系，再乘 `map_to_start`。此时输出等价于 `A × result_t_`；直接计算 `B × result_t_` 会重复叠加建图点到启动点的相对变换。

启动日志 `[GICP 启动]` 显示所选编号和实际读取的位姿文件；`[定位初始化]` 显示底盘在 `map` 与 PCD 中的位置、朝向；`[匹配初值]` 显示实际传给配准的 `PCD ← odom` 变换。位置单位 m，航向单位 rad。这些信息在初始化时输出，首次配准成功仍以 `[定位就绪]` 为准。

`map_to_pcd` 是两份地图之间的固定标定，切换启动点时保持不变；若重新建图或更改地图坐标约定，则需要重新核对。地图 YAML 的 `origin` 描述栅格图像原点在 `map` 中的位置，不能直接当作 `map_to_pcd`。两张地图即使显示中心不同，也应先核对相同物理点的坐标与轴方向，再确定平移和旋转。

不同启动点改变公式中的 B，因此得到不同的 GICP 初值。如果将 A 也替换为同一个 B，`inverse(B) × B` 会相消，初值丢失启动位置的区别，并错误地把地图关系绑定到当前机器人位姿。仅当已把 PCD 中所有点转换到栅格地图的 `map` 坐标系时，`mapping_pose` 的六个字段才都设为 0，使 `map_to_pcd` 为单位变换；单纯降采样不完成这项坐标转换。

节点先等待局部 TF，再发布换算出的初始变换；配准不收敛、有效对应点不足或结果非有限时保留上次变换。TF 定时器与配准使用独立回调组，独立可执行程序采用多线程执行器，使 TF 发布不必等待一轮配准结束。TF 持续发布不代表每一帧都有新匹配；首次匹配成功以 `[定位就绪]` 日志为准。

首次有效收敛输出 `[定位就绪]`，后续有效收敛以 INFO 级别输出 `[定位更新]`，每 3 秒最多一次；失败后再次有效收敛立即输出 `[定位恢复]`。成功日志中的 `source` 是本轮参与配准的实时点云点数，`target` 是实际参与匹配的地图点数。实时扫描只覆盖当前可见范围，地图则由多帧构成；相同体素边长代表相同采样尺度，不能保证两者点数或局部密度处处相等。

配准告警记录失败原因、实际迭代次数与上限、有效对应点数与本轮源点数、地图点数、误差、耗时和连续失败轮数。`converged=0` 表示本轮迭代未满足收敛条件；`inliers` 仅表示参与匹配的有效对应点数量，不能单独证明定位成功。当前每轮最多迭代 10 次，平移和旋转的单次更新量需分别不超过 0.001 m 和 0.1°；这两个阈值是迭代停止条件，不代表绝对定位精度。告警中的“运行到迭代上限”与“优化器提前退出”用于区分排查方向；`error` 是配准目标函数值，不是米制位置误差。收敛仍可能发生在错误的局部最优位置，需要结合 RViz 中实时点云与地图的重合情况核对。

RViz 发布的 `/initialpose` 必须使用 `map` 坐标系；节点按同一公式重新设置配准初值与初始 TF，并清空待配准点云。总导航运行时不要启动第二个 `map → odom` 发布者。

修改定位 C++ 或 launch 后，构建 `small_gicp_relocalization`，重新加载工作空间并重启导航。
