# 参数文件与配置修改

本文所有路径相对于 `chairman_navigation/`。先 `source install/setup.bash`。示例数值用于说明修改方式，请按自己的标定和控制需求填写；YAML 片段应合并到已有节点或条目，保留其他字段，不重复同名键。

## 1. 参数部分的文件框架

```text
src/config/                               # 基础配置中央副本，使用 sync 覆盖
├── sync.py
├── manifest.json                         # 文件名到目标路径的映射
├── fast_lio/
│   ├── mid360.yaml                       # 当前 MID360 链路使用
│   └── avia.yaml、horizon.yaml、ouster64.yaml、velodyne.yaml
├── fishbot_navigation2/
│   ├── nav2_params.yaml                  # Nav2 和 simple_nav_node 基础参数
│   └── rviz/*.rviz
├── livox_ros_driver2/
│   ├── MID360_config.json                # 当前 MID360 网络配置
│   ├── HAP_config.json、mixed_HAP_MID360_config.json
│   └── *.rviz
└── odometry/
    ├── initial_poses.yaml                # 4 个场地的里程计初始位姿
    └── regions.yaml                      # 红/蓝区域边界
src/function/
├── config/on_demand/                     # 按需配置，不使用 sync
│   ├── points.yaml                       # 点位坐标、名称、模式
│   ├── functions.yaml                    # 编号、模块、说明、任务自身参数
│   └── modes/
│       ├── base.yaml                     # 模式 1
│       ├── dynamic.yaml                  # 模式 2：按距离切换
│       └── pre_align.yaml                # 模式 3：提前对齐
└── detail/continuous/
    ├── micro_ros/config/boot.yaml         # 串口和 Agent 环境，不使用 sync
    └── odometry/config/                  # 由 src/config/odometry/ 同步到此
src/fishbot/fishbot_navigation2/
├── launch/navigation2.launch.py          # 导航启动、地图路径、TF、扫描参数
├── maps/room.yaml                        # 2D 地图描述
└── PCD/test.pcd                          # launch 引用的点云地图
configuration/main_boot.py                # 选择场地、编排启动；不保存雷达或串口参数
```

先判断参数属于哪一类：

| 类型 | 编辑处 | 生效方式 |
|---|---|---|
| 雷达、FAST-LIO、Nav2、odometry 基础配置 | `src/config/` | `sync --files ...`，重启相应节点 |
| 点位/任务参数/临时速度模式 | `src/function/config/on_demand/` | 构建 `framework`，重启 `sum` |
| micro-ROS | `src/function/detail/continuous/micro_ros/config/boot.yaml` | 构建 `micro_ros`，重启 Agent |
| 导航启动关系、地图、TF、扫描设置 | `src/fishbot/fishbot_navigation2/` | 构建 `fishbot_navigation2`，重启导航 |
| 场地编号 | `main_boot` 交互输入或 `--selected-pose` | 传给本次启动的所有子进程 |

## 2. sync.py 的使用说明

### 2.1 全部覆盖

```bash
ros2 run chairman_config sync --all
```

覆盖 `src/config/manifest.json` 登记的所有文件，不是扫描并覆盖任意新增文件。

### 2.2 指定文件覆盖

```bash
ros2 run chairman_config sync --files fast_lio/mid360.yaml
ros2 run chairman_config sync --files fishbot_navigation2/nav2_params.yaml odometry/regions.yaml
```

`--files` 后写相对 `src/config/` 的文件名，多个文件以空格分隔，不写 `src/config/` 前缀。参数只有：

| 参数 | 含义 |
|---|---|
| `--all` | 覆盖全部已登记配置 |
| `--files 文件1 [文件2 ...]` | 覆盖这些文件 |
| `-h` / `--help` | 显示帮助 |

两种覆盖操作必须二选一；不带参数显示用法并退出。没有内容校验、预览或交互选择步骤。文件原样复制，原文件自动备份到 `.configuration_backups/`；写入失败时回退已经写入的文件。未知配置名、目标文件不存在或路径越过本工程边界会报错。

### 2.3 实际覆盖到哪里

| `--files` 名称 | 源码目标 |
|---|---|
| `fast_lio/mid360.yaml` | `src/fishbot/fast_lio/config/mid360.yaml` |
| `fishbot_navigation2/nav2_params.yaml` | `src/fishbot/fishbot_navigation2/config/nav2_params.yaml` |
| `livox_ros_driver2/MID360_config.json` | `src/fishbot/livox_ros_driver2/config/MID360_config.json` |
| `odometry/initial_poses.yaml` | `src/function/detail/continuous/odometry/config/initial_poses.yaml` |
| `odometry/regions.yaml` | `src/function/detail/continuous/odometry/config/regions.yaml` |

其他可用名称以 `src/config/manifest.json` 为准，包括其登记的其他雷达配置和 RViz 文件。同步同时更新本工程已有的安装配置。运行中的 ROS 节点不会自动重新加载磁盘文件，需要重启。不要只编辑目标副本，否则下次 `sync` 会用中央副本覆盖它。

## 3. 按需功能参数

这些文件不走 `sync`。修改或新增文件之后，在退出旧 `sum` 后执行：

```bash
colcon build --symlink-install --base-paths src configuration tool --packages-select framework
source install/setup.bash
# 再由总启动启动 sum，或在已设置同一 SELECTED_POSE 的调试终端单独运行。
ros2 run framework sum
```

### 3.1 改点位坐标或点位所用模式

文件：**`src/function/config/on_demand/points.yaml`**。

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

文件：**`src/function/config/on_demand/functions.yaml`**。

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

### 3.3 改内置速度模式

修改对应的完整路径：

| 模式 | 文件 |
|---|---|
| 1 基础（base） | `src/function/config/on_demand/modes/base.yaml` |
| 2 按距离切换 | `src/function/config/on_demand/modes/dynamic.yaml` |
| 3 提前对齐 | `src/function/config/on_demand/modes/pre_align.yaml` |

例如在 `base.yaml` 已有 `simple_nav_node` 下改：

```yaml
simple_nav_node:
  nav2_profile_1_v_linear_max: 0.40
  nav2_profile_1_v_linear_min: -0.40
  nav2_profile_1_max_velocity: [0.40, 0.40, 1.0]
  nav2_profile_1_min_velocity: [-0.40, -0.40, -1.0]
```

同时调整控制器与速度平滑器的限速，保留文件的 `id/name/base_mode` 和其他参数。修改 `dynamic.yaml` 的 `simple_nav_node.nav2_profile_2_switch_distance`，例如 `1.0`，可调整快慢切换距离；修改 `pre_align.yaml` 的 `simple_nav_node.nav2_profile_3_pre_align_distance`，例如 `0.4`，可调整提前对齐距离。

**正常无参数启动时，这三个文件的同名模式参数会覆盖 `nav2_params.yaml` 内的旧模式默认值。** 要改任务速度，应改这里，避免只改旧 `nav2_profile_*` 字段后看不到效果。

### 3.4 新模式的临时 Nav2 参数

新建 **`src/function/config/on_demand/modes/precise.yaml`**：

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

然后把 `points.yaml` 某点的 `mode` 改成 `4`。`base_mode` 只能取 1/2/3，继承对应流程；新增模式用 `overrides` 写参数，不新增一套 `simple_nav_node`。参数按实际 ROS 节点名分组，名称、类型及动态修改能力必须由该节点支持。

任务先保存运行值，再应用当前模式，结束或中止后恢复原值；不修改磁盘 `nav2_params.yaml`。若恢复失败，后续任务被阻止。排查服务后可调用：

```bash
ros2 service call /restore_navigation_parameters std_srvs/srv/Trigger '{}'
```

## 4. micro-ROS 参数

唯一编辑文件：**`src/function/detail/continuous/micro_ros/config/boot.yaml`**。

```yaml
workspace: ""
device: /dev/ttyUSB1
baud: 115200
```

| 字段 | 说明 |
|---|---|
| `workspace` | 默认留空 `""`，使用项目统一构建的 Agent。只有使用外部工作空间时才填写相对项目根目录的路径，外部目录需有 `install/setup.bash` |
| `device` | 当前设备串口名，例如 `/dev/ttyUSB0`；设备路径本身是系统路径 |
| `baud` | 整数波特率，须与下位机一致；原值为 921600 |

Agent 源码保留在 `src/function/detail/continuous/micro_ros/src/`，启动包的 `package.xml` 和 `CMakeLists.txt` 位于 `micro_ros/launcher/`，使 colcon 能从根目录同时发现启动包和 Agent。`agent.py`、`config/boot.yaml` 的编辑位置保持不变。

首次运行或修改 Agent 源码时，在项目根目录执行即可：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

所有包使用根目录的 `install/`，不再加载 `micro_ros/install/` 或原来的 `/uros_ws`。首次编译需要准备系统依赖，并联网下载 Agent 的构建依赖。

该配置文件不在 `configuration/`，也不经过 `sync`。修改后，在项目根目录执行：

```bash
colcon build --symlink-install --base-paths src configuration tool --packages-select micro_ros
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

通过 `tool get_init_pose` 获取的四个值按上述字段填写。此文件用于 odometry 的位姿转换；场地编号仍由 `main_boot` 选择。

```bash
ros2 run chairman_config sync --files odometry/initial_poses.yaml
```

重启 odometry 生效。`navigation2.launch.py` 中还保留了旧 `PRESET_POSES/INIT_*` 变量，但当前生成的 `init_pose_str` 没有被发布动作引用；不要把修改这些旧变量当成已完成初始位姿发布。

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

蓝方改 `regions_blue`。坐标为地图系米，保持 min < max 和区域编号的既有含义。15 号区域对准任务使用 id 1～13 计算所在区域中心。

```bash
ros2 run chairman_config sync --files odometry/regions.yaml
```

odometry 和 `sum` 都会读取区域配置，因此都要重启。

## 7. 导航 launch、地图与 RViz

文件：**`src/fishbot/fishbot_navigation2/launch/navigation2.launch.py`**。

这里修改节点启动关系、TF 安装偏移、两个点云转扫描节点的高度/距离范围及地图文件路径。例如要换 2D 地图：把新地图和其描述放入 `src/fishbot/fishbot_navigation2/maps/`，将 `map_yaml_path` 的 `'room.yaml'` 改为新文件名。地图 YAML 内的 `image` 应指向该地图对应图像，采用相对路径。

若要调整扫描高度，在此 launch 的 `pointcloud_to_laserscan_local`、`pointcloud_to_laserscan_global` 对应 `parameters` 中修改 `min_height/max_height`；局部和全局是两份配置，需要分别确认。

```bash
colcon build --symlink-install --base-paths src configuration tool --packages-select fishbot_navigation2
source install/setup.bash
```

随后重启导航服务。launch 本身不属于 `sync` 管理的文件。

RViz 的中央副本是 **`src/config/fishbot_navigation2/rviz/nav2_default_view.rviz`**。例如在该文件的 `Visualization Manager → Global Options → Fixed Frame` 设置显示坐标系为 `map`，再同步：

```bash
ros2 run chairman_config sync --files fishbot_navigation2/rviz/nav2_default_view.rviz
```

重启 RViz 后加载。Livox 的 RViz 配置同理，使用 `manifest.json` 中对应键。
