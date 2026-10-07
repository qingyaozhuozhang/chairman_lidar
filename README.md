# chairman_navigation

面向 BR 建筑机器人的 ROS2 导航与功能调用工程，以 `odometry_navigation` 为基础整理，使用 Livox MID360、FAST-LIO 和 Nav2。工程统一管理导航启动、基础配置、按需任务和持续运行功能。

当前提供场地选择、固定点导航、临时速度/PID 模式、旋转与移动等特殊任务，以及 micro-ROS 通信和位姿工具。具体运动实现位于各功能目录，可以通过统一接口组合或扩展。

**当前实现状态：**沿用原 FAST-LIO 里程计和运动控制算法。`small_gicp_relocalization` 节点仍随导航启动，但点云配准计算已禁用，保留初始位姿处理和 `map → odom` 变换发布。仓库中的地图、点位、外参和速度参数需要按实际机器人与场地调整。

| 文档 | 内容 |
|---|---|
| [参数修改说明](docs/parameter_modification.md) | 各类参数的编辑路径、同步命令和生效方式 |
| [功能开发说明](docs/function_development.md) | 菜单交互、任务接口、新增点位/模式/特殊功能/持续功能 |
| [功能模板](docs/templates/) | 按需任务、速度模式和持续节点模板 |

## 1. 整个代码框架

下文文件路径均相对于 `chairman_navigation/`，命令也在此目录执行。

```text
chairman_navigation/
├── src/
│   ├── config/                          # ROS 包 chairman_config：基础参数中央副本
│   │   ├── fast_lio/                    # 含 mid360.yaml
│   │   ├── fishbot_navigation2/          # 含 nav2_params.yaml、rviz/
│   │   ├── livox_ros_driver2/            # 含 MID360_config.json
│   │   ├── odometry/                    # 初始位姿、区域边界
│   │   ├── manifest.json                # 中央副本到实际配置的相对路径映射
│   │   └── sync.py                      # 覆盖全部或指定配置文件
│   ├── fishbot/                         # 算法、驱动和消息接口包
│   │   ├── fast_lio/
│   │   ├── fishbot_navigation2/
│   │   │   └── launch/navigation2.launch.py  # 总导航入口
│   │   ├── livox_ros_driver2/
│   │   ├── pb_nav2_plugins/
│   │   ├── pb_omni_pid_pursuit_controller/
│   │   ├── pointcloud_to_laserscan/
│   │   ├── small_gicp_relocalization/
│   │   ├── terrain_analysis/
│   │   ├── terrain_analysis_ext/
│   │   └── custom_msg/
│   └── function/
│       ├── config/on_demand/
│       │   ├── points.yaml              # 红蓝点位、名称、所用模式
│       │   ├── functions.yaml           # 特殊功能编号、说明、模块、参数
│       │   └── modes/                   # 任务期间使用的速度/PID参数
│       ├── detail/
│       │   ├── continuous/              # 持续功能：独立 ROS 包或功能包集合
│       │   │   ├── odometry/            # 包 odometry：/odom_map 等位姿转换
│       │   │   └── micro_ros/           # micro-ROS 包集合，与导航统一构建
│       │   │       ├── launcher/        # ROS 包 micro_ros 的 CMakeLists.txt、package.xml
│       │   │       ├── agent.py         # ros2 run micro_ros agent
│       │   │       ├── config/boot.yaml
│       │   │       └── src/             # micro_ros_setup、micro_ros_agent、micro_ros_msgs
│       │   └── on_demand/               # 按需功能，Python 包 chairman_tasks
│       │       ├── fixed_point/
│       │       │   ├── task.py          # 定点导航实际实现
│       │       │   └── modes/           # dynamic.py、pre_align.py
│       │       └── special/<功能名>/task.py  # 每个功能写自己的实现
│       └── framework/                   # ROS 包 framework
│           ├── sum.py                   # 功能菜单
│           └── core/                    # 公共接口、通信、参数恢复
├── configuration/                       # ROS 包 configuration
│   ├── main_boot.py                     # 场地选择与总启动
│   └── tests/
├── tool/                                # ROS 包 tool
│   ├── get_init_pose.py
│   ├── get_pose.py
│   └── downsample_map.py
├── docs/                                # 配置详解、功能开发和模板
├── runtime/                             # 启动时生成的运行输出，不提交
├── .gitignore                           # 排除构建产物、运行输出和配置备份
└── README.md
```

`continuous` 启动后持续处理数据；`on_demand` 收到一次调用后执行任务并返回结果。`framework/core/runtime.py` 管理 ROS 连接、任务生命周期和退出；具体运动实现放在对应 `detail/on_demand/` 目录。基础配置通过 `sync` 覆盖磁盘文件，任务模式通过 ROS 参数服务临时生效，结束后恢复。

## 2. 从构建到总启动

### 2.1 获取源码与准备环境

已验证的构建环境为 **Ubuntu 22.04 x86_64、ROS2 Humble、Python 3.10**。先按 [ROS2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 准备 ROS2；克隆源码前还需要 Git，未安装时执行 `sudo apt install git`。

以下克隆命令按 `chairman_navigation/` 本身作为仓库根目录编写。在 GitHub 页面通过 **Code → HTTPS** 复制本仓库地址，运行时粘贴：

```bash
read -r -p "本仓库的 Git URL: " navigation_repo_url
git clone "$navigation_repo_url" chairman_navigation
cd chairman_navigation
```

后续命令均在包含 `src/`、`configuration/`、`tool/` 的工程根目录执行。如果源码位于一个更大的仓库中，先进入其中的 `chairman_navigation/` 子目录，避免与原工程的同名包一起构建。

安装构建工具和导航启动使用的 ROS 包：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  git build-essential cmake python3-colcon-common-extensions python3-rosdep \
  python3-pytest python3-yaml python3-open3d gnome-terminal \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

首次使用 rosdep 的电脑先运行一次 `sudo rosdep init`，已初始化的电脑跳过。随后安装各包声明的依赖：

```bash
rosdep update
rosdep install --from-paths src configuration tool --ignore-src --rosdistro humble -r -y
```

`micro_ros_agent`、`micro_ros_msgs`、`micro_ros_setup` 已包含在项目源码中，rosdep 的 `--ignore-src` 会跳过这些本地包，仅安装其系统依赖。其余依赖报错需处理完成后再构建。

以下依赖需准备：

| 依赖 | 准备方式 | 本工程中的用途 |
|---|---|---|
| Livox SDK2 | 按 [官方 C++ 安装说明](https://github.com/Livox-SDK/Livox-SDK2#2-installation) 编译并安装共享库和头文件 | `livox_ros_driver2` 的编译与运行依赖 |
| small_gicp | 按 [官方 C++ 安装说明](https://github.com/koide3/small_gicp#installation) 安装头文件；仅安装 Python 包不能替代 C++ 依赖 | 当前保留的定位节点源码仍包含这些头文件，即使配准计算已禁用也需要准备 |
| micro-ROS Agent | 源码位于 `src/function/detail/continuous/micro_ros/src/`，随根目录 `colcon build` 一起构建 | 向匹配固件的下位机提供通信；首次编译需联网下载 Micro-XRCE-DDS-Agent 等构建依赖 |

`micro_ros/` 是包集合目录，包描述文件位于 `launcher/`，因此 colcon 能继续发现同目录 `src/` 下的 Agent 和消息包。所有产物统一进入项目根目录的 `build/`、`install/`、`log/`，无需单独构建内部工作空间。

`config/boot.yaml` 的 `workspace` 默认留空，直接使用 `source install/setup.bash` 加载的本项目 Agent。只有切换到外部 Agent 工作空间时才填写该字段。暂不连接下位机时可用 `ros2 run configuration main_boot --no-micro-ros` 跳过 Agent。

### 2.2 构建项目

依赖准备完成后，在工程根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Livox 驱动会自动选择当前 ROS2 提供的消息接口构建方式，不需要先进入驱动目录运行 `./build.sh humble`，也不需要传 `-DHUMBLE_ROS=humble`。首次构建和后续构建都可在本目录执行 `colcon build`。

如果首次构建停在 `micro_ros_agent:build 6%`，查看 `log/latest_build/micro_ros_agent/stdout_stderr.log`。`Performing download step (git clone) for 'xrceagent'` 表示正在从 GitHub 下载依赖；连接超时会自动重试，需恢复网络后重新构建。成功下载和编译后，保留 `build/` 即可复用依赖缓存。

内存紧张时，可用 `MAKEFLAGS="-j2 -l2" colcon build --executor sequential` 限制同时编译的数量。开发时也可选择 `colcon build --symlink-install`；新增模块或配置文件后仍需构建，以便安装到对应包中。

`source /opt/ros/humble/setup.bash` 提供 ROS2 基础环境；`source install/setup.bash` 让当前终端找到本工程的功能包。它们不会让运行中的节点重新加载代码或参数。若使用的 Python 虚拟环境缺少 ROS 构建模块，退出该虚拟环境后重新加载 ROS 环境再构建。

目前共 19 个包（包含 micro-ROS Agent、消息包和 setup 工具）。新终端需要重新 `source install/setup.bash`。迁移电脑时复制源码，重新安装依赖和构建；[.gitignore](.gitignore) 已排除 `build/`、`install/`、`log/`、`runtime/` 和 `.configuration_backups/` 等本机生成内容。

### 2.3 首次运行前确认参数

| 项目 | 编辑位置 | 需要确认的内容 |
|---|---|---|
| MID360 网络 | `src/config/livox_ros_driver2/MID360_config.json` | 雷达 IP、主机接收 IP，与实际网卡设置一致 |
| 雷达与 IMU 配置 | `src/config/fast_lio/mid360.yaml` | 输入话题、滤波参数、安装外参 |
| Nav2 基础配置 | `src/config/fishbot_navigation2/nav2_params.yaml` | 机器人尺寸、控制器、代价地图等参数 |
| 场地初始位姿与区域 | `src/config/odometry/initial_poses.yaml`、`src/config/odometry/regions.yaml` | 4 个场地的初始位姿及红蓝区域边界 |
| 定点与任务模式 | `src/function/config/on_demand/` | 红蓝点位、任务参数、速度模式 |
| 下位机通信 | `src/function/detail/continuous/micro_ros/config/boot.yaml` | 默认使用本项目 Agent；串口设备、波特率及可选外部工作空间 |
| 地图和机器人模型 | `src/fishbot/fishbot_navigation2/launch/navigation2.launch.py` 及其引用资源 | 当前加载 `maps/room.yaml`、`PCD/test.pcd`；核对 URDF、TF 安装偏移和扫描范围 |

按设备修改 `src/config/` 的雷达 IP、里程计初始位姿和 Nav2 基础参数，然后同步，例如：

```bash
ros2 run chairman_config sync --files livox_ros_driver2/MID360_config.json odometry/initial_poses.yaml
```

micro-ROS 和按需功能参数不使用 `sync`，修改后构建对应的 `micro_ros` 或 `framework` 包再启动。修改地图、模型和导航 launch 后构建 `fishbot_navigation2`。具体文件、修改示例和生效步骤见 [参数修改说明](docs/parameter_modification.md)。

### 2.4 运行 main_boot.py

```bash
ros2 run configuration main_boot
```

当前终端询问：

```text
初始位置 1红武馆 / 2红对抗 / 3蓝武馆 / 4蓝对抗:
```

输入 1～4。`main_boot.py` 将选择通过 `SELECTED_POSE` 传给每个子进程，默认分别打开终端：

| 终端 | 实际入口 | 职责 |
|---|---|---|
| Micro ROS Agent | `ros2 run micro_ros agent` | 持续串口通信 |
| Navigation2 | `ros2 launch fishbot_navigation2 navigation2.launch.py` | 雷达、FAST-LIO、地形分析、Nav2、RViz 等导航服务 |
| Function sum | `ros2 run framework sum` | 功能选择与执行；不再选择场地 |
| Odometry | `ros2 run odometry odometry` | 原里程计转换与 `/odom_map` 发布 |

导航服务统一调用 `src/fishbot/fishbot_navigation2/launch/navigation2.launch.py`。`sum` 只使用已选场地对应的红/蓝点位和区域，不弹出第二次场地选择。默认需要 `gnome-terminal`。启动命令下发后，应等相关节点就绪再选择功能。

`main_boot` 的全部参数：

| 参数 | 默认行为/作用 | 示例 |
|---|---|---|
| 不加参数 | 交互选择场地，再启动上述服务 | `ros2 run configuration main_boot` |
| `--selected-pose 1\|2\|3\|4` | 直接指定场地，跳过场地提示 | `--selected-pose 4` |
| `--no-sum` | 不启动菜单，也不启动替代的功能服务；导航和持续功能继续启动 | `--selected-pose 1 --no-sum` |
| `--no-micro-ros` | 跳过 Agent，其他入口照常启动 | `--selected-pose 1 --no-micro-ros` |
| `--headless` | 在当前终端管理子进程；功能端用 `preset_nav_node` 仅提供服务，不显示菜单 | `--selected-pose 1 --headless` |
| `--workspace 路径` | 默认自动定位工程；必要时指定工程根目录 | `--workspace .` |
| `--dry-run` | 选择场地并打印命令，不启动进程 | `--selected-pose 4 --dry-run` |
| `-h` / `--help` | 查看帮助 | `--help` |

常用组合：

```bash
ros2 run configuration main_boot --selected-pose 4
ros2 run configuration main_boot --selected-pose 4 --no-sum
ros2 run configuration main_boot --selected-pose 1 --no-micro-ros
```

默认多终端模式在各终端分别停止服务；先退出功能任务，再停止导航。`--headless` 模式由总启动负责进程组清理；可以同时加 `--no-sum` 完全不运行功能服务。跳过 `sum` 也会跳过它内部的速度转发。

`--headless` 只改变进程与终端的管理方式；当前导航 launch 仍会启动 RViz，因此该选项本身不会关闭图形界面。

### 2.5 日常修改与生效确认

普通代码、参数修改使用增量构建即可，无需删除 `build/`、`install/`、`log/`。先保存文件，停止受影响的旧节点，再按下表操作；新增功能文件后也需要构建。

| 修改内容 | 生效步骤 |
|---|---|
| `src/config/` 中已登记的中央配置 | `sync --files ...` 更新源码目标和已有安装配置，再重启相应节点 |
| 点位、任务参数、`modes/*.yaml`、按需任务 Python 代码 | 构建 `framework`，重启 `sum` |
| `src/function/detail/continuous/micro_ros/config/boot.yaml` 或 Agent 启动代码 | 构建 `micro_ros`，重启 Agent |
| 导航 launch、地图、URDF | 构建 `fishbot_navigation2`，重启导航 |
| odometry C++ 代码、总启动或工具代码 | 构建对应的 `odometry`、`configuration` 或 `tool` 包，重启对应入口 |

也可以统一执行完整的增量构建，随后按需同步中央配置：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
# 仅在修改了该中央配置时执行；名称按实际修改的文件替换。
ros2 run chairman_config sync --files fishbot_navigation2/nav2_params.yaml
ros2 run configuration main_boot
```

`colcon build` 不会代替 `sync` 将中央配置复制到实际使用位置，也不会重启旧节点。ROS2 读取磁盘配置和进程中的参数；VS Code 中未保存的编辑内容不会参与构建或加载。

需要确认 Nav2 参数是否用上新值时，先检查当前终端找到的安装位置，并比较配置文件：

```bash
ros2 pkg prefix fishbot_navigation2
diff -u \
  src/config/fishbot_navigation2/nav2_params.yaml \
  "$(ros2 pkg prefix fishbot_navigation2)/share/fishbot_navigation2/config/nav2_params.yaml" \
  && echo "中央配置与安装配置一致"
```

路径应指向本工程的 `install/`。重启导航、等待节点就绪后，可在另一个已加载本工程环境的终端读取实际运行值：

```bash
ros2 param get /controller_server FollowPath.v_linear_max
```

检查基础值时先不执行按需任务；任务模式可能临时覆盖基础值。若安装文件一致而运行值不同，检查启动参数覆盖、当前模式和是否仍有旧节点运行。点位或新功能则通过重启后的 `sum` 菜单确认。

## 3. sum.py 使用说明

正常使用由 `main_boot` 自动打开；单独调试时：

```bash
ros2 run framework sum
```

没有自定义命令行参数，不接受 `--selected-pose`、`--call` 等选项。场地来自启动环境 `SELECTED_POSE`；未设置时独立运行缺省为场地 1。`main_boot` 只能把环境传给它启动的子进程，不能改变另一个已经打开的终端。若跳过自动 `sum` 后在另一个终端调试，应显式沿用同一场地，例如 `SELECTED_POSE=4 ros2 run framework sum`。

界面显示“定点导航”和“特殊功能”两组编号及说明。输入编号执行，结束后打印结果并回到菜单；-9 和 16 再提示输入偏置 `x y z`，单位米。执行时 Ctrl+C 中止本次任务，等待停止和参数恢复后返回菜单；菜单中 `q` 或 Ctrl+C 退出。某些任务需要外部结束条件：登阶等待 `/nav_topic=0`，抬升检测等待高度达到阈值。

高级调试可使用 ROS2 标准参数，例如 `ros2 run framework sum --ros-args -p max_vel_linear:=0.3`。这属于本次进程参数，不是场地或功能选择参数。不要同时启动两个 `sum`，也不要与仅服务入口 `ros2 run framework preset_nav_node` 同时运行。

完整菜单示例、任务接口及添加功能步骤见 [功能开发说明](docs/function_development.md)。

## 4. sync.py 使用说明

先改中央副本，再选一种覆盖操作：

```bash
# 全覆盖：覆盖 manifest.json 登记的全部配置
ros2 run chairman_config sync --all
# 指定一个或多个文件覆盖
ros2 run chairman_config sync --files fast_lio/mid360.yaml
ros2 run chairman_config sync --files fishbot_navigation2/nav2_params.yaml odometry/regions.yaml
# 帮助
ros2 run chairman_config sync --help
```

| 参数 | 说明 |
|---|---|
| `--all` | 覆盖所有已登记配置 |
| `--files 文件1 [文件2 ...]` | 名称相对于 `src/config/`，多个名称用空格分隔 |
| `-h` / `--help` | 查看帮助 |

`--all` 和 `--files` 必须二选一。没有内容校验、预览或交互选择模式。原文件自动备份到 `.configuration_backups/`；更新本工程源码和已有安装副本后，需要重启相关节点。临时任务模式不通过 `sync` 写入 Nav2 文件。

## 5. tool 工具使用

### 获取初始位姿

```bash
ros2 run tool get_init_pose
```

无自定义参数。订阅 `/initialpose`；在 RViz 用 **2D Pose Estimate** 点选位置后，终端输出 `INIT_X/INIT_Y/INIT_Z/INIT_W`。其中 Z/W 是四元数 qz/qw，Z 不是高度。将值写入 `src/config/odometry/initial_poses.yaml` 对应场地，再运行 `sync --files odometry/initial_poses.yaml`。

### 获取目标点

```bash
ros2 run tool get_pose
```

无自定义参数。订阅 `/goal_pose`；在 RViz 用 **2D Nav Goal** 点选后输出 `GOAL_X/GOAL_Y/GOAL_Z/GOAL_W`。把 `[x,y,qz,qw]` 写入 `src/function/config/on_demand/points.yaml`。此工具只监听，但 RViz 发布的导航目标可能同时被正在运行的导航节点接收。

### 地图降采样

```bash
ros2 run tool downsample_map input.pcd output.pcd --voxel-size 0.1
```

`input.pcd` 是现有输入文件；`output.pcd` 是输出文件，父目录须存在；`--voxel-size` 是体素边长，单位米，默认 0.1，必须为正有限数。相对路径以当前终端目录为准，输入输出不能相同。需要 Open3D。`--help` 可查看用法。原始地图路径可使用 `src/fishbot/fishbot_navigation2/PCD/test.pcd`。

## 6. 新功能、新参数从哪里加

| 增加内容 | 文件位置/操作 |
|---|---|
| 普通固定点 | `src/function/config/on_demand/points.yaml` 中增加点位 |
| 速度模式 | `src/function/config/on_demand/modes/` 新建唯一 id 的 YAML |
| 特殊功能/多点流程 | `src/function/detail/on_demand/special/<名称>/task.py` 实现 `run(ctx, request)`，在 `functions.yaml` 登记 |
| 持续功能 | `src/function/detail/continuous/` 下增加独立 ROS 包 |
| 某任务的新参数 | 在 `functions.yaml` 的该任务 `parameters` 下增加字段，任务代码从 `ctx.config` 读取 |
| 新基础配置文件 | 在 `src/config/` 增加中央副本，并在 `manifest.json` 登记实际目标路径；目标包也须安装和加载该文件 |

新增 YAML 或 Python 模块后重新构建对应包，再重启相关进程。步骤和可复制模板分别见 [配置说明](docs/parameter_modification.md) 与 [功能开发说明](docs/function_development.md)。
