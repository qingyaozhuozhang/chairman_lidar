# chairman_lidar

面向 BR 建筑机器人的 ROS 2 实车定位与导航工程，集成 Livox MID360、FAST-LIO、small_gicp 地图配准和 Nav2 全向运动控制。工程统一管理场地选择、设备通信、导航启动、按需任务及配置同步。

固定点导航提供基础、冲刺/精调、提前对齐三种流程；特殊任务覆盖闭环移动、旋转、区域对齐、登阶和抬升检测。任务通过菜单或 ROS 服务调用，执行结束后完成停车和临时参数恢复。

定位链路由 `small_gicp_relocalization_node` 对实时点云与点云地图进行配准，发布 `map1 → odom1`；`tf_relay_node` 将该变换转发为 Nav2 使用的 `map → odom`。地图、初始位姿、传感器外参和底盘限制应与实际机器人及场地一致。

| 文档 | 内容 |
|---|---|
| [参数修改说明](docs/parameter_modification.md) | 配置位置、字段含义、同步命令和生效方式 |
| [功能开发说明](docs/function_development.md) | 菜单与服务接口、代码阅读顺序、新增点位和任务 |
| [功能模板](docs/templates/) | 按需任务、点位序列、速度模式和持续节点模板 |

## 1. 整个代码框架

下文路径均相对于 `chairman_lidar/`，命令在包含 `src/`、`configuration/` 和 `tool/` 的工程根目录执行。

```text
chairman_lidar/
├── src/
│   ├── config/                         # ROS 包 chairman_config：中央配置与同步工具
│   │   ├── framework/on_demand/        # 点位、特殊任务和速度配置
│   │   ├── fast_lio/                   # MID360 输入、滤波和安装外参
│   │   ├── fishbot_navigation2/         # Nav2 启动基准与 RViz 配置
│   │   ├── livox_ros_driver2/           # 雷达网络配置
│   │   ├── odometry/                   # 场地初始位姿与区域边界
│   │   ├── manifest.json               # 中央配置到功能包文件的映射
│   │   └── sync.py                     # 指定文件或全部配置同步
│   ├── fishbot/                        # 驱动、定位、导航及消息接口
│   │   ├── fishbot_navigation2/launch/navigation2.launch.py
│   │   ├── fast_lio/
│   │   ├── livox_ros_driver2/
│   │   ├── small_gicp_relocalization/   # 点云配准与 C++ TF 转发节点
│   │   ├── pb_omni_pid_pursuit_controller/
│   │   ├── pb_nav2_plugins/
│   │   ├── pointcloud_to_laserscan/
│   │   ├── terrain_analysis/
│   │   ├── terrain_analysis_ext/
│   │   └── custom_msg/
│   └── function/
│       ├── framework/                  # ROS 包 framework
│       │   ├── sum.py                  # 菜单、单行状态条与进程入口
│       │   └── core/                   # 公共 ROS 通信
│       │       ├── communication.py    # 话题、服务、位姿查询和速度输出
│       │       ├── actions.py          # Action 发送、取消与终态确认
│       │       └── parameters.py       # 参数服务、原值保存与恢复确认
│       ├── detail/
│       │   ├── on_demand/              # Python 包 chairman_tasks
│       │   │   ├── task.py             # 请求调度、任务互斥与 TaskContext
│       │   │   ├── config.py           # 配置加载与校验
│       │   │   ├── fixed_point/task.py # 三种导航模式及参数应用
│       │   │   └── special/            # 特殊任务入口与控制实现
│       │   └── continuous/            # 独立运行的持续功能
│       │       ├── odometry/           # /odom_map 发布与区域检测
│       │       └── micro_ros/
│       │           ├── launcher/       # ROS 包 micro_ros 的构建文件
│       │           ├── agent.py        # Agent 启动入口
│       │           ├── config/boot.yaml
│       │           └── src/            # Agent、消息和 setup 源码包
│       └── config/on_demand/           # framework 构建、安装使用的配置副本
├── configuration/                      # 场地选择与总启动
│   ├── main_boot.py
│   └── tests/
├── tool/                               # 位姿读取、点位标定与地图降采样
├── docs/                               # 参数说明、功能开发与模板
├── build/                              # 构建产物
├── install/                            # ROS 包安装结果
└── log/                                # 构建日志；robot/ 保存运行输出
```

功能代码按 `framework/sum.py → detail/on_demand/task.py → 具体功能/task.py` 的顺序阅读。`framework/core` 提供共享通信能力；任务路由、配置解释和运动流程位于 `detail/on_demand`。`continuous` 使用独立节点持续处理数据。

配置分为磁盘配置与运行参数：`sync` 将中央文件同步至包内和已有安装副本；导航模式通过 ROS 参数服务临时更新正在运行的 Nav2，任务结束后恢复执行前的值。总启动的运行输出位于 `log/robot/`，其中 `Log/` 和 `PCD/` 接收算法生成的文件。

## 2. 从构建到总启动

### 2.1 获取源码与准备环境

已验证的构建环境为 **Ubuntu 22.04 x86_64、ROS 2 Humble、Python 3.10**。先按 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 准备 ROS 2；克隆源码前还需要 Git，未安装时执行 `sudo apt install git`。

以下克隆命令按 `chairman_lidar/` 本身作为仓库根目录编写。在 GitHub 页面通过 **Code → HTTPS** 复制本仓库地址，运行时粘贴：

```bash
read -r -p "本仓库的 Git URL: " navigation_repo_url
git clone "$navigation_repo_url" chairman_lidar
cd chairman_lidar
```

后续命令均在包含 `src/`、`configuration/`、`tool/` 的工程根目录执行。如果源码位于一个更大的仓库中，先进入其中的 `chairman_lidar/` 子目录，避免同时发现其他工作空间中的同名包。

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
| small_gicp | 按 [官方 C++ 安装说明](https://github.com/koide3/small_gicp#installation) 安装头文件；仅安装 Python 包不能替代 C++ 依赖 | 点云地图配准所需的 C++ 依赖；应确保 CMake 能找到头文件与依赖库 |
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

Livox 驱动会自动选择当前 ROS 2 提供的消息接口构建方式，不需要先进入驱动目录运行 `./build.sh humble`，也不需要传 `-DHUMBLE_ROS=humble`。首次构建和后续构建都可在本目录执行 `colcon build`。

如果首次构建停在 `micro_ros_agent:build 6%`，查看 `log/latest_build/micro_ros_agent/stdout_stderr.log`。`Performing download step (git clone) for 'xrceagent'` 表示正在从 GitHub 下载依赖；连接超时会自动重试，需恢复网络后重新构建。成功下载和编译后，保留 `build/` 即可复用依赖缓存。

内存紧张时，可用 `MAKEFLAGS="-j2 -l2" colcon build --executor sequential` 限制同时编译的数量。开发时也可选择 `colcon build --symlink-install`；新增模块或配置文件后仍需构建，以便安装到对应包中。

`source /opt/ros/humble/setup.bash` 提供 ROS 2 基础环境；`source install/setup.bash` 让当前终端找到本工程的功能包。它们不会让运行中的节点重新加载代码或参数。若使用的 Python 虚拟环境缺少 ROS 构建模块，退出该虚拟环境后重新加载 ROS 环境再构建。

目前共 19 个包（包含 micro-ROS Agent、消息包和 setup 工具）。新终端需要重新 `source install/setup.bash`。迁移电脑时复制源码，重新安装依赖和构建；[.gitignore](.gitignore) 已排除 `build/`、`install/`、`log/`、`.configuration_backups/` 等本机生成内容。

### 2.3 首次运行前确认参数

| 项目 | 编辑位置 | 需要确认的内容 |
|---|---|---|
| MID360 网络 | `src/config/livox_ros_driver2/MID360_config.json` | 雷达 IP、主机接收 IP，与实际网卡设置一致 |
| 雷达与 IMU 配置 | `src/config/fast_lio/mid360.yaml` | 输入话题、滤波参数、安装外参 |
| Nav2 基础配置 | `src/config/fishbot_navigation2/nav2_params.yaml` | 机器人尺寸、控制器、代价地图等参数 |
| 场地初始位姿与区域 | `src/config/odometry/initial_poses.yaml`、`src/config/odometry/regions.yaml` | 4 个场地的初始位姿及红蓝区域边界 |
| 定点与任务模式 | `src/config/framework/on_demand/` | 红蓝点位、特殊任务、三种速度模式及手写控制参数 |
| 下位机通信 | `src/function/detail/continuous/micro_ros/config/boot.yaml` | 默认使用本项目 Agent；串口设备、波特率及可选外部工作空间 |
| 地图和机器人模型 | `src/fishbot/fishbot_navigation2/launch/navigation2.launch.py` 及其引用资源 | 当前加载 `maps/room.yaml`、`PCD/scans.pcd`；核对 URDF、TF 安装偏移和扫描范围 |

点云地图不随普通源码提交。迁移或克隆工程后，应另行准备对应场地的 `PCD/scans.pcd`，或修改 launch 引用的文件名，并构建导航包。地图文件的存在与坐标系应在启动前确认。

按设备修改 `src/config/` 的雷达 IP、里程计初始位姿和 Nav2 基础参数，然后同步，例如：

```bash
ros2 run chairman_config sync --files livox_ros_driver2/MID360_config.json odometry/initial_poses.yaml
```

点位、模式和特殊功能配置已登记到 `sync`；同步后重启 Function sum。micro-ROS 的 `boot.yaml` 由本包直接管理，修改后构建 `micro_ros` 并重启 Agent。修改地图、模型和导航 launch 后构建 `fishbot_navigation2`。具体文件、修改示例和生效步骤见 [参数修改说明](docs/parameter_modification.md)。

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
| Odometry | `ros2 run odometry odometry` | 地图位姿输出、初始位姿发布与区域检测 |

导航 launch 同时启动 C++ 节点 `tf_relay_node`：源 TF 就绪前发布配置的初始变换，之后以 20 Hz 转发最近一次 `map1 → odom1`，输出时间戳为当前 ROS 时间加 0.1 s。独立入口为 `ros2 run small_gicp_relocalization tf_relay_node`；总导航已启动时无需重复运行。

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
| 点位、任务参数、`modes/*.yaml` 中已登记的中央配置 | 同步 `framework/on_demand/...`，重启 Function sum |
| 按需任务 Python 代码或新增模式文件 | 构建 `framework`，重启 Function sum |
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

`colcon build` 不会代替 `sync` 将中央配置复制到实际使用位置，也不会重启旧节点。ROS 2 读取磁盘配置和进程中的参数；VS Code 中未保存的编辑内容不会参与构建或加载。

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

三种模式统一输出任务开始与最终完成、失败或中止结果；运行过程在同一行更新阶段、误差和耗时，最多每秒刷新 5 次。状态条表示任务仍在执行，不表示完成百分比。阶段切换更新状态条，整次任务结束并完成收尾后自动重新显示完整菜单；重定向到文件时不输出状态条。

| 模式 | 配置文件 | 执行过程 |
|---|---|---|
| 1 | `speed_1.yaml` | 使用基础参数导航至目标 |
| 2 | `speed_2.yaml` | 远处使用冲刺组，进入 `switch_distance` 后切换精调组 |
| 3 | `speed_3.yaml` | 先执行提前点或原地朝向调整，确认第一段结束后进入真实目标 |

模式配置位于 `src/config/framework/on_demand/modes/`，使用与 `nav2_params.yaml` 一致的节点名称及 `ros__parameters` 层级。模式 2 的 `sprint_parameters` 与 `parameters` 对应两个阶段，参数名相同、数值独立。`special.yaml` 在同目录配置手写运动，不作为导航模式编号加载。

高级调试可使用 ROS 2 标准参数，例如 `ros2 run framework sum --ros-args -p controller_server.FollowPath.v_linear_max:=0.3`。这属于本次进程参数，不是场地或功能选择参数。不要同时启动两个 `sum`，也不要与仅服务入口 `ros2 run framework preset_nav_node` 同时运行。

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

`--all` 和 `--files` 必须二选一。工具校验映射和路径范围，不检查参数语义，也不提供预览或交互选择模式。原文件自动备份到 `.configuration_backups/`；更新本工程源码和已有安装副本后，需要重启相关节点。同步模式文件只更新其磁盘配置；执行任务时才向 Nav2 下发运行参数，结束后恢复原值。

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

无自定义参数。订阅 `/goal_pose`；在 RViz 用 **2D Nav Goal** 点选后输出 `GOAL_X/GOAL_Y/GOAL_Z/GOAL_W`。把 `[x,y,qz,qw]` 写入 `src/config/framework/on_demand/points.yaml`，同步后重启 Function sum。此工具只监听，但 RViz 发布的导航目标可能同时被正在运行的导航节点接收。

### 实车快速标定点位

```bash
ros2 run tool point
```

先启动导航和 `odometry`，等待 map 位姿就绪。在菜单中选择区域/固定点、红蓝半场和编号，将机器人停稳后按回车，即采集并保存当前点位。可连续选择下一点，无需重启工具。

工具直接订阅 `/odom_map`（`custom_msg/msg/PoseEuler`），不读取 `/rosout` 日志，也不把 FAST-LIO 原始 `/Odometry` 坐标直接写成地图点位。每次取确认后收到的 **10 条有效消息**，x/y/z 分别去掉一个最大值和一个最小值，用剩余 8 个求平均；yaw 先处理 ±π 环绕再去极值。只把平均 x/y 写入配置，采集速度取决于实际话题频率（例如 10 Hz 约 1 秒），不会等待日志节流周期。默认 5 秒超时，消息不足时不写入。

| 模式 | 写入位置 | 修改内容与生效方式 |
|---|---|---|
| 固定点 | `src/config/framework/on_demand/points.yaml` | 修改 x/y，保留 qz/qw、名称与模式；15/16 为动态目标。完成后同步 `framework/on_demand/points.yaml` 并重启 Function sum |
| 区域中心 | `src/config/odometry/regions.yaml` | 选中区域改为中心±0.6 m；完成后执行 `ros2 run chairman_config sync --files odometry/regions.yaml`，重启 odometry 与功能进程 |

每次修改自动备份到配置所在目录的 `.point_backups/`，保留 YAML 其他字段和注释。默认只编辑本工程的源码配置，不直接修改运行中的参数或安装副本。工具根据自身路径定位工程，也可用 `--workspace /项目根目录` 指定；`--timeout 10` 调整超时，`--topic /odom_map` 指定同类型的 map 位姿话题。`POINT_WORKSPACE` 环境变量也可指定工程。

修改工具代码后执行 `colcon build --packages-select tool`、`source install/setup.bash`。也可在已加载 ROS 环境的终端直接运行 `python3 tool/point.py`。

### 地图降采样

```bash
ros2 run tool downsample_map input.pcd output.pcd --voxel-size 0.1
```

`input.pcd` 是现有输入文件；`output.pcd` 是输出文件，父目录须存在；`--voxel-size` 是体素边长，单位米，默认 0.1，必须为正有限数。相对路径以当前终端目录为准，输入输出不能相同。需要 Open3D。`--help` 可查看用法。原始地图路径可使用 `src/fishbot/fishbot_navigation2/PCD/scans.pcd`。

## 6. 新功能、新参数从哪里加

| 增加内容 | 文件位置/操作 |
|---|---|
| 普通固定点 | `src/config/framework/on_demand/points.yaml` 中增加点位，同步后重启功能进程 |
| 速度模式 | 在中央与包内 `modes/` 增加唯一 id 的 YAML，在 `manifest.json` 登记映射并构建 `framework` |
| 特殊功能/多点流程 | `src/function/detail/on_demand/special/<名称>/task.py` 实现 `run(ctx, request)`，在 `functions.yaml` 登记 |
| 持续功能 | `src/function/detail/continuous/` 下增加独立 ROS 包 |
| 某任务的新参数 | 在 `functions.yaml` 的该任务 `parameters` 下增加字段，任务代码从 `ctx.config` 读取 |
| 新基础配置文件 | 在 `src/config/` 增加中央副本，并在 `manifest.json` 登记实际目标路径；目标包也须安装和加载该文件 |

新增 YAML 或 Python 模块后重新构建对应包，再重启相关进程。步骤和可复制模板分别见 [配置说明](docs/parameter_modification.md) 与 [功能开发说明](docs/function_development.md)。
