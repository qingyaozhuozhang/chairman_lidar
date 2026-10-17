# chairman_lidar

面向 BR 建筑机器人的 ROS 2 实车定位与导航工程，集成 Livox MID360、FAST-LIO、small_gicp 地图配准和 Nav2 全向运动控制。工程统一管理场地选择、设备通信、导航启动、按需任务及配置同步。

定位链路由 `small_gicp_relocalization_node` 根据所选启动点初始化，对实时点云与点云地图进行配准，并持续发布 Nav2 使用的 `map → odom`。地图、初始位姿、底盘限制应与实际机器人及场地一致。

| 文档 | 内容 |
|---|---|
| [参数修改说明](docs/parameter_modification.md) | 参数配置位置、字段含义、同步命令和生效方式 |
| [功能开发说明](docs/function_development.md) | 功能服务接口、新增点位和任务、持续以及按需调用功能 |
| [常用命令](docs/regular_commands.md) | 分终端启动、记录和查看运行日志 |
| [功能模板](docs/templates/) | 按需调用功能、点位序列、速度模式和持续调用功能模板 |

## 1. 整个代码框架

下文路径均相对于 `chairman_lidar/`，命令在包含 `src/`、`configuration/` 和 `tool/` 的工程根目录执行。

```text
chairman_lidar/
├── src/
│   ├── config/                         # ROS 包 chairman_config：中央参数配置服务（重要参数）
│   │   ├── framework/on_demand/        # 点位、特殊任务和速度模式配置
│   │   ├── fast_lio/                   # MID360 输入、滤波和安装外参
│   │   ├── fishbot_navigation2/         # Nav2 启动基准与 RViz 配置
│   │   ├── livox_ros_driver2/           # 雷达网络配置
│   │   ├── odometry/                   # 场地初始位姿与区域边界
│   │   ├── small_gicp_relocalization/   # 点云地图坐标变换、配准与 TF 参数
│   │   ├── manifest.json               # 中央配置到功能包文件的映射
│   │   └── sync.py                     # 参数同步配置文件
│   ├── fishbot/                        # 驱动、定位、导航及消息接口
│   │   ├── fishbot_navigation2/launch/navigation2.launch.py
│   │   ├── fast_lio/                   # 高精度、实时里程计与建图算法
│   │   ├── livox_ros_driver2/          # livox_mid360雷达驱动
│   │   ├── small_gicp_relocalization/   # gicp点云配准算法
│   │   ├── pb_omni_pid_pursuit_controller/ #全向轮底盘控制器
│   │   ├── pb_nav2_plugins/            # Nav2导航扩展插件包
│   │   ├── pointcloud_to_laserscan/    # 三维点云数据映射成二维点云数据
│   │   ├── terrain_analysis/           # 局部代价地图点云分析
│   │   ├── terrain_analysis_ext/       # 全局代价地图点云分析
│   │   └── custom_msg/                 # 消息通信协议
│   └── function/                       
│       ├── framework/                  # ROS 包 framework
│       │   ├── sum.py                  # 功能菜单、进程入口
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
├── configuration/                      
│   ├── main_boot.py                    # 总启动文件
│   └── tests/
├── tool/                               # 位姿读取、点位标定与地图降采样
├── docs/                               # 参数说明、功能开发与模板
```

## 2. 从构建到总启动

### 2.1 获取源码与准备环境

已验证的构建环境为 **Ubuntu 22.04 x86_64、ROS 2 Humble、Python 3.10**。先按 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 准备 ROS 2；克隆源码前还需要 Git，未安装时执行 `sudo apt install git`。

```bash
# 首先将仓库克隆到本地
git clone git@github.com:qingyaozhuozhang/chairman_lidar.git
```

以下依赖需准备：

| 依赖 | 准备方式 | 本工程中的用途 |
|---|---|---|
| Livox SDK2 | 按 [官方 C++ 安装说明](https://github.com/Livox-SDK/Livox-SDK2#2-installation) 编译并安装共享库和头文件 | `livox_ros_driver2` 的编译与运行依赖 |
| small_gicp | 按 [官方 C++ 安装说明](https://github.com/koide3/small_gicp#installation) 安装头文件；仅安装 Python 包不能替代 C++ 依赖 | 点云地图配准所需的 C++ 依赖；应确保 CMake 能找到头文件与依赖库 |
| micro-ROS Agent | 源码位于 `src/function/detail/continuous/micro_ros/src/`，随根目录 `colcon build` 一起构建 | 向匹配固件的下位机提供通信；首次编译需联网下载 Micro-XRCE-DDS-Agent 等构建依赖 |

**注：**
- 若无需用到`micro_ros`这个与下位机通信方式，可以不做相关处理（`micro_ros`的[参考资料](https://github.com/micro-ROS/micro_ros_setup/tree/humble)）

- `Livox SDK2`本地配置教程可参考[网站说明](https://qingyaozhuozhang.github.io/navigation/lidar/Mid-360%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E9%85%8D%E7%BD%AE%E6%95%99%E7%A8%8B/)

```bash
# 教程：
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make
sudo make install
```
- `small_gicp`功能包本地配置教程可参考[网站说明](https://github.com/koide3/small_gicp)
```bash
# 教程:
sudo apt install -y libeigen3-dev libomp-dev

git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

- 暂不连接下位机时可用 `ros2 run configuration main_boot --no-micro-ros` 跳过 Agent。

### 2.2 构建项目

依赖准备完成后，在工程根目录执行：

```bash
colcon build
source install/setup.bash
```

如果首次构建停在 `micro_ros_agent:build 6%`，查看 `log/latest_build/micro_ros_agent/stdout_stderr.log`。`Performing download step (git clone) for 'xrceagent'` 表示正在从 GitHub 下载依赖；连接超时会自动重试，需恢复网络后重新构建。成功下载和编译后，保留 `build/` 即可复用依赖缓存。

目前共 19 个包（包含 micro-ROS Agent、消息包和 setup 工具）。新终端需要重新 `source install/setup.bash`。

### 2.3 首次运行前确认参数

| 项目 | 编辑位置 | 需要确认的内容 |
|---|---|---|
| MID360 网络 | `src/config/livox_ros_driver2/MID360_config.json` | 雷达 IP、主机接收 IP，与实际网卡设置一致 |
| 雷达与 IMU 配置 | `src/config/fast_lio/mid360.yaml` | 输入话题、滤波参数、安装外参 |
| Nav2 基础配置 | `src/config/fishbot_navigation2/nav2_params.yaml` | 机器人尺寸、控制器、代价地图等参数 |
| 建图点、场地初始位姿与区域 | `src/config/odometry/initial_poses.yaml`、`src/config/odometry/regions.yaml` | PCD 建图原点、4 个场地的启动位姿及红蓝区域边界 |
| GICP 地图降采样 | `tool/downsample_map.py` | `GLOBAL_LEAF_SIZE` 为离线地图体素边长，默认 0.05 m；处理 `base.pcd` 生成 `final.pcd` |
| 定点与任务模式 | `src/config/framework/on_demand/` | 红蓝点位、特殊任务、三种速度模式及特殊运动控制参数 |
| 下位机通信 | `src/function/detail/continuous/micro_ros/config/boot.yaml` | 默认使用本项目 Agent；UDP IPv4 监听端口 8888 及可选外部工作空间 |
| 地图和机器人模型 | `src/config/fishbot_navigation2/rviz`和`src/fishbot/fishbot_navigation/urdf` | 当前加载 `src/fishbot/fisbot_navigation/maps/room.yaml`、`src/fishbot/fishbot_navigation2/PCD/final.pcd`；核对 URDF、TF 安装偏移和扫描范围 |

**注：**
- 凡是在中央参数配置文件`src/config`进行操作的都需要使用`sync`文件进行同步（具体参数文件使用说明见[参数修改说明](docs/parameter_modification.md)）

例如：按设备修改 `src/config/` 的雷达 IP、里程计初始位姿和 Nav2 基础参数，然后同步

```bash
ros2 run chairman_config sync --files livox_ros_driver2/MID360_config.json odometry/initial_poses.yaml
```
- 点云存放于[gitee仓库](https://gitee.com/chairman11/pcd-point-cloud-repository)

### 2.4 项目总启动入口
运行`main_boot`文件

```bash
ros2 run configuration main_boot
```

当前终端询问：

```bash
初始位置 1红武馆 / 2红对抗 / 3蓝武馆 / 4蓝对抗:
```

输入 1～4。`main_boot.py` 将选择通过 `SELECTED_POSE` 传给每个子进程，默认分别打开终端：

| 终端 | 实际入口 | 职责 |
|---|---|---|
| Micro ROS Agent | `ros2 run micro_ros agent` | UDP IPv4 通信，默认监听 8888 端口 |
| Navigation2 | `ros2 launch fishbot_navigation2 navigation2.launch.py` | 雷达、FAST-LIO、地形分析、Nav2、RViz 等导航服务 |
| Function sum | `ros2 run framework sum` | 功能选择与执行 |
| Odometry | `ros2 run odometry odometry` | 地图位姿输出与区域检测 |

导航 launch 读取 odometry 包内的 `initial_poses.yaml`：`mapping_pose` 保存 PCD 建图原点在 `map` 中的位姿，转换为固定的 `map_to_pcd`；`poses` 中的 1～4 号保存底盘启动位姿，由 `SELECTED_POSE` 选择。定位节点等待本次 FAST-LIO 的局部里程计 TF，结合两组位姿计算配准初值，同时发布初始 `map → odom`。首次配准成功后，以有效匹配结果持续修正该变换。同一份 PCD 切换启动点时，`mapping_pose` 保持不变。

建图点初值采用 1 号点，定义 PCD 的零位姿。选择其他启动点时，先计算它相对建图点的位置与朝向用于初始化配准；输出端以所选启动点为地图参考，将 GICP 结果转换到该参考系后映射回 `map`。建图点独立保存，修改 1～4 号启动点不会自动改变地图原点。

`small_gicp_relocalization_node` 是 `map → odom` 的唯一发布者，默认以 20 Hz 发布，时间戳为当前 ROS 时间加 0.1 s。Navigation2 终端中的 `[定位初始化]` 表示初始 TF 已建立；出现 `[定位就绪] GICP 配准成功` 才表示首次匹配成功，确认 RViz 中的位置和朝向正确后再执行导航。odometry 默认不自动发布 `/initialpose`；RViz 的 `2D Pose Estimate` 可手动重新初始化 GICP。

导航服务统一调用 `src/fishbot/fishbot_navigation2/launch/navigation2.launch.py`。`sum` 只使用已选场地对应的红/蓝点位和区域，不弹出第二次场地选择。

`main_boot` 的全部参数：

| 参数 | 默认行为/作用 | 示例 |
|---|---|---|
| 不加参数 | 交互选择场地，再启动上述服务 | `ros2 run configuration main_boot` |
| `--selected-pose 1\|2\|3\|4` | 直接指定场地，跳过场地提示 | `--selected-pose 4` |
| `--no-sum` | 不启动功能菜单；导航和持续功能继续启动 | `--selected-pose 1 --no-sum` |
| `--no-micro-ros` | 跳过 micro_ros的Agent，其他入口照常启动 | `--selected-pose 1 --no-micro-ros` |
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

### 2.5 日常修改与生效确认

普通代码、参数修改使用增量构建即可，无需删除 `build/`、`install/`、`log/`。

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
colcon build
source install/setup.bash
# 仅在修改了该中央配置时执行；名称按实际修改的文件替换。
ros2 run chairman_config sync --files fishbot_navigation2/nav2_params.yaml
ros2 run configuration main_boot
```

`colcon build` 不会代替 `sync` 将中央配置复制到实际使用位置，也不会重启旧节点。

需要确认 Nav2 参数是否用上新值时，启动导航、等待节点就绪后，可在另一个已加载本工程环境的终端读取实际运行值：

```bash
ros2 param get /controller_server FollowPath.v_linear_max
```

## 3. sum.py 使用说明

正常使用由 `main_boot` 自动打开；单独调试时：

```bash
ros2 run framework sum
```

没有自定义命令行参数，不接受 `--selected-pose`、`--call` 等选项。场地来自启动环境 `SELECTED_POSE`；未设置时独立运行缺省为场地 1。`main_boot` 只能把环境传给它启动的子进程，不能改变另一个已经打开的终端。若跳过自动 `sum` 后在另一个终端调试，应显式沿用同一场地，例如 `SELECTED_POSE=4 ros2 run framework sum`。


功能运行过程：三种速度模式统一输出任务开始与最终完成、失败或中止结果；运行过程在同一行更新阶段、误差和耗时，最多每秒刷新 5 次。状态条表示任务仍在执行，不表示完成百分比。阶段切换更新状态条，整次任务结束并完成收尾后自动重新显示完整菜单；重定向到文件时不输出状态条。

| 模式 | 配置文件 | 执行过程 |
|---|---|---|
| 1 | `speed_1.yaml` | 使用基础参数导航至目标 |
| 2 | `speed_2.yaml` | 远处使用冲刺组，进入 `switch_distance` 后切换精调组 |
| 3 | `speed_3.yaml` | 先执行提前点或原地朝向调整，确认第一段结束后进入真实目标 |
| 4 | `special.yaml` | 特殊功能参数 |

模式配置位于 `src/config/framework/on_demand/modes/`，使用与 `nav2_params.yaml` 一致的节点名称及 `ros__parameters` 层级。每个模式独立定义参数和流程设置，点位的 `mode` 对应文件中的 `id`。模式 2 的 `sprint_parameters` 与 `parameters` 对应两个阶段，参数名相同、数值独立。任务期间通过参数服务临时赋值，结束后恢复运行前的值；磁盘 `nav2_params.yaml` 保留启动基准。

完整菜单示例、任务接口及添加功能步骤见 [功能开发说明](docs/function_development.md)。

## 4. sync.py 使用说明

选择一种覆盖操作：

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

`--all` 和 `--files` 必须二选一。工具校验映射和路径范围，不检查参数语义，也不提供预览或交互选择模式。原文件自动备份到 `.configuration_backups/`。

## 5. tool 工具使用

### 获取初始位姿

```bash
ros2 run tool get_init_pose
```

无自定义参数。订阅 `/initialpose`；在 RViz 用 **2D Pose Estimate** 点选位置后，终端输出 `INIT_X/INIT_Y/INIT_Z/INIT_W`。其中 Z/W 是四元数 qz/qw，Z 不是高度。将值写入 `src/config/odometry/initial_poses.yaml` 对应场地，再运行 `ros2 run chairman_config sync --files odometry/initial_poses.yaml`。

### 获取目标点

```bash
ros2 run tool get_pose
```

无自定义参数。订阅 `/goal_pose`；在 RViz 用 **2D Nav Goal** 点选后输出 `GOAL_X/GOAL_Y/GOAL_Z/GOAL_W`。把 `[x,y,qz,qw]` 写入 `src/config/framework/on_demand/points.yaml`，同步后重启 Function sum。此工具只监听，但 RViz 发布的导航目标可能同时被正在运行的导航节点接收。

### 实车快速标定点位

```bash
ros2 run tool point
```

先启动导航和 `odometry`，等待 GICP 匹配成功并核对 RViz 地图位姿。在菜单中选择区域、固定点或启动点，将机器人停稳后按回车，即采集并保存当前位姿。区域/固定点需选择红蓝半场和编号；启动点先选菜单 `3`，再选启动编号 1～4，并将车头调整到实际启动朝向。可连续选择下一点，无需重启工具。

工具直接订阅 `/odom_map`（`custom_msg/msg/PoseEuler`），不读取 `/rosout` 日志，也不把 FAST-LIO 原始 `/Odometry` 坐标直接写成地图点位。每次取确认后收到的 **10 条有效消息**，x/y/z 分别去掉一个最大值和一个最小值，用剩余 8 个求平均；yaw 先处理 ±π 环绕再去极值。固定点和区域只使用平均 x/y，启动点还将平均 yaw 转成 qz/qw 写入配置。采集速度取决于实际话题频率（例如 10 Hz 约 1 秒），不会等待日志节流周期。默认 5 秒超时，消息不足时不写入。

| 模式 | 写入位置 | 修改内容与生效方式 |
|---|---|---|
| 固定点 | `src/config/framework/on_demand/points.yaml` 与 `src/function/config/on_demand/points.yaml` | 两份文件中同一半场、同一编号的 x/y 同时更新，保留各自的 qz/qw、名称与模式；15/16 为动态目标。|
| 区域中心 | `src/config/odometry/regions.yaml` 与 `src/function/detail/continuous/odometry/config/regions.yaml` | 两份文件中同一半场、同一编号的正方形边界同时更新为中心±0.6 m。|
| 启动点 | `src/config/odometry/initial_poses.yaml` 与 `src/function/detail/continuous/odometry/config/initial_poses.yaml` | 同时更新所选编号的 `lidar_init_x/y/qz/qw`，保留其他启动点和注释。构建 `odometry` 后，重启 Navigation2 与 odometry。|

标定只写两份源码配置，不写 `install`，不创建备份目录。启动点采集依赖正确的地图定位：可从已能正确定位的位置启动，再移动到待记录的启动点。尚未配准成功时，显示的初始估计不能用于验证或重新标定自身。

工具根据自身路径定位工程，也可用 `--workspace /项目根目录` 指定；`--timeout 10` 调整超时，`--topic /odom_map` 指定同类型的 map 位姿话题。`POINT_WORKSPACE` 环境变量也可指定工程。

修改工具代码后执行 `colcon build --packages-select tool`、`source install/setup.bash`。也可在已加载 ROS 环境的终端直接运行 `python3 tool/point.py`。

### 地图降采样

在工程根目录运行：

```bash
python3 tool/downsample_map.py
colcon build --packages-select fishbot_navigation2
source install/setup.bash
```

默认读取 `src/fishbot/fishbot_navigation2/PCD/base.pcd`，生成同目录的 `final.pcd`。`base.pcd` 保留原始地图，导航 launch 加载安装目录中的 `PCD/final.pcd`；原始地图不安装到运行包。更新地图后需重新构建 `fishbot_navigation2` 并重启 Navigation2。

`tool/downsample_map.py` 中的 `GLOBAL_LEAF_SIZE = 0.05` 是地图体素边长，单位 m，也可用 `--global-leaf-size 0.05` 临时指定。脚本使用固定在 PCD 原点的格网，每个占用体素保留全部有效点的 XYZ 质心，不改变地图坐标系。依赖 NumPy 和 Open3D；Open3D 用于 PCD 读写。可通过 `ros2 run tool downsample_map` 调用已构建安装的工具，或显式指定其他文件：

```bash
python3 tool/downsample_map.py input.pcd output.pcd --global-leaf-size 0.05
```

自定义路径相对于当前终端目录，输入输出必须不同，输出目录须存在。输出为二进制 XYZ PCD，并在文件头注释 `chairman_lidar_voxel_size` 中记录体素边长；GICP 使用 XYZ 并重新计算协方差，不使用原始文件中的 intensity、normal 或 curvature 字段。脚本写入完成后才替换输出文件，输入地图始终保留。

GICP 直接使用离线地图，不再对地图降采样。FAST-LIO 保持原有输出；GICP 从 `final.pcd` 文件头读取体素边长，用相同尺度与每格质心规则处理实时点云，再进行配准，不需要单独配置实时体素参数。地图读取、搜索树建立和协方差计算仍需执行，实际耗时可查看 `[地图就绪]` 日志。离线降采样不改变 `mapping_pose` 或启动点的坐标含义。

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
