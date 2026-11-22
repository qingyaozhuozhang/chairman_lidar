# 常用命令
### 一.这个用于记录终端信息
运行时通过 `tee` 同时显示、记录标准输出与错误输出；程序退出后，`EXIT` 处理将临时文件重命名为正式日志，并输出保存位置。日志统一存放在项目根目录的 `./run_log/`，与 `src/` 同级；项目移动到其他位置后无需修改命令中的路径。

## 1. Micro ROS Agent

```bash
bash -c '
set -o pipefail
[ -d src ] || { echo "请先进入 chairman_lidar 项目根目录"; exit 1; }
log_dir="./run_log"
mkdir -p "$log_dir" || exit 1
tmp=$(mktemp "$log_dir/.Micro_ROS_Agent_XXXXXX") || exit 1
trap '"'"'n=1; while [ -f "$tmp" ]; do log="$log_dir/Micro_ROS_Agent_${n}.log"; mv -nT -- "$tmp" "$log" || break; n=$((n+1)); done; [ ! -f "$tmp" ] && echo "日志已保存：$log"'"'"' EXIT
trap : INT
trap "exit 129" HUP
ros2 run micro_ros agent 2>&1 | tee -i "$tmp"
'
```

## 2. Navigation2

```bash
bash -c '
set -o pipefail
[ -d src ] || { echo "请先进入 chairman_lidar 项目根目录"; exit 1; }
log_dir="./run_log"
mkdir -p "$log_dir" || exit 1
tmp=$(mktemp "$log_dir/.Navigation2_XXXXXX") || exit 1
trap '"'"'n=1; while [ -f "$tmp" ]; do log="$log_dir/Navigation2_${n}.log"; mv -nT -- "$tmp" "$log" || break; n=$((n+1)); done; [ ! -f "$tmp" ] && echo "日志已保存：$log"'"'"' EXIT
trap : INT
trap "exit 129" HUP
ros2 launch fishbot_navigation2 navigation2.launch.py 2>&1 | tee -i "$tmp"
'
```

## 3. Function sum

```bash
bash -c '
set -o pipefail
[ -d src ] || { echo "请先进入 chairman_lidar 项目根目录"; exit 1; }
log_dir="./run_log"
mkdir -p "$log_dir" || exit 1
tmp=$(mktemp "$log_dir/.Function_sum_XXXXXX") || exit 1
trap '"'"'n=1; while [ -f "$tmp" ]; do log="$log_dir/Function_sum_${n}.log"; mv -nT -- "$tmp" "$log" || break; n=$((n+1)); done; [ ! -f "$tmp" ] && echo "日志已保存：$log"'"'"' EXIT
trap : INT
trap "exit 129" HUP
ros2 run framework sum 2>&1 | tee -i "$tmp"
'
```

## 4. Odometry

```bash
bash -c '
set -o pipefail
[ -d src ] || { echo "请先进入 chairman_lidar 项目根目录"; exit 1; }
log_dir="./run_log"
mkdir -p "$log_dir" || exit 1
tmp=$(mktemp "$log_dir/.Odometry_XXXXXX") || exit 1
trap '"'"'n=1; while [ -f "$tmp" ]; do log="$log_dir/Odometry_${n}.log"; mv -nT -- "$tmp" "$log" || break; n=$((n+1)); done; [ ! -f "$tmp" ] && echo "日志已保存：$log"'"'"' EXIT
trap : INT
trap "exit 129" HUP
ros2 run odometry odometry 2>&1 | tee -i "$tmp"
'
```

## 5. 保存与查看

每类终端独立使用 `1、2、3……` 序号，不使用日期。退出保存时从 1 开始查找未占用的序号，保留已有日志；`mv -nT` 遇到同名文件时跳过，继续尝试下一个序号。

```text
chairman_lidar/
├── src/
└── run_log/
    ├── Micro_ROS_Agent_1.log
    ├── Navigation2_1.log
    ├── Navigation2_2.log
    ├── Navigation2_3.log
    ├── Function_sum_1.log
    └── Odometry_1.log
```

- 正常退出或按 Ctrl+C 结束程序后，终端显示“日志已保存”。Function sum 在任务执行期间按 Ctrl+C 只中止任务；录制继续，回到菜单输入 `q` 才结束程序并保存。
- 关闭终端窗口时，通过 `HUP` 触发退出保存；窗口已经关闭，因此看不到保存提示，到 `run_log/` 查看即可。系统断电或强制杀进程无法执行退出处理，已写入的内容可能仍保留在该目录的隐藏临时文件中。
- `tee -i` 在 Ctrl+C 时继续读取输出，直到程序关闭输出流，避免遗漏正常退出过程中输出的日志。外层 Bash 的 `INT` 处理不主动结束菜单进程。
- 记录从本条命令启动后开始，包含程序实际输出的内容。管道输出时 Function sum 不绘制动态进度条，任务开始和结束等日志正常记录。

在项目根目录查看最近保存的日志：

```bash
ls -lt run_log/*.log
```

查看某次导航日志，替换为实际文件名：

```bash
less -R run_log/Navigation2_1.log
```

在 `less` 中按 `q` 返回终端。


点云裁减命令
# 只裁剪，不降采样
python3 tool/downsample_map.py base.pcd final.pcd \
  --no-downsample \
  --crop --crop-min-xy -1.65 -0.55 --crop-max-xy 4.5 11.5

# 只降采样（原行为）
python3 tool/downsample_map.py base.pcd final.pcd --voxel-size 0.05

# 既裁剪又降采样
python3 tool/downsample_map.py base.pcd final.pcd \
  --voxel-size 0.05 \
  --crop --crop-min-xy -1.65 -0.55 --crop-max-xy 4.5 11.5

保存点云地图
ros2 service call /map_save std_srvs/srv/Trigger

### 二.独立建图与点位地图

在工程根目录构建并加载环境后运行：

```bash
# 雷达 + FAST-LIO 建图；Ctrl+C 后保存 scans.pcd 并降采样生成 final.pcd
ros2 run configuration main_map
# 不打开建图 RViz
ros2 run configuration main_map --no-rviz
# 只检查命令与路径，不启动设备
ros2 run configuration main_map --dry-run
```

原始地图保存在 `src/fishbot/fast_lio/PCD/scans.pcd`，降采样结果保存在 `src/fishbot/fishbot_navigation2/PCD/final.pcd`。体素边长默认 0.05 m，可通过 `--global-leaf-size 0.05` 指定。降采样依赖 NumPy、Open3D，请使用已安装这些依赖的 Python 环境；按 Ctrl+C 后等待保存和降采样完成提示，再关闭终端。

```bash
# 更新安装目录中的导航地图
colcon build --packages-select fishbot_navigation2
source install/setup.bash
# 导航使用 Point-LIO
ros2 run configuration main_boot
# 根据当前源码点位生成彩色标注地图；无需启动导航
ros2 run tool show_points
# 指定另一张底图
ros2 run tool show_points --map src/fishbot/fishbot_navigation2/maps/room_zhengzhi.yaml
```

标注图片为 `src/function/config/on_demand/points_map/points_map.png`，同目录生成地图描述 `points_map.yaml`，再次运行覆盖生成结果。红蓝点位采用 `src/function/config/on_demand/points.yaml`；底图默认 `silou_edited.yaml`。原始地图保持不变，标注图仅供查看。


### 测试链路问题
1.检查Map Server
- `ros2 lifecycle get /map_server`
- 输出额结果

  - | 输出             | 含义                       |
    | ---------------- | -------------------------- |
    | active [3]       | （正常）地图服务器已经激活 |
    | inactive [2]     | （异常）已配置，但没有激活 |
    | unconfigured [1] | （异常）尚未完成配置       |

2.检查Planner Server

- `ros2 lifecycle get /planner_server`

3.检查Controller Server

- `ros2 lifecycle get /controller_server`

4.确认静态地图有没有发布

- `ros2 topic echo /map --once --qos-durability transient_local --field info`

5.检查GICP和Fast_lio的TF

- `ros2 run tf2_ros tf2_echo map base_footprint`