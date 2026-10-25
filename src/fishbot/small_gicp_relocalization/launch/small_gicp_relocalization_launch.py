# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    selected = int(os.environ.get("SELECTED_POSE", "1"))
    pose_file = (
        Path(get_package_share_directory("odometry")) / "config/initial_poses.yaml"
    )
    config = yaml.safe_load(pose_file.read_text())
    mapping_pose = config.get("mapping_pose") if isinstance(config, dict) else None
    mapping_fields = ("x", "y", "z", "roll", "pitch", "yaw")
    if not isinstance(mapping_pose, dict) or not all(
        type(mapping_pose.get(key)) in (int, float) and math.isfinite(mapping_pose[key])
        for key in mapping_fields
    ):
        raise ValueError(
            "initial_poses.yaml 的 mapping_pose 必须包含有限数值 x/y/z/roll/pitch/yaw"
        )
    map_to_pcd = [float(mapping_pose[key]) for key in mapping_fields]
    poses = config["poses"]
    if selected not in (1, 2, 3, 4) or selected not in poses:
        raise ValueError("SELECTED_POSE 必须在 initial_poses.yaml 中定义为 1、2、3、4")
    pose = poses[selected]
    x, y, qz, qw = [
        float(pose[key])
        for key in ("lidar_init_x", "lidar_init_y", "lidar_init_qz", "lidar_init_qw")
    ]
    if not all(math.isfinite(value) for value in (x, y, qz, qw)):
        raise ValueError("启动位置与四元数必须为有限数值")
    norm = math.hypot(qz, qw)
    if norm < 1e-9:
        raise ValueError("启动四元数不能为零")
    qz, qw = qz / norm, qw / norm
    yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "prior_pcd_file",
                default_value="",
                description="已有 PCD 地图的路径；总导航启动时自动传入",
            ),
            LogInfo(msg=f"[GICP 启动] 选择启动点 {selected}；建图点与启动点文件：{pose_file}"),
            Node(
                package="small_gicp_relocalization",
                executable="small_gicp_relocalization_node",
                name="small_gicp_relocalization",
                output="screen",
                parameters=[
                    {
                        "initial_map_pose": [x, y, 0.0, 0.0, 0.0, yaw],
                        "map_to_pcd": map_to_pcd,
                        "prior_pcd_file": ParameterValue(
                            LaunchConfiguration("prior_pcd_file"), value_type=str
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
