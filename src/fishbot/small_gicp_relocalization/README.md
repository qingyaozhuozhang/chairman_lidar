# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/LihanChen2004/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

## Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/LihanChen2004/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=release
    ```

## Usage

### Standalone executables

The package installs two C++ executables:

```bash
ros2 run small_gicp_relocalization small_gicp_relocalization_node
ros2 run small_gicp_relocalization tf_relay_node
```

`tf_relay_node` reads `map1 -> odom1` and republishes the same transform as
`map -> odom`. It publishes the configured initial transform until the source
becomes available, then retains the latest valid source transform. Output
timestamps use the current ROS clock plus `time_offset_sec`.

| Parameter | Default | Meaning |
|---|---|---|
| `source_parent`, `source_child` | `map1`, `odom1` | Source TF pair |
| `target_parent`, `target_child` | `map`, `odom` | Published TF pair |
| `publish_rate_hz` | `20.0` | Output rate; range `(0, 1000]` |
| `time_offset_sec` | `0.1` | Output timestamp offset in seconds |
| `init_pose` | `[-5.08, -1.5, 0.0, 0.0, 0.0, -1.5708]` | Initial `[x, y, z, roll, pitch, yaw]`, meters/radians |
| `use_sim_time` | `false` | Use the ROS simulation clock |

`fishbot_navigation2/launch/navigation2.launch.py` starts both executables.
The relay's `init_pose` is the fallback map-to-odom transform, not a robot pose
subscription. Edit its parameters in that launch file for the actual map.
Only one relay should publish the target TF pair at a time.

### Localization launch

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```
