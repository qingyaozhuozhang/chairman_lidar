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

### Localization launch

From the `chairman_lidar` project root after sourcing the workspace:

```bash
SELECTED_POSE=1 ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py \
  prior_pcd_file:="$PWD/src/fishbot/fishbot_navigation2/PCD/scans.pcd"
```

The launch reads `mapping_pose` and the selected startup pose (1–4) from
`odometry/config/initial_poses.yaml`. `mapping_pose` gives the PCD coordinate
origin in `map`, with named `x/y/z/roll/pitch/yaw` fields (meters/radians).
Its initial values use startup point 1, which defines the PCD zero pose;
the mapping pose is saved independently of subsequent startup-point calibration.
The selected entry under `poses` represents `base_footprint` in the 2D navigation
`map` frame. Local odometry
must provide `odom -> base_footprint`, including the sensor mounting offset.
The navigation launch includes this localization launch and supplies the map path.
Run only one localization instance for a given TF tree.

The node retains the prior PCD in its original coordinates. With `T_A_B` mapping
coordinates from B to A, the initial matching estimate is:

```text
T_pcd_odom = inverse(T_map_pcd) * T_map_robot * inverse(T_odom_robot)
```

`T_map_pcd` is the fixed map calibration (`map_to_pcd`, supplied from `mapping_pose`);
it does not change when selecting another startup position. The node records the
selected startup reference in both frames: `T_map_start` and
`T_pcd_start = inverse(T_map_pcd) * T_map_start`. Before matching, the node publishes
`T_map_start * inverse(T_odom_robot)` as `map -> odom`. After each valid match,
it publishes `T_map_start * inverse(T_pcd_start) * T_pcd_odom`, equivalent to
`T_map_pcd * T_pcd_odom`. Converting the matching result to the startup reference
prevents applying the startup offset twice. Invalid matches retain the previous result.
The first valid match emits `[定位就绪]`. Subsequent valid matches emit `[定位更新]`
at most once every three seconds, and a valid match after rejected results emits
`[定位恢复]` immediately. These messages report numerical convergence, not verified
global accuracy; check live cloud alignment in RViz. An initial TF alone does not
confirm matching.

### Parameters and TF publication

The node loads the configured PCD and voxel-filters the map with
`global_leaf_size` and live scans with `registered_leaf_size` (both 0.05 m by
default). PCD coordinates remain unchanged by startup-pose selection. Mapping
and startup poses come from `odometry/config/initial_poses.yaml`.

| Parameter | Default | Meaning |
|---|---|---|
| `map_frame`, `odom_frame` | `map`, `odom` | Published TF pair |
| `robot_base_frame` | `base_footprint` | Robot frame used for startup pose conversion |
| `initial_map_pose` | Required; supplied by launch | Robot pose in map: `[x, y, z, roll, pitch, yaw]`, meters/radians |
| `map_to_pcd` | Required; supplied by launch | Fixed pose of the PCD coordinate system in map, from `mapping_pose`; meters/radians |
| `prior_pcd_file` | Required | Prior PCD map path |
| `global_leaf_size` | `0.05` | Map voxel size, meters |
| `registered_leaf_size` | `0.05` | Live scan voxel size, meters |
| `input_cloud_topic` | `/cloud_registered` | Point cloud transformed into odom using its frame and timestamp |
| `registration_rate_hz` | `2.0` | Matching rate; range `(0, 100]` |
| `publish_rate_hz` | `20.0` | Output rate; range `(0, 1000]` |
| `time_offset_sec` | `0.1` | Output timestamp offset in seconds |
| `use_sim_time` | `false` | Use the ROS simulation clock |

PCD loading, voxel sampling, covariance estimation and search tree construction
run at startup. Sampling does not change the PCD coordinate frame, so it does
not change `map_to_pcd`. Set that transform to identity only if the PCD
coordinates have already been transformed into `map`.

`small_gicp_relocalization_node` owns initialization, matching and TF publication.
Its TF timer runs in a separate callback group; the standalone executable uses a
multithreaded executor. Use a multithreaded container when loading the component
to preserve this scheduling behavior. Output timestamps use current ROS time plus
`time_offset_sec`, independently of the matching rate.

RViz `/initialpose` messages in `map_frame` reinitialize the matching estimate and
TF with the same conversion, and clear pending point clouds. Odometry startup
does not automatically publish this message.

### Standalone executable

Direct execution requires explicit mapping and startup poses, for example
(meters/radians):

```bash
ros2 run small_gicp_relocalization small_gicp_relocalization_node --ros-args \
  -p prior_pcd_file:="$PWD/src/fishbot/fishbot_navigation2/PCD/scans.pcd" \
  -p 'map_to_pcd:=[-5.082, -1.5, 0.0, 0.0, 0.0, -1.5707963267948966]' \
  -p 'initial_map_pose:=[-5.082, -1.5, 0.0, 0.0, 0.0, -1.5707963267948966]'
```
