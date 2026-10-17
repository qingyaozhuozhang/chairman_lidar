// Copyright 2026 chairman_lidar contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__POSE_TRANSFORM_HPP_
#define SMALL_GICP_RELOCALIZATION__POSE_TRANSFORM_HPP_

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace small_gicp_relocalization
{

inline Eigen::Isometry3d poseFromXyzRpy(const std::vector<double> & pose)
{
  if (pose.size() != 6 || !std::all_of(pose.begin(), pose.end(), [](double value) {
        return std::isfinite(value);
      })) {
    throw std::invalid_argument("pose requires six finite values: [x, y, z, roll, pitch, yaw]");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() << pose[0], pose[1], pose[2];
  result.linear() = (Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()))
                      .toRotationMatrix();
  return result;
}

// T_A_B 将 B 坐标变换到 A；启动位姿以 map 中的机器人参考点表示。
inline Eigen::Isometry3d initialAlignment(
  const Eigen::Isometry3d & map_to_pcd, const Eigen::Isometry3d & map_to_robot,
  const Eigen::Isometry3d & odom_to_robot)
{
  // 先求所选启动点在原 PCD 中的位置、朝向，再换算局部点云所用 odom 的配准初值。
  const Eigen::Isometry3d pcd_to_robot = map_to_pcd.inverse() * map_to_robot;
  return pcd_to_robot * odom_to_robot.inverse();
}

// 输出以所选启动点为参考；先将 PCD 配准结果转换到该参考系，避免重复叠加启动偏移。
inline Eigen::Isometry3d mapOdometryFromStart(
  const Eigen::Isometry3d & map_to_start, const Eigen::Isometry3d & pcd_to_start,
  const Eigen::Isometry3d & pcd_to_odom)
{
  const Eigen::Isometry3d start_to_odom = pcd_to_start.inverse() * pcd_to_odom;
  return map_to_start * start_to_odom;
}

}  // namespace small_gicp_relocalization
#endif  // SMALL_GICP_RELOCALIZATION__POSE_TRANSFORM_HPP_
