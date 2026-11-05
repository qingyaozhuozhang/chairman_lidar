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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "small_gicp_relocalization/pose_transform.hpp"

using small_gicp_relocalization::initialAlignment;
using small_gicp_relocalization::mapOdometryFromStart;
using small_gicp_relocalization::poseFromXyzRpy;

TEST(InitialAlignment, IncludesMapRotationAndBodyOffset)
{
  const auto map_pcd = poseFromXyzRpy({10., -2., 0., 0., 0., M_PI_2});
  const auto map_robot = poseFromXyzRpy({4., 6., 0., 0., 0., M_PI_2});
  const auto odom_robot = poseFromXyzRpy({-0.25, 0., 0., 0., 0., 0.});
  const auto pcd_odom = initialAlignment(map_pcd, map_robot, odom_robot);
  EXPECT_NEAR(pcd_odom.translation().x(), 8.25, 1e-9);
  EXPECT_NEAR(pcd_odom.translation().y(), 6.0, 1e-9);
  EXPECT_TRUE(pcd_odom.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(InitialAlignment, HandlesNonzeroLocalPositionAndOrientation)
{
  const auto map_pcd = poseFromXyzRpy({0., 0., 0., 0., 0., 0.});
  const auto map_robot = poseFromXyzRpy({5., 7., 0., 0., 0., M_PI_2});
  const auto odom_robot = poseFromXyzRpy({2., 1., 0., 0., 0., M_PI_2});
  const auto pcd_odom = initialAlignment(map_pcd, map_robot, odom_robot);
  EXPECT_NEAR(pcd_odom.translation().x(), 3.0, 1e-9);
  EXPECT_NEAR(pcd_odom.translation().y(), 6.0, 1e-9);
  EXPECT_TRUE(pcd_odom.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

TEST(InitialAlignment, SecondStartUsesItsRelativePositionInTheOriginalPcd)
{
  const auto map_pcd = poseFromXyzRpy({-5.082, -1.5, 0., 0., 0., -M_PI_2});
  const auto map_robot = poseFromXyzRpy({5.708, -5.257, 0., 0., 0., M_PI_2});
  const auto odom_robot = poseFromXyzRpy({-0.2365, 0., 0., 0., 0., 0.});
  const auto pcd_odom = initialAlignment(map_pcd, map_robot, odom_robot);
  const auto pcd_robot = pcd_odom * odom_robot;
  EXPECT_NEAR(pcd_robot.translation().x(), 3.757, 1e-9);
  EXPECT_NEAR(pcd_robot.translation().y(), 10.790, 1e-9);
  EXPECT_NEAR(std::abs(std::atan2(pcd_robot(1, 0), pcd_robot(0, 0))), M_PI, 1e-5);
  EXPECT_NEAR(pcd_odom.translation().x(), 3.5205, 1e-9);
  EXPECT_NEAR(pcd_odom.translation().y(), 10.790, 1e-9);
  EXPECT_TRUE((map_pcd * pcd_odom * odom_robot).isApprox(map_robot, 1e-9));
}

TEST(InitialAlignment, MappingPointIsTheRobotOriginInPcd)
{
  const auto map_pcd = poseFromXyzRpy({-5.082, -1.5, 0., 0., 0., -M_PI_2});
  const auto odom_robot = poseFromXyzRpy({-0.2365, 0., 0., 0., 0., 0.});
  const auto seed = initialAlignment(map_pcd, map_pcd, odom_robot);
  EXPECT_TRUE((seed * odom_robot).isApprox(Eigen::Isometry3d::Identity(), 1e-9));
  EXPECT_TRUE(initialAlignment(map_pcd, map_pcd, Eigen::Isometry3d::Identity())
                .isApprox(Eigen::Isometry3d::Identity(), 1e-9));
}

TEST(MapOdometry, StartingPoseDoesNotCountThePcdOffsetTwice)
{
  const auto map_start = poseFromXyzRpy({5.708, -5.257, 0., 0., 0., M_PI_2});
  const auto pcd_start = poseFromXyzRpy({3.757, 10.790, 0., 0., 0., M_PI});
  const auto seed = poseFromXyzRpy({3.5205, 10.790, 0., 0., 0., M_PI});
  const auto odom_robot = poseFromXyzRpy({-0.2365, 0., 0., 0., 0., 0.});
  const auto output = mapOdometryFromStart(map_start, pcd_start, seed);
  EXPECT_NEAR(output.translation().x(), 5.708, 1e-9);
  EXPECT_NEAR(output.translation().y(), -5.0205, 1e-9);
  EXPECT_TRUE((output * odom_robot).isApprox(map_start, 1e-9));

  const auto matched = poseFromXyzRpy({3.5605, 10.760, 0., 0., 0., M_PI + 0.02});
  const auto corrected = mapOdometryFromStart(map_start, pcd_start, matched);
  const auto expected = poseFromXyzRpy({5.678, -5.0605, 0., 0., 0., M_PI_2 + 0.02});
  EXPECT_TRUE(corrected.isApprox(expected, 1e-9));
}

TEST(MapOdometry, ReinitializationCanUseAnotherStartupReference)
{
  const auto map_pcd = poseFromXyzRpy({-5.082, -1.5, 0., 0., 0., -M_PI_2});
  const auto odom_robot = poseFromXyzRpy({2., -3., 0.4, 0.05, -0.03, 0.7});
  for (const auto & map_start :
       {poseFromXyzRpy({5.708, -5.257, 0., 0., 0., M_PI_2}),
        poseFromXyzRpy({5.708, 5.257, 0., 0., 0., -M_PI_2})}) {
    const Eigen::Isometry3d pcd_start = map_pcd.inverse() * map_start;
    const auto seed = initialAlignment(map_pcd, map_start, odom_robot);
    const auto output = mapOdometryFromStart(map_start, pcd_start, seed);
    EXPECT_TRUE((output * odom_robot).isApprox(map_start, 1e-9));
  }
}

TEST(InitialAlignment, RejectsMalformedOrNonFinitePose)
{
  EXPECT_THROW(poseFromXyzRpy({1., 2.}), std::invalid_argument);
  EXPECT_THROW(
    poseFromXyzRpy({1., 2., 0., 0., 0., std::numeric_limits<double>::quiet_NaN()}),
    std::invalid_argument);
}
