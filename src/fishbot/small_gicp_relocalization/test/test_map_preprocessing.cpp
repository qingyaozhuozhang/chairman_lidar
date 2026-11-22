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

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/util/normal_estimation_omp.hpp"
#include "small_gicp_relocalization/map_preprocessing.hpp"
#include "small_gicp_relocalization/pose_transform.hpp"

using small_gicp_relocalization::toCovariancePoints;

TEST(MapPreprocessing, VoxelSizeComesFromMapHeader)
{
  std::istringstream input(
    "# .PCD v0.7\n# chairman_lidar_voxel_size 0.125\nVERSION .7\nDATA binary\n");
  EXPECT_DOUBLE_EQ(small_gicp_relocalization::readMapVoxelSize(input), 0.125);
}

TEST(MapPreprocessing, MissingOrInvalidVoxelSizeIsRejected)
{
  for (const std::string text :
       {"VERSION .7\nDATA ascii\n# chairman_lidar_voxel_size 0.05\n",
        "# chairman_lidar_voxel_size 0\nDATA binary\n",
        "# chairman_lidar_voxel_size nan\nDATA binary\n",
        "# chairman_lidar_voxel_size 0.05 extra\nDATA binary\n",
        "# chairman_lidar_voxel_size 0.05\n# chairman_lidar_voxel_size 0.1\nDATA binary\n"}) {
    std::istringstream input(text);
    EXPECT_THROW(small_gicp_relocalization::readMapVoxelSize(input), std::runtime_error);
  }
}

TEST(MapPreprocessing, LiveVoxelCentroidsMatchOfflineConvention)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto & point : std::vector<pcl::PointXYZ>{
         {-0.04f, 0.01f, 0.01f},
         {-0.02f, 0.03f, 0.01f},
         {0.01f, 0.01f, 0.01f},
         {0.03f, 0.03f, 0.01f},
         {0.051f, 0.01f, 0.01f}}) {
    cloud.push_back(point);
  }
  const auto sampled = small_gicp_relocalization::downsampleLivePoints(cloud, 0.05);
  ASSERT_EQ(sampled->size(), 3u);
  std::sort(
    sampled->begin(), sampled->end(), [](const auto & a, const auto & b) { return a.x < b.x; });
  EXPECT_NEAR(sampled->at(0).x, -0.03, 1e-6);
  EXPECT_NEAR(sampled->at(0).y, 0.02, 1e-6);
  EXPECT_NEAR(sampled->at(1).x, 0.02, 1e-6);
  EXPECT_NEAR(sampled->at(2).x, 0.051, 1e-6);
  cloud.clear();
  for (int i = 0; i < 3000; ++i) {
    cloud.push_back({i < 1000 ? 0.01f : 0.04f, 0.01f, 0.01f});
  }
  const auto centroid = small_gicp_relocalization::downsampleLivePoints(cloud, 0.05);
  ASSERT_EQ(centroid->size(), 1u);
  EXPECT_NEAR(centroid->at(0).x, 0.03, 1e-6);
}

TEST(MapPreprocessing, PreparedMapPreservesAllCoordinates)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0.01f, 0.02f, 0.03f));
  cloud.push_back(pcl::PointXYZ(0.02f, 0.03f, 0.04f));
  const auto target = toCovariancePoints(cloud);
  ASSERT_EQ(target->size(), 2u);
  for (size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_TRUE(target->at(i).getVector4fMap().isApprox(cloud.at(i).getVector4fMap()));
    EXPECT_FLOAT_EQ(target->at(i).data[3], 1.0f);
  }
}

TEST(MapPreprocessing, SharedSearchTreeSupportsCovariancesAndRegistration)
{
  pcl::PointCloud<pcl::PointXYZ> cloud, scan;
  for (int i = 0; i < 300; ++i) {
    const pcl::PointXYZ point(std::sin(i * 0.37), std::cos(i * 0.79), std::sin(i * 1.53));
    cloud.push_back(point);
    scan.push_back(pcl::PointXYZ(point.x - 0.05f, point.y, point.z));
  }
  const auto target = toCovariancePoints(cloud);
  const auto source = toCovariancePoints(scan);
  small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>> tree(
    target, small_gicp::KdTreeBuilderOMP(2));
  small_gicp::estimate_covariances_omp(*target, tree, 10, 2);
  small_gicp::estimate_covariances_omp(*source, 10, 2);
  for (const auto & point : *target) {
    EXPECT_TRUE(point.cov.allFinite());
  }
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  registration.reduction.num_threads = 2;
  const auto result = registration.align(*target, *source, tree, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.T_target_source.translation().x(), 0.05, 1e-4);
  EXPECT_NEAR(result.T_target_source.translation().y(), 0.0, 1e-4);
  EXPECT_NEAR(result.T_target_source.translation().z(), 0.0, 1e-4);
}

TEST(InitialAlignment, GicpMatchesSecondStartFarFromMappingOrigin)
{
  using small_gicp_relocalization::initialAlignment;
  using small_gicp_relocalization::mapOdometryFromStart;
  using small_gicp_relocalization::poseFromXyzRpy;
  const auto map_pcd = poseFromXyzRpy({-5.082, -1.5, 0., 0., 0., -M_PI_2});
  const auto map_robot = poseFromXyzRpy({5.708, -5.257, 0., 0., 0., M_PI_2});
  const auto odom_robot = poseFromXyzRpy({-0.2365, 0., 0., 0., 0., 0.});
  const auto actual = poseFromXyzRpy({3.5605, 10.7600, 0., 0., 0., M_PI + 0.02});
  pcl::PointCloud<pcl::PointXYZ> cloud, scan;
  for (int i = 0; i < 300; ++i) {
    const Eigen::Vector3d point(
      3.757 + std::sin(i * 0.37), 10.790 + std::cos(i * 0.79), std::sin(i * 1.53));
    const Eigen::Vector3d local = actual.inverse() * point;
    cloud.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    scan.push_back(pcl::PointXYZ(local.x(), local.y(), local.z()));
  }
  const auto target = toCovariancePoints(cloud);
  const auto source = toCovariancePoints(scan);
  small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>> tree(
    target, small_gicp::KdTreeBuilderOMP(2));
  small_gicp::estimate_covariances_omp(*target, tree, 10, 2);
  small_gicp::estimate_covariances_omp(*source, 10, 2);
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  registration.reduction.num_threads = 2;
  registration.rejector.max_dist_sq = 4.0;
  registration.optimizer.max_iterations = 10;
  const auto seed = initialAlignment(map_pcd, map_robot, odom_robot);
  const auto result = registration.align(*target, *source, tree, seed);
  EXPECT_TRUE(result.converged);
  EXPECT_GE(result.num_inliers, 250u);
  EXPECT_TRUE(result.T_target_source.isApprox(actual, 1e-4));
  const Eigen::Isometry3d pcd_start = map_pcd.inverse() * map_robot;
  const auto output = mapOdometryFromStart(map_robot, pcd_start, result.T_target_source);
  const auto expected = poseFromXyzRpy({5.678, -5.0605, 0., 0., 0., M_PI_2 + 0.02});
  EXPECT_TRUE(output.isApprox(expected, 1e-4));
  const auto from_origin =
    registration.align(*target, *source, tree, Eigen::Isometry3d::Identity());
  EXPECT_EQ(from_origin.num_inliers, 0u);
}
