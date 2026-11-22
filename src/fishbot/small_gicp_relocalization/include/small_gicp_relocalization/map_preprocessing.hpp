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

#ifndef SMALL_GICP_RELOCALIZATION__MAP_PREPROCESSING_HPP_
#define SMALL_GICP_RELOCALIZATION__MAP_PREPROCESSING_HPP_

#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "pcl/point_cloud.h"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_point_traits.hpp"
#include "small_gicp/util/downsampling.hpp"

namespace small_gicp_relocalization
{

// 体素边长随离线地图保存，实时扫描无需维护另一份参数。
inline double readMapVoxelSize(std::istream & input)
{
  double leaf_size = 0.0;
  bool found = false;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream fields(line);
    std::string key, name, extra;
    fields >> key;
    if (key == "DATA") {
      break;
    }
    if (key != "#" || !(fields >> name) || name != "chairman_lidar_voxel_size") {
      continue;
    }
    if (
      found || !(fields >> leaf_size) || (fields >> extra) || !std::isfinite(leaf_size) ||
      leaf_size <= 0.0 || !std::isfinite(1.0 / leaf_size)) {
      throw std::runtime_error("PCD has invalid or duplicate chairman_lidar_voxel_size metadata");
    }
    found = true;
  }
  if (!found) {
    throw std::runtime_error(
      "PCD has no voxel size metadata; generate final.pcd with tool/downsample_map.py");
  }
  return leaf_size;
}

// 仅为后续协方差估计转换点类型，保留输入点的坐标和数量。
inline pcl::PointCloud<pcl::PointCovariance>::Ptr toCovariancePoints(
  const pcl::PointCloud<pcl::PointXYZ> & map)
{
  auto target = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
  target->resize(map.size());
  for (size_t i = 0; i < map.size(); ++i) {
    target->at(i).getVector4fMap() = map.at(i).getVector4fMap();
  }
  return target;
}

inline pcl::PointCloud<pcl::PointCovariance>::Ptr downsampleLivePoints(
  const pcl::PointCloud<pcl::PointXYZ> & scan, double leaf_size)
{
  // 每个占用体素只取一个完整质心，与离线脚本的格网和聚合方式一致。
  return small_gicp::voxelgrid_sampling<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(scan, leaf_size);
}

}  // namespace small_gicp_relocalization
#endif  // SMALL_GICP_RELOCALIZATION__MAP_PREPROCESSING_HPP_
