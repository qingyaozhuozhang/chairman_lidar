#ifndef SMALL_GICP_RELOCALIZATION__MAP_CONSISTENCY_HPP_
#define SMALL_GICP_RELOCALIZATION__MAP_CONSISTENCY_HPP_

#include <cmath>
#include <cstddef>
#include <memory>

#include <Eigen/Geometry>
#include "pcl/point_cloud.h"
#include "small_gicp/ann/kdtree.hpp"
#include "small_gicp/pcl/pcl_point.hpp"

namespace small_gicp_relocalization
{

/// @brief 删除在 target 地图中没有近邻支撑的 source 点。
inline pcl::PointCloud<pcl::PointCovariance>::Ptr filterByMapConsistency(
  const pcl::PointCloud<pcl::PointCovariance> & source,
  const small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>> & target_tree,
  const Eigen::Isometry3d & T_target_source,
  double max_distance)
{
  auto output = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
  if (max_distance <= 0.0) {
    *output = source;
    return output;
  }

  const double max_sq_distance = max_distance * max_distance;
  output->reserve(source.size());

  for (size_t i = 0; i < source.size(); ++i) {
    const Eigen::Vector3d p_source =
      source.at(i).getVector4fMap().template head<3>().template cast<double>();
    if (!p_source.allFinite()) {
      continue;
    }

    const Eigen::Vector4d p_target = (T_target_source * p_source).homogeneous();
    size_t target_index = 0;
    double sq_distance = 0.0;
    const size_t found = target_tree.nearest_neighbor_search(
      p_target, &target_index, &sq_distance);

    if (found > 0 && std::isfinite(sq_distance) && sq_distance <= max_sq_distance) {
      output->push_back(source.at(i));
    }
  }

  return output;
}

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__MAP_CONSISTENCY_HPP_