#include "small_gicp_relocalization/cropped_cloud.hpp"
#include <pcl/filters/crop_box.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cropByXY(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    float x_min, float x_max, float y_min, float y_max)
{
  auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (!input || input->empty()) return output;

  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(input);
  crop.setMin(Eigen::Vector4f(x_min, y_min, -1000.0f, 1.0f));  // Z 不限制
  crop.setMax(Eigen::Vector4f(x_max, y_max,  1000.0f, 1.0f));
  crop.filter(*output);
  return output;
}