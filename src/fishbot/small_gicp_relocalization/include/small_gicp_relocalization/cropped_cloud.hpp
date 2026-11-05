#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cropByXY(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    float x_min, float x_max, float y_min, float y_max);