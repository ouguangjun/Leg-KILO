#ifndef LEG_KILO_PCL_TYPES_H
#define LEG_KILO_PCL_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>

namespace legkilo {

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using CloudConstPtr = PointCloudType::ConstPtr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

}  // namespace legkilo

#endif  // LEG_KILO_PCL_TYPES_H