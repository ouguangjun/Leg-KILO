// SPDX-License-Identifier: MIT
// @file pcl_types.h
// @brief PCL point type aliases and cloud typedefs.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_PCL_TYPES_H
#define LEG_KILO_PCL_TYPES_H

#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(>=, 1, 11, 0)
#include <memory>
#else
#include <boost/make_shared.hpp>
#endif

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <vector>

namespace legkilo {

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using CloudConstPtr = PointCloudType::ConstPtr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

enum class LidarMatchType : uint8_t {
    Unused = 0,
    Point2Plane = 1,
    Ndt = 2,
};

using LidarMatchTypes = std::vector<LidarMatchType>;
using LidarMatchTypesPtr = std::shared_ptr<LidarMatchTypes>;

struct GaussPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d pt;
    Eigen::Matrix3d cov;
    GaussPoint() {
        pt = Eigen::Vector3d::Zero();
        cov = Eigen::Matrix3d::Zero();
    }
    GaussPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& v) : pt(p), cov(v) {}
};
using GaussCloud = std::vector<GaussPoint, Eigen::aligned_allocator<GaussPoint>>;
using GaussCloudPtr = std::shared_ptr<GaussCloud>;

namespace pcl_utils {
template <typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr makeCloud() {
#if PCL_VERSION_COMPARE(>=, 1, 11, 0)
    return std::make_shared<pcl::PointCloud<PointT>>();
#else
    return boost::make_shared<pcl::PointCloud<PointT>>();
#endif
}

inline void GaussCloudToPclCloud(const GaussCloud& cloud_gauss, CloudPtr& cloud_pcl) {
    cloud_pcl->clear();
    const size_t N = cloud_gauss.size();
    cloud_pcl->resize(N);
    for (size_t i = 0; i < N; ++i) { cloud_pcl->points[i].getVector3fMap() = cloud_gauss[i].pt.cast<float>(); }
}

using CloudVec = std::vector<Eigen::Vector3f>;
using CloudVecPtr = std::shared_ptr<CloudVec>;
inline CloudVecPtr PclCloudToVecCloud(const CloudPtr& cloud_pcl) {
    auto cloud_vec = std::make_shared<CloudVec>();
    cloud_vec->reserve(cloud_pcl->size());
    for (const auto& pt : cloud_pcl->points) { cloud_vec->emplace_back(pt.x, pt.y, pt.z); }
    return cloud_vec;
}

}  // namespace pcl_utils

}  // namespace legkilo

#endif  // LEG_KILO_PCL_TYPES_H
