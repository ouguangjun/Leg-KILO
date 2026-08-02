// SPDX-License-Identifier: MIT
// @file lidar_processing.h
// @brief LiDAR preprocessing: filtering, conversion and noise.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_LIDAR_PROCESSING_H
#define LEG_KILO_LIDAR_PROCESSING_H

#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "interface/common/ros_compat.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    // std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    // (std::uint16_t, ring, ring)
)
// clang-format on
namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    std::uint16_t reflectivity;
    // std::uint8_t ring;
    std::uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    // (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on
namespace hesai_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    // std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace hesai_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    // (std::uint16_t, ring, ring)
)
// clang-format on
namespace kitti_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace kitti_ros
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(kitti_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
)
// clang-format on

namespace legkilo {

class LidarProcessing {
   public:
    struct Config {
        float min_range_ = 1.0f;
        float max_range_ = 100.0f;
        int filter_num_ = 1;
        common::LidarType lidar_type_ = common::LidarType::VEL;
        double time_scale_ = 1.0;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LidarProcessing() = delete;
    LidarProcessing(LidarProcessing::Config config);
    ~LidarProcessing();

    common::LidarType getLidarType() const;
    void processing(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void processing(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan);

    template <typename T>
    inline bool rangeCheck(const T& p) {
        const float d2 = p.x * p.x + p.y * p.y + p.z * p.z;
        return (d2 < min_range2_) || (d2 > max_range2_);
    }

    template <typename PointT>
    inline bool nanCheck(const PointT& p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
    }

   private:
    void velodyneHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void ousterHander(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void hesaiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void kittiHandler(const ros_compat::PointCloud2MsgConstPtr& msg, common::LidarScan& lidar_scan);
    void livoxHandler(const ros_compat::LivoxCustomMsgConstPtr& msg, common::LidarScan& lidar_scan);

    CloudPtr cloud_pcl_;

    float min_range2_ = 1.0f;
    float max_range2_ = 10000.0f;
    Config config_;
};

}  // namespace legkilo

#endif  // LEG_KILO_LIDAR_PROCESSING_H
