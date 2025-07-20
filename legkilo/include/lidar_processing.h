#ifndef LEG_KILO_LIDAR_PROCESSING_H
#define LEG_KILO_LIDAR_PROCESSING_H

#include "common.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <glog/logging.h>


namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

namespace legkilo{

class LidarProcessing{
public:
    struct Config{
        float blind_ = 1.0;
        int filter_num_ = 1;
        bool point_stamp_correct_ = true; // for leg kilo dataset
        common::LidarType lidar_type_ = common::LidarType::VEL;
        double time_scale_ = 1.0;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LidarProcessing() = delete;
    LidarProcessing(LidarProcessing::Config config);
    ~LidarProcessing();

    common::LidarType getLidarType() const;
    void processing(const sensor_msgs::PointCloud2::ConstPtr& msg,  common::LidarScan& lidar_scan);
    
    template <typename T>
    inline bool blindCheck(const T& p){
        return config_.blind_ * config_.blind_ > p.x * p.x + p.y * p.y + p.z * p.z;
    }

private:
    void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr& msg,  common::LidarScan& lidar_scan);
    void ousterHander(const sensor_msgs::PointCloud2::ConstPtr& msg,  common::LidarScan& lidar_scan);
    CloudPtr cloud_pcl_;
    Config config_;

};

} //namespace legkilo

#endif //LEG_KILO_LIDAR_PROCESSING_H