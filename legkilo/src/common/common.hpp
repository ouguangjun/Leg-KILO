#ifndef LEG_KILO_COMMON_H
#define LEG_KILO_COMMON_H

#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>

namespace legkilo {

#define THREAD_SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using CloudConstPtr = PointCloudType::ConstPtr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

namespace common {

struct LidarScan {
    double lidar_begin_time_;
    double lidar_end_time_;
    CloudPtr cloud_;
};

// leg order: FR FL RR RL
struct KinImuMeas {
    double time_stamp_;
    double foot_pos_[4][3];
    double foot_vel_[4][3];
    bool contact_[4];
    double acc_[3];
    double gyr_[3];
};

struct MeasGroup {
    LidarScan lidar_scan_;
    std::deque<sensor_msgs::ImuPtr> imus_;
    std::deque<KinImuMeas> kin_imus_;
};

enum class LidarType { VEL = 1, OUSTER = 2 };

}  // namespace common
}  // namespace legkilo

#endif  // LEG_KILO_COMMON_H