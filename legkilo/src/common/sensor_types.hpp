#ifndef LEG_KILO_SENSOR_TYPES_H
#define LEG_KILO_SENSOR_TYPES_H

#include <sensor_msgs/Imu.h>
#include <deque>

#include "pcl_types.h"

namespace legkilo {
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

#endif  // LEG_KILO_SENSOR_TYPES_H