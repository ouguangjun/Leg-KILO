// SPDX-License-Identifier: MIT
// @file sensor_types.hpp
// @brief Common sensor data structures and LidarType.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_SENSOR_TYPES_H
#define LEG_KILO_SENSOR_TYPES_H

#include <algorithm>
#include <cctype>
#include <deque>
#include <stdexcept>
#include <string>

#include "interface/common/ros_compat.h"
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
    std::deque<ros_compat::ImuMsgPtr> imus_;
    std::deque<KinImuMeas> kin_imus_;
};

enum class SensorType { LO = 0, LIO = 1, KILO = 2 };

inline SensorType parseSensorType(std::string sensor_type) {
    std::transform(sensor_type.begin(), sensor_type.end(), sensor_type.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if (sensor_type == "LO") return SensorType::LO;
    if (sensor_type == "LIO") return SensorType::LIO;
    if (sensor_type == "KILO") return SensorType::KILO;
    throw std::invalid_argument("Unsupported sensor_type: " + sensor_type);
}

inline const char* toString(const SensorType sensor_type) {
    switch (sensor_type) {
        case SensorType::LO: return "LO";
        case SensorType::LIO: return "LIO";
        case SensorType::KILO: return "KILO";
    }
    return "UNKNOWN";
}

inline bool usesImu(const SensorType sensor_type) { return sensor_type == SensorType::LIO; }

inline bool usesKinematics(const SensorType sensor_type) { return sensor_type == SensorType::KILO; }

inline bool usesLidarOnly(const SensorType sensor_type) { return sensor_type == SensorType::LO; }

enum class LidarType { VEL = 1, OUSTER = 2, HESAI = 3, KITTI = 4, LIVOX = 5 };

}  // namespace common
}  // namespace legkilo

#endif  // LEG_KILO_SENSOR_TYPES_H
