// SPDX-License-Identifier: MIT
// @file options.h
// @brief Runtime options and global flags shared by ROS interfaces.
#ifndef LEG_KILO_INTERFACE_COMMON_OPTIONS_H_
#define LEG_KILO_INTERFACE_COMMON_OPTIONS_H_

#include <atomic>
#include <string>

#include "common/sensor_types.hpp"

namespace legkilo {
namespace options {

extern std::string kLidarTopic;
extern std::string kKinematicTopic;
extern std::string kImuTopic;

extern common::SensorType kSensorType;
extern std::atomic_bool FLAG_EXIT;
extern bool kRedundancy;

inline bool useImu() { return common::usesImu(kSensorType); }

inline bool useKinematics() { return common::usesKinematics(kSensorType); }

inline bool useLidarOnly() { return common::usesLidarOnly(kSensorType); }

}  // namespace options
}  // namespace legkilo

#endif  // LEG_KILO_INTERFACE_COMMON_OPTIONS_H_
