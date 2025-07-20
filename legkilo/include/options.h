#ifndef LEG_KILO_OPTIONS_H
#define LEG_KILO_OPTIONS_H

#include <string>
#include <vector>

namespace legkilo{
namespace options{

extern std::string kLidarTopic;
extern std::string kKinematicTopic;
extern std::string kImuTopic;

extern bool kKinAndImuUse; // use kin. and imu together
extern bool kImuUse; // only use imu 
extern bool FLAG_EXIT;
extern bool kRedundancy; // for legkilo dataset

} // namespace options
} // namespace legkilo

#endif // LEG_KILO_OPTIONS_H