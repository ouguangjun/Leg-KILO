#include "interface/common/options.h"

namespace legkilo {
namespace options {

common::SensorType kSensorType = common::SensorType::LIO;
std::atomic_bool FLAG_EXIT{false};
bool kRedundancy = false;

std::string kLidarTopic;
std::string kKinematicTopic;
std::string kImuTopic;

}  // namespace options
}  // namespace legkilo
