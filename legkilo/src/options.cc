#include "options.h"

namespace legkilo {
namespace options {

bool kKinAndImuUse = false;
bool kImuUse = false;
bool FLAG_EXIT = false;
bool kRedundancy = false;

std::string kLidarTopic;
std::string kKinematicTopic;
std::string kImuTopic;

} // namespace options
} // namespace legkilo