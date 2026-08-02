#pragma once

#include <Eigen/Core>

namespace kiss_matcher {

/// @brief Fast floor (https://stackoverflow.com/questions/824118/why-is-floor-so-slow).
/// @param pt  Double vector
/// @return    Floored int vector
inline Eigen::Array4i fast_floor(const Eigen::Array4d& pt) {
  const Eigen::Array4i ncoord = pt.cast<int>();
  return ncoord - (pt < ncoord.cast<double>()).cast<int>();
}

inline Eigen::Array3i fast_floor_array3f(const Eigen::Array3f& pt) {
  Eigen::Array3i ncoord = pt.cast<int>();
  return ncoord - (pt < ncoord.cast<float>()).cast<int>();
}

inline Eigen::Array3i fast_floor_vector3f(const Eigen::Vector3f& pt) {
  return fast_floor_array3f(pt.array());
}

}  // namespace kiss_matcher
