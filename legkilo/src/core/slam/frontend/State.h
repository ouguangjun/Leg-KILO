// SPDX-License-Identifier: MIT
// @file State.h
// @brief State vector, covariance and helpers.
// @author Ou Guangjun
// @created 2025-09-21
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_STATE_H
#define LEG_KILO_STATE_H

#include "common/math_utils.hpp"

namespace legkilo {

constexpr int DIM_STATE = 24;
using StateVec = Eigen::Matrix<double, DIM_STATE, 1>;
using StateCov = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
using StateF = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
using StateMat = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
using StateQ = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;

class State {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Mat3D rot_;    // rotation
    Vec3D pos_;    // position
    Vec3D vel_;    // velocity
    Vec3D ba_;     // accelerator bias
    Vec3D bw_;     // gyroscope bias
    Vec3D grav_;   // global gravity
    Vec3D imu_a_;  // acceleration
    Vec3D imu_w_;  // Angular velocity

    State();
    void operator+=(const StateVec& delta);
    StateVec operator-(const State& other) const;
};

}  // namespace legkilo

#endif  // LEG_KILO_STATE_H
