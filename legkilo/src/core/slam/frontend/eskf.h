// SPDX-License-Identifier: MIT
// @file eskf.h
// @brief Error-state Kalman filter interfaces and updates.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_ESKF_H
#define LEG_KILO_ESKF_H

#include "common/math_utils.hpp"
#include "common/sensor_types.hpp"
#include "core/slam/frontend/State.h"

namespace legkilo {

struct ObsShared {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, Eigen::Dynamic, 1> pt_z;
    Eigen::Matrix<double, Eigen::Dynamic, 6> pt_h;
    Eigen::Matrix<double, Eigen::Dynamic, 1> pt_R;

    Eigen::Matrix<double, Eigen::Dynamic, 1> ki_z;
    Eigen::Matrix<double, Eigen::Dynamic, DIM_STATE> ki_h;
    Eigen::Matrix<double, Eigen::Dynamic, 1> ki_R;
};

// Process noise parameters for ESKF (used to build Q only)
struct EskfProcessNoise {
    double vel_process_cov = 0.0;
    double imu_acc_process_cov = 0.0;
    double imu_gyr_process_cov = 0.0;
    double acc_bias_process_cov = 0.0;
    double gyr_bias_process_cov = 0.0;
};

enum class PredictModelType { Inertial = 0, ConstantVelocity = 1 };

class ESKF {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct PredictInput {
        Vec3D imu_acc = Vec3D::Zero();
        Vec3D imu_gyr = Vec3D::Zero();
        Vec3D world_vel = Vec3D::Zero();
    };

    ESKF(const EskfProcessNoise& noise);

    State& state() { return state_; }

    const State& state() const { return state_; }

    void setState(const State& state) { state_ = state; }

    State getState() const { return state_; }

    Mat3D getRot() const { return state_.rot_; }

    Vec3D getPos() const { return state_.pos_; }

    Vec3D getVel() const { return state_.vel_; }

    Mat3D getRotCov() const { return cov_.block<3, 3>(0, 0); }

    Mat3D getPosCov() const { return cov_.block<3, 3>(3, 3); }

    Mat3D getVelCov() const { return cov_.block<3, 3>(6, 6); }

    StateQ& Q() { return Q_; }

    const StateQ& Q() const { return Q_; }

    void setQ(const StateQ& Q) { Q_ = Q; }

    PredictModelType predictModel() const { return predict_model_; }

    void setPredictModel(PredictModelType predict_model);

    StateCov& cov() { return cov_; }

    const StateCov& cov() const { return cov_; }

    const EskfProcessNoise& processNoise() const { return process_noise_; }

    StateVec getFunctionf(double dt, const PredictInput& input) const;

    StateF getFx(double dt, const PredictInput& input) const;

    void predict(double dt, const PredictInput& input, bool prop_state, bool prop_cov);

    void updateByPoints(const ObsShared& obs_shared);

    bool updateByCloud(const ObsShared& obs_shared, const State& prior_state, int iter_max, int iter_cur);

    void updateByImu(const ObsShared& obs_shared);

    void updateByKinImu(const ObsShared& obs_shared);

   private:
    ESKF() = default;

    StateVec getFunctionfInertial(double dt, const PredictInput& input) const;

    StateVec getFunctionfConstantVelocity(double dt, const PredictInput& input) const;

    StateF getFxInertial(double dt, const PredictInput& input) const;

    StateF getFxConstantVelocity(double dt, const PredictInput& input) const;

    void rebuildProcessNoise();

    EskfProcessNoise process_noise_;
    State state_;
    StateCov cov_;
    StateQ Q_;
    PredictModelType predict_model_ = PredictModelType::Inertial;

    bool init_state = false;
};
}  // namespace legkilo
#endif  // LEG_KILO_ESKF_H
