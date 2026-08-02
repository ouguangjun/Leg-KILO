// SPDX-License-Identifier: MIT
// @file state_initial.hpp
// @brief State initialization using IMU or Kin+IMU.
// @author Ou Guangjun
// @created 2025-04-26
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_STATE_INITIAL_HPP
#define LEG_KILO_STATE_INITIAL_HPP

#include <algorithm>
#include <deque>

#include "common/math_utils.hpp"
#include "common/sensor_types.hpp"

namespace legkilo {

enum class InitType { Identity = 1, GravityAlignment = 2 };

class StateInitial {
   public:
    struct Config {
        double gravity = 9.81;
        size_t min_samples = 1;
        InitType init_type = InitType::Identity;
    };

    explicit StateInitial(const Config& cfg) : cfg_(cfg) { reset(); }
    virtual ~StateInitial() = default;
    virtual size_t ingest(const common::MeasGroup& meas) = 0;

    bool isReady() const { return N_ >= static_cast<int>(cfg_.min_samples); }
    void reset() {
        N_ = 0;
        mean_acc_.setZero();
        mean_gyr_.setZero();
        acc_norm_ = 0.0;
    }

    virtual double accNorm() const { return acc_norm_; }
    virtual Eigen::Vector3d gravityVec() const {
        if (cfg_.init_type == InitType::GravityAlignment) { return Eigen::Vector3d(0, 0, -cfg_.gravity); }
        if (acc_norm_ <= 0.0) return Eigen::Vector3d(0, 0, -cfg_.gravity);
        return -cfg_.gravity * (mean_acc_ / acc_norm_);
    }
    virtual Eigen::Vector3d gyroBias() const { return mean_gyr_; }
    virtual Eigen::Matrix3d initRotation() const {
        if (cfg_.init_type == InitType::GravityAlignment && acc_norm_ > 0.0) {
            Eigen::Vector3d acc_dir = -mean_acc_.normalized();
            Eigen::Vector3d g_world(0, 0, -1.0);
            Eigen::Quaterniond q0 = Eigen::Quaterniond::FromTwoVectors(acc_dir, g_world);
            q0.normalize();
            return q0.toRotationMatrix();
        }
        return Eigen::Matrix3d::Identity();
    }

   protected:
    inline void updateMean(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr) {
        N_ += 1;
        const double invN = 1.0 / static_cast<double>(N_);
        mean_acc_ += (acc - mean_acc_) * invN;
        mean_gyr_ += (gyr - mean_gyr_) * invN;
    }

    Config cfg_;
    int N_ = 0;
    Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();
    double acc_norm_ = 0.0;
};

class StateInitialByImu : public StateInitial {
   public:
    using StateInitial::StateInitial;
    size_t ingest(const common::MeasGroup& meas) override {
        size_t added = 0;
        for (const auto& imu : meas.imus_) {
            const auto& a = imu->linear_acceleration;
            const auto& g = imu->angular_velocity;
            const Eigen::Vector3d acc(a.x, a.y, a.z);
            const Eigen::Vector3d gyr(g.x, g.y, g.z);
            updateMean(acc, gyr);
            ++added;
        }
        if (N_ > 0) acc_norm_ = mean_acc_.norm();
        return added;
    }
};

class StateInitialByLidar : public StateInitial {
   public:
    using StateInitial::StateInitial;
    size_t ingest(const common::MeasGroup& meas) override {
        (void)meas;
        N_ = std::max(N_, static_cast<int>(cfg_.min_samples));
        acc_norm_ = cfg_.gravity;
        return 0;
    }
    double accNorm() const override { return cfg_.gravity; }
    Eigen::Vector3d gravityVec() const override { return Eigen::Vector3d::Zero(); }
    Eigen::Vector3d gyroBias() const override { return Eigen::Vector3d::Zero(); }
    Eigen::Matrix3d initRotation() const override { return Eigen::Matrix3d::Identity(); }
};

class StateInitialByKinImu : public StateInitial {
   public:
    using StateInitial::StateInitial;
    size_t ingest(const common::MeasGroup& meas) override {
        size_t added = 0;
        for (const auto& ki : meas.kin_imus_) {
            const Eigen::Vector3d acc(ki.acc_[0], ki.acc_[1], ki.acc_[2]);
            const Eigen::Vector3d gyr(ki.gyr_[0], ki.gyr_[1], ki.gyr_[2]);
            updateMean(acc, gyr);
            ++added;
        }
        if (N_ > 0) acc_norm_ = mean_acc_.norm();
        return added;
    }
};

}  // namespace legkilo
#endif  // LEG_KILO_STATE_INITIAL_HPP
