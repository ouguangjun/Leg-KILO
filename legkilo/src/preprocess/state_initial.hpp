#ifndef LEG_KILO_STATE_INITIAL_HPP
#define LEG_KILO_STATE_INITIAL_HPP

#include "common/common.hpp"
#include "core/slam/eskf.h"

namespace legkilo {

using namespace common;

class StateInitial {
   public:
    virtual ~StateInitial() = default;
    virtual void processing(MeasGroup& measure, ESKF& eskf) = 0;
    inline double getAccNorm() const { return acc_norm_; }

   protected:
    explicit StateInitial(double G_m_s2) : G_m_s2_(G_m_s2){};

    int N = 0;
    bool b_first_frame_ = true;
    double G_m_s2_ = 9.81;
    double acc_norm_;
    Eigen::Vector3d mean_acc_;
    Eigen::Vector3d mean_gyr_;
    Eigen::Vector3d cov_acc_;
    Eigen::Vector3d cov_gyr_;
};

class StateInitialByImu : public StateInitial {
   public:
    explicit StateInitialByImu(double G_m_s2) : StateInitial(G_m_s2) {}

    void processing(MeasGroup& measure, ESKF& eskf) override {
        Eigen::Vector3d cur_acc;
        Eigen::Vector3d cur_gyr;

        if (b_first_frame_) {
            b_first_frame_ = false;
            N = 1;
            const auto& imu_acc = measure.imus_.front()->linear_acceleration;
            const auto& imu_gyr = measure.imus_.front()->angular_velocity;
            mean_acc_ << imu_acc.x, imu_acc.y, imu_acc.z;
            mean_gyr_ << imu_gyr.x, imu_gyr.y, imu_gyr.z;
            cov_acc_.setZero();
            cov_gyr_.setZero();
        }

        for (const auto& imu : measure.imus_) {
            const auto& imu_acc = imu->linear_acceleration;
            const auto& imu_gyr = imu->angular_velocity;
            cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
            cur_gyr << imu_gyr.x, imu_gyr.y, imu_gyr.z;

            mean_acc_ += (cur_acc - mean_acc_) / N;
            mean_gyr_ += (cur_gyr - mean_gyr_) / N;

            cov_acc_ = cov_acc_ * (N - 1.0) / N +
                       (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
            cov_gyr_ = cov_gyr_ * (N - 1.0) / N +
                       (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

            N++;
        }
        acc_norm_ = mean_acc_.norm();
        eskf.state().grav_ = -mean_acc_ / acc_norm_ * G_m_s2_;
        eskf.state().bw_ = mean_gyr_;
        eskf.state().rot_ = Mat3D::Identity();
        eskf.cov() = 0.000001 * StateCov::Identity();
        eskf.initProcessCovQ();
        return;
    }
};

class StateInitialByKinImu : public StateInitial {
   public:
    explicit StateInitialByKinImu(double G_m_s2) : StateInitial(G_m_s2) {}

    void processing(MeasGroup& measure, ESKF& eskf) override {
        Eigen::Vector3d cur_acc;
        Eigen::Vector3d cur_gyr;

        if (b_first_frame_) {
            b_first_frame_ = false;
            N = 1;
            const auto& imu_acc = measure.kin_imus_.front().acc_;
            const auto& imu_gyr = measure.kin_imus_.front().gyr_;
            mean_acc_ << imu_acc[0], imu_acc[1], imu_acc[2];
            mean_gyr_ << imu_gyr[0], imu_gyr[1], imu_gyr[2];
            cov_acc_.setZero();
            cov_gyr_.setZero();
        }

        for (const auto& kin_imu : measure.kin_imus_) {
            const auto& imu_acc = kin_imu.acc_;
            const auto& imu_gyr = kin_imu.gyr_;
            cur_acc << imu_acc[0], imu_acc[1], imu_acc[2];
            cur_gyr << imu_gyr[0], imu_gyr[1], imu_gyr[2];

            mean_acc_ += (cur_acc - mean_acc_) / N;
            mean_gyr_ += (cur_gyr - mean_gyr_) / N;

            cov_acc_ = cov_acc_ * (N - 1.0) / N +
                       (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
            cov_gyr_ = cov_gyr_ * (N - 1.0) / N +
                       (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

            N++;
        }
        acc_norm_ = mean_acc_.norm();
        eskf.state().grav_ = -mean_acc_ / acc_norm_ * G_m_s2_;
        eskf.state().bw_ = mean_gyr_;
        eskf.state().rot_ = Mat3D::Identity();
        eskf.cov() = 0.000001 * StateCov::Identity();
        eskf.initProcessCovQ();
        return;
    }
};

}  // namespace legkilo
#endif  // LEG_KILO_STATE_INITIAL_HPP