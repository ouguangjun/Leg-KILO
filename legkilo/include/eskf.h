#ifndef LEG_KILO_ESKF_H
#define LEG_KILO_ESKF_H

#include "common.hpp"
#include "math_utils.hpp"

namespace legkilo {

constexpr int DIM_STATE = 30;
using Mat3D = Eigen::Matrix<double, 3, 3>;
using Vec3D = Eigen::Matrix<double, 3, 1>;
using StateVec = Eigen::Matrix<double, DIM_STATE, 1>;
using StateCov = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
using StateF = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
using StateQ = Eigen::Matrix<double, DIM_STATE, DIM_STATE>;
struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Mat3D rot_;      // rotation
    Vec3D pos_;      // position
    Vec3D vel_;      // velocity
    Vec3D ba_;       // accelerator bias
    Vec3D bw_;       // gyroscope bias
    Vec3D grav_;     // global gravity
    Vec3D imu_a_;    // acceleration
    Vec3D imu_w_;    // Angular velocity
    Vec3D bv_;       // kinematic velocity bias
    Vec3D contact_;  // contact foot position

    State();
    void operator+=(const StateVec& delta);
    StateVec operator-(const State& other);
};

struct ObsShared {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix<double, Eigen::Dynamic, 1> pt_z;
    Eigen::Matrix<double, Eigen::Dynamic, 6> pt_h;
    Eigen::Matrix<double, Eigen::Dynamic, 1> pt_R;

    Eigen::Matrix<double, Eigen::Dynamic, 1> ki_z;
    Eigen::Matrix<double, Eigen::Dynamic, DIM_STATE> ki_h;
    Eigen::Matrix<double, Eigen::Dynamic, 1> ki_R;
};

class ESKF {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct Config {
        double vel_process_cov;
        double imu_acc_process_cov;
        double imu_gyr_process_cov;
        double contact_process_cov;
        double acc_bias_process_cov;
        double gyr_bias_process_cov;
        double kin_bias_process_cov;

        double imu_acc_meas_noise;
        double imu_acc_z_meas_noise;
        double imu_gyr_meas_noise;
        double kin_meas_noise;
        double chd_meas_noise;
        double contact_meas_noise;
        double lidar_point_meas_ratio;
    };

    ESKF(const Config& config) : config_(config) {}

    State& state() { return state_; }
    const State& state() const { return state_; }
    void setState(const State& state) { state_ = state; }
    Mat3D getRot() { return state_.rot_; }
    Vec3D getPos() { return state_.pos_; }

    StateQ& Q() { return Q_; }
    const StateQ& Q() const { return Q_; }
    void setQ(const StateQ& Q) { Q_ = Q; }

    StateCov& cov() { return cov_; }
    const StateCov& cov() const { return cov_; }

    Config& config() { return config_; }
    const Config& config() const { return config_; }

    void initProcessCovQ();
    // bool initState(const common::MeasGroup& meas, bool pos_init);
    StateVec getFunctionf(double dt);
    StateF getFx(double dt);
    void predict(double dt, bool prop_state, bool prop_cov);
    void updateByPoints(ObsShared& obs_shared);
    void updateByImu(ObsShared& obs_shared);
    void updateByKinImu(ObsShared& obs_shared);

   private:
    ESKF() = default;

    Config config_;
    State state_;
    StateCov cov_;
    StateQ Q_;

    bool init_state = false;
};
}  // namespace legkilo
#endif  // LEG_KILO_ESKF_H