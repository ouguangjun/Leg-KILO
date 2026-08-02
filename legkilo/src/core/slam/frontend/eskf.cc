#include "core/slam/frontend/eskf.h"

namespace legkilo {

ESKF::ESKF(const EskfProcessNoise& noise) : process_noise_(noise) { rebuildProcessNoise(); }

void ESKF::setPredictModel(PredictModelType predict_model) {
    predict_model_ = predict_model;
    rebuildProcessNoise();
}

StateVec ESKF::getFunctionf(double dt, const PredictInput& input) const {
    switch (predict_model_) {
        case PredictModelType::Inertial: return getFunctionfInertial(dt, input);
        case PredictModelType::ConstantVelocity: return getFunctionfConstantVelocity(dt, input);
    }
    return StateVec::Zero();
}

StateVec ESKF::getFunctionfInertial(double dt, const PredictInput& input) const {
    StateVec vec = StateVec::Zero();
    vec.segment<3>(0) = dt * input.imu_gyr;
    vec.segment<3>(3) = dt * state_.vel_;
    vec.segment<3>(6) = dt * (state_.rot_ * input.imu_acc + state_.grav_);
    return vec;
}

StateVec ESKF::getFunctionfConstantVelocity(double dt, const PredictInput& input) const {
    StateVec vec = StateVec::Zero();
    vec.segment<3>(0) = dt * input.imu_gyr;
    vec.segment<3>(3) = dt * input.world_vel;
    return vec;
}

StateF ESKF::getFx(double dt, const PredictInput& input) const {
    switch (predict_model_) {
        case PredictModelType::Inertial: return getFxInertial(dt, input);
        case PredictModelType::ConstantVelocity: return getFxConstantVelocity(dt, input);
    }
    return StateF::Identity();
}

StateF ESKF::getFxInertial(double dt, const PredictInput& input) const {
    StateF Fx = StateF::Identity();
    Fx.block<3, 3>(0, 0) = Exp(Eigen::Matrix<double, 3, 1>(-dt * input.imu_gyr));
    Fx.block<3, 3>(0, 21) = dt * Mat3D::Identity();
    Fx.block<3, 3>(3, 6) = dt * Mat3D::Identity();
    Fx.block<3, 3>(6, 0) = (-dt) * state_.rot_ * SKEW_SYM_MATRIX(input.imu_acc);
    Fx.block<3, 3>(6, 15) = dt * Mat3D::Identity();
    Fx.block<3, 3>(6, 18) = dt * state_.rot_;
    return Fx;
}

StateF ESKF::getFxConstantVelocity(double dt, const PredictInput& input) const {
    StateF Fx = StateF::Identity();
    Fx.block<3, 3>(0, 0) = Exp(Eigen::Matrix<double, 3, 1>(-dt * input.imu_gyr));
    return Fx;
}

void ESKF::rebuildProcessNoise() {
    Q_.setZero();

    switch (predict_model_) {
        case PredictModelType::Inertial:
            Q_.block<3, 3>(6, 6) = process_noise_.vel_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(9, 9) = process_noise_.acc_bias_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(12, 12) = process_noise_.gyr_bias_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(18, 18) = process_noise_.imu_acc_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(21, 21) = process_noise_.imu_gyr_process_cov * Mat3D::Identity();
            break;
        case PredictModelType::ConstantVelocity:
            Q_.block<3, 3>(0, 0) = process_noise_.imu_gyr_process_cov * Mat3D::Identity();
            Q_.block<3, 3>(3, 3) = process_noise_.vel_process_cov * Mat3D::Identity();
            break;
    }
}

void ESKF::predict(double dt, const PredictInput& input, bool prop_state, bool prop_cov) {
    if (prop_state) { state_ += getFunctionf(dt, input); }
    if (prop_cov) {
        StateF Fx = getFx(dt, input);
        cov_ = Fx * cov_ * Fx.transpose() + (dt * dt) * Q_;
    }
}

void ESKF::updateByPoints(const ObsShared& obs_shared) {
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& z = obs_shared.pt_z;
    const Eigen::Matrix<double, Eigen::Dynamic, 6>& h = obs_shared.pt_h;
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& r = obs_shared.pt_R;

    int dof_measurements = static_cast<int>(h.rows());

    if (dof_measurements == 1) {
        Eigen::Matrix<double, DIM_STATE, 1> PHT = cov_.block<DIM_STATE, 6>(0, 0) * h.transpose();
        double HPHT_R_inv = 1 / (1e-6 + (h * PHT.topRows(6))(0) + r(0));
        Eigen::Matrix<double, DIM_STATE, 1> K = HPHT_R_inv * PHT;
        StateVec delta_x = K * z;
        state_ += delta_x;
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    } else if (dof_measurements <= DIM_STATE) {
        Eigen::MatrixXd PHT = cov_.block<DIM_STATE, 6>(0, 0) * h.transpose();
        Eigen::MatrixXd HPHT_R = h * PHT.topRows(6);
        HPHT_R.diagonal() += r;
        Eigen::MatrixXd K = PHT * HPHT_R.inverse();
        StateVec delta_x = K * z;
        state_ += delta_x;
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    } else {
        Eigen::MatrixXd H_T_R_inv = h.transpose() * r.cwiseInverse().asDiagonal();
        Eigen::MatrixXd P_temp = cov_.inverse();
        P_temp.block<6, 6>(0, 0) += H_T_R_inv * h;
        Eigen::MatrixXd K = P_temp.inverse().block<DIM_STATE, 6>(0, 0) * H_T_R_inv;
        StateVec delta_x = K * z;
        state_ += delta_x;
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    }
}

bool ESKF::updateByCloud(const ObsShared& obs_shared, const State& prior_state, int iter_max, int iter_cur) {
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& z = obs_shared.pt_z;
    const Eigen::Matrix<double, Eigen::Dynamic, 6>& h = obs_shared.pt_h;
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& r = obs_shared.pt_R;

    Eigen::MatrixXd H_T_R_inv = h.transpose() * r.cwiseInverse().asDiagonal();
    Eigen::MatrixXd P_temp = cov_.inverse();
    P_temp.block<6, 6>(0, 0) += H_T_R_inv * h;
    Eigen::MatrixXd K = P_temp.inverse().block<DIM_STATE, 6>(0, 0) * H_T_R_inv;
    StateVec delta_x;
    if (iter_cur == 0) {
        delta_x = K * z;
    } else {
        StateVec delta_prior = prior_state - state_;
        delta_x = K * z + delta_prior - K * h * delta_prior.segment<6>(0);
    }
    state_ += delta_x;
    Vec3D dtheta = delta_x.segment<3>(0);
    Vec3D dpos = delta_x.segment<3>(3);
    bool flag_converge = false;
    if ((dtheta.norm() * 57.3 < 0.1) && (dpos.norm() * 100 < 0.1)) { flag_converge = true; }
    if (flag_converge || (iter_cur >= iter_max - 1)) { cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0); }
    return flag_converge;
}

void ESKF::updateByImu(const ObsShared& obs_shared) {
    // simplify by matlab
    Eigen::Matrix<double, DIM_STATE, 6> PHT = cov_.block<DIM_STATE, 6>(0, 9) + cov_.block<DIM_STATE, 6>(0, 18);
    Eigen::Matrix<double, 6, DIM_STATE> HP = cov_.block<6, DIM_STATE>(9, 0) + cov_.block<6, DIM_STATE>(18, 0);
    Eigen::Matrix<double, 6, 6> HPHT = PHT.block<6, 6>(9, 0) + PHT.block<6, 6>(18, 0);
    HPHT.diagonal() += obs_shared.ki_R;
    Eigen::Matrix<double, DIM_STATE, 6> K = PHT * HPHT.inverse();
    StateVec delta_x = K * obs_shared.ki_z;
    state_ += delta_x;
    cov_ -= K * HP;
}

void ESKF::updateByKinImu(const ObsShared& obs_shared) {
    Eigen::MatrixXd PHT = cov_ * obs_shared.ki_h.transpose();
    Eigen::MatrixXd HPHT = obs_shared.ki_h * PHT;
    HPHT.diagonal() += obs_shared.ki_R;
    Eigen::MatrixXd K = PHT * HPHT.inverse();
    StateVec delta_x = K * obs_shared.ki_z;
    state_ += delta_x;
    cov_ = cov_ - K * obs_shared.ki_h * cov_;
}
}  // namespace legkilo
