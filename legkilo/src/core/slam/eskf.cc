#include "core/slam/eskf.h"

namespace legkilo {

State::State() {
    rot_ = Mat3D::Identity();
    pos_ = Vec3D::Zero();
    vel_ = Vec3D::Zero();
    ba_ = Vec3D::Zero();
    bw_ = Vec3D::Zero();
    grav_ = Vec3D(0.0, 0.0, -9.81);
    imu_a_ = Vec3D::Zero();
    imu_w_ = Vec3D::Zero();
    bv_ = Vec3D::Zero();
    contact_ = Vec3D::Zero();
}

void State::operator+=(const StateVec& delta) {
    this->rot_ = this->rot_ * Exp(delta(0, 0), delta(1, 0), delta(2, 0));
    this->pos_ += delta.block<3, 1>(3, 0);
    this->vel_ += delta.block<3, 1>(6, 0);
    this->ba_ += delta.block<3, 1>(9, 0);
    this->bw_ += delta.block<3, 1>(12, 0);
    this->grav_ += delta.block<3, 1>(15, 0);
    this->imu_a_ += delta.block<3, 1>(18, 0);
    this->imu_w_ += delta.block<3, 1>(21, 0);
    this->bv_ += delta.block<3, 1>(24, 0);
    this->contact_ += delta.block<3, 1>(27, 0);
}

StateVec State::operator-(const State& other) {
    StateVec delta;
    Mat3D rot_delta = other.rot_.transpose() * this->rot_;
    delta.block<3, 1>(0, 0) = Log(rot_delta);
    delta.block<3, 1>(3, 0) = this->pos_ - other.pos_;
    delta.block<3, 1>(6, 0) = this->vel_ - other.vel_;
    delta.block<3, 1>(9, 0) = this->ba_ - other.ba_;
    delta.block<3, 1>(12, 0) = this->bw_ - other.bw_;
    delta.block<3, 1>(15, 0) = this->grav_ - other.grav_;
    delta.block<3, 1>(18, 0) = this->imu_a_ - other.imu_a_;
    delta.block<3, 1>(21, 0) = this->imu_w_ - other.imu_w_;
    delta.block<3, 1>(24, 0) = this->bv_ - other.bv_;
    delta.block<3, 1>(27, 0) = this->contact_ - other.contact_;
    return delta;
}

void ESKF::initProcessCovQ() {
    Q_.setZero();
    Q_.block<3, 3>(6, 6).diagonal() << config_.vel_process_cov, config_.vel_process_cov, config_.vel_process_cov;
    Q_.block<3, 3>(9, 9).diagonal() << config_.acc_bias_process_cov, config_.acc_bias_process_cov,
        config_.acc_bias_process_cov;
    Q_.block<3, 3>(12, 12).diagonal() << config_.gyr_bias_process_cov, config_.gyr_bias_process_cov,
        config_.gyr_bias_process_cov;
    Q_.block<3, 3>(18, 18).diagonal() << config_.imu_acc_process_cov, config_.imu_acc_process_cov,
        config_.imu_acc_process_cov;
    Q_.block<3, 3>(21, 21).diagonal() << config_.imu_gyr_process_cov, config_.imu_gyr_process_cov,
        config_.imu_gyr_process_cov;
    Q_.block<3, 3>(24, 24).diagonal() << config_.kin_bias_process_cov, config_.kin_bias_process_cov,
        config_.kin_bias_process_cov;
    Q_.block<3, 3>(27, 27).diagonal() << config_.contact_process_cov, config_.contact_process_cov,
        config_.contact_process_cov;
}

StateVec ESKF::getFunctionf(double dt) {
    StateVec vec = StateVec::Zero();
    vec.segment<3>(0) = dt * state_.imu_w_;
    vec.segment<3>(3) = dt * state_.vel_;
    vec.segment<3>(6) = dt * (state_.rot_ * state_.imu_a_ + state_.grav_);
    return vec;
}

StateF ESKF::getFx(double dt) {
    StateF Fx = StateF::Identity();
    Fx.block<3, 3>(0, 0) = Exp(Eigen::Matrix<double, 3, 1>(-dt * state_.imu_w_));
    Fx.block<3, 3>(0, 21) = dt * Mat3D::Identity();
    Fx.block<3, 3>(3, 6) = dt * Mat3D::Identity();
    Fx.block<3, 3>(6, 0) = (-dt) * state_.rot_ * SKEW_SYM_MATRIX(state_.imu_a_);
    Fx.block<3, 3>(6, 15) = dt * Mat3D::Identity();
    Fx.block<3, 3>(6, 18) = dt * state_.rot_;
    return Fx;
}

void ESKF::predict(double dt, bool prop_state, bool prop_cov) {
    if (prop_state) { state_ += getFunctionf(dt); }
    if (prop_cov) {
        StateF Fx = getFx(dt);
        cov_ = Fx * cov_ * Fx.transpose() + (dt * dt) * Q_;
    }
}

void ESKF::updateByPoints(ObsShared& obs_shared) {
    Eigen::Matrix<double, Eigen::Dynamic, 1>& z = obs_shared.pt_z;
    Eigen::Matrix<double, Eigen::Dynamic, 6>& h = obs_shared.pt_h;
    Eigen::Matrix<double, Eigen::Dynamic, 1>& r = obs_shared.pt_R;

    int dof_measurements = static_cast<int>(h.rows());

    if (dof_measurements == 1) {
        Eigen::Matrix<double, DIM_STATE, 1> PHT = cov_.block<DIM_STATE, 6>(0, 0) * h.transpose();
        double HPHT_R_inv = 1 / (0.0001 + (h * PHT.topRows(6))(0) + r(0));
        Eigen::Matrix<double, DIM_STATE, 1> K = HPHT_R_inv * PHT;
        StateVec delta_x = K * z;
        state_ += delta_x;
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    } else {
        Eigen::MatrixXd PHT = cov_.block<DIM_STATE, 6>(0, 0) * h.transpose();
        Eigen::MatrixXd HPHT_R = h * PHT.topRows(6);
        HPHT_R.diagonal() += r;
        Eigen::MatrixXd K = PHT * HPHT_R.inverse();
        StateVec delta_x = K * z;
        state_ += delta_x;
        cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    }

    // else{
    //     Eigen::MatrixXd H_T_R_inv = h.transpose() * r.cwiseInverse().asDiagonal();
    //     Eigen::MatrixXd P_temp = H_T_R_inv * h + cov_.inverse().block<6, 6>(0, 0);
    //     Eigen::MatrixXd K = P_temp.inverse().block<DIM_STATE, 6>(0, 0) * H_T_R_inv;
    //     StateVec delta_x = K * z;
    //     state_ += delta_x;
    //     cov_ = cov_ - K * h * cov_.block<6, DIM_STATE>(0, 0);
    // }
}

void ESKF::updateByImu(ObsShared& obs_shared) {
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

void ESKF::updateByKinImu(ObsShared& obs_shared) {
    Eigen::MatrixXd PHT = cov_ * obs_shared.ki_h.transpose();
    Eigen::MatrixXd HPHT = obs_shared.ki_h * PHT;
    HPHT.diagonal() += obs_shared.ki_R;
    Eigen::MatrixXd K = PHT * HPHT.inverse();
    StateVec delta_x = K * obs_shared.ki_z;
    state_ += delta_x;
    cov_ = cov_ - K * obs_shared.ki_h * cov_;
}
}  // namespace legkilo