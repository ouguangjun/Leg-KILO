#include "core/slam/frontend/State.h"

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
}

StateVec State::operator-(const State& other) const {
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
    return delta;
}

}  // namespace legkilo
