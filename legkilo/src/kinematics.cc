#include "kinematics.h"

namespace legkilo {

void Kinematics::processing(const unitree_legged_msgs::HighState& high_state, common::KinImuMeas& kin_imu_meas) {
    kin_imu_meas.time_stamp_ = high_state.stamp.toSec();

    for (int i = 0; i < 3; ++i) {
        kin_imu_meas.acc_[i] = high_state.imu.accelerometer[i];
        kin_imu_meas.gyr_[i] = high_state.imu.gyroscope[i];
    }

    /*  4 legs, 3 moters
        this project leg order: FR FL RR RL
        unitree leg order: FL FR RL RR
    */
    kin_imu_meas.contact_[0] = contacts_[0].update(high_state.footForce[1]);
    kin_imu_meas.contact_[1] = contacts_[1].update(high_state.footForce[0]);
    kin_imu_meas.contact_[2] = contacts_[2].update(high_state.footForce[3]);
    kin_imu_meas.contact_[3] = contacts_[3].update(high_state.footForce[2]);

    double foot_angle[4][3];
    double foot_angle_vel[4][3];
    for (int i = 0; i < 3; ++i) {
        foot_angle[0][i] = high_state.motorState[3 + i].q;
        foot_angle_vel[0][i] = high_state.motorState[3 + i].dq;
        foot_angle[1][i] = high_state.motorState[0 + i].q;
        foot_angle_vel[1][i] = high_state.motorState[0 + i].dq;
        foot_angle[2][i] = high_state.motorState[9 + i].q;
        foot_angle_vel[2][i] = high_state.motorState[9 + i].dq;
        foot_angle[3][i] = high_state.motorState[6 + i].q;
        foot_angle_vel[3][i] = high_state.motorState[6 + i].dq;
    }

    this->caculateFootPosVel(foot_angle, foot_angle_vel, kin_imu_meas.foot_pos_, kin_imu_meas.foot_vel_);

    // static int counts = 0;
    // int index = 1;
    // int uni_index = 0;
    // if(!(counts++  % 50)){
    //     std::cout <<  "contact: " <<kin_imu_meas.contact_[index] << std::endl;
    //     std::cout << "force: " << high_state.footForce[uni_index] << std::endl;
    //     std::cout << "my pos: " << kin_imu_meas.foot_pos_[index][0] << " " << kin_imu_meas.foot_pos_[index][1] << " "
    //     << kin_imu_meas.foot_pos_[index][2] << std::endl; std::cout << "uni pos: " <<
    //     high_state.footPosition2Body[uni_index].x << " " << high_state.footPosition2Body[uni_index].y << " "<<
    //     high_state.footPosition2Body[uni_index].z <<  std::endl; std::cout << "my vel: " <<
    //     kin_imu_meas.foot_vel_[index][0] << " " << kin_imu_meas.foot_vel_[index][1] << " " <<
    //     kin_imu_meas.foot_vel_[index][2] << std::endl; std::cout << "uni vel: " <<
    //     high_state.footSpeed2Body[uni_index].x << " " << high_state.footSpeed2Body[uni_index].y << " " <<
    //     high_state.footSpeed2Body[uni_index].z << std::endl << std::endl;
    // }
}

void Kinematics::caculateFootPosVel(const double (&foot_angle)[4][3], const double (&foot_angle_vel)[4][3],
                                    double (&foot_pos)[4][3], double (&foot_vel)[4][3]) {
    for (int i = 0; i < 4; ++i) {
        int lfoot = -1, ffoot = -1;
        if (i < 2) ffoot = 1;
        if (i == 0 || i == 2) lfoot = 1;

        double s1 = sin(foot_angle[i][0]);
        double s2 = sin(foot_angle[i][1]);
        double s23 = sin(foot_angle[i][1] + foot_angle[i][2]);

        double c1 = cos(foot_angle[i][0]);
        double c2 = cos(foot_angle[i][1]);
        double c23 = cos(foot_angle[i][1] + foot_angle[i][2]);

        foot_pos[i][0] = -lt_ * s2 - lc_ * s23 + ffoot * ox_;
        foot_pos[i][1] = lfoot * d_ * c1 + lc_ * s1 * c23 + lt_ * c2 * s1 + lfoot * oy_;
        foot_pos[i][2] = lfoot * d_ * s1 - lc_ * c1 * c23 - lt_ * c1 * c2;

        double jacb[3][3];
        jacb[0][0] = 0.0;
        jacb[0][1] = -lc_ * c23 - lt_ * c2;
        jacb[0][2] = -lc_ * c23;
        jacb[1][0] = lt_ * c1 * c2 - lfoot * d_ * s1 + lc_ * c1 * c23;
        jacb[1][1] = -s1 * (lc_ * s23 + lt_ * s2);
        jacb[1][2] = -lc_ * s23 * s1;
        jacb[2][0] = lt_ * c2 * s1 + lfoot * d_ * c1 + lc_ * s1 * c23;
        jacb[2][1] = c1 * (lc_ * s23 + lt_ * s2);
        jacb[2][2] = lc_ * s23 * c1;

        foot_vel[i][0] = jacb[0][1] * foot_angle_vel[i][1] + jacb[0][2] * foot_angle_vel[i][2];
        foot_vel[i][1] =
            jacb[1][0] * foot_angle_vel[i][0] + jacb[1][1] * foot_angle_vel[i][1] + jacb[1][2] * foot_angle_vel[i][2];
        foot_vel[i][2] =
            jacb[2][0] * foot_angle_vel[i][0] + jacb[2][1] * foot_angle_vel[i][1] + jacb[2][2] * foot_angle_vel[i][2];
    }
}

}  // namespace legkilo
