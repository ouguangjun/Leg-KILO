#ifndef LEG_KILO_KINEMATICS_H
#define LEG_KILO_KINEMATICS_H

#include "common/common.hpp"

#include <unitree_legged_msgs/HighState.h>

namespace legkilo {

class ContactDetector {
    double T_on_, T_off_;
    bool in_contact_{true};

   public:
    ContactDetector(double ton, double toff) : T_on_(ton), T_off_(toff) {}
    bool update(double val) {
        if (!in_contact_ && val > T_on_)
            in_contact_ = true;
        else if (in_contact_ && val < T_off_)
            in_contact_ = false;
        return in_contact_;
    }
};

class Kinematics {
   public:
    struct Config {
        double leg_offset_x;
        double leg_offset_y;
        double leg_calf_length;
        double leg_thigh_length;
        double leg_thigh_offset;
        double contact_force_threshold_up;
        double contact_force_threshold_down;
    };

    explicit Kinematics(const Config& config)
        : ox_(config.leg_offset_x),
          oy_(config.leg_offset_y),
          lc_(config.leg_calf_length),
          lt_(config.leg_thigh_length),
          d_(config.leg_thigh_offset),
          contacts_(std::vector<ContactDetector>(
              4, ContactDetector(config.contact_force_threshold_up, config.contact_force_threshold_down))) {}

    void processing(const unitree_legged_msgs::HighState& high_state, common::KinImuMeas& kin_imu_meas);

   private:
    void caculateFootPosVel(const double (&foot_angle)[4][3], const double (&foot_angle_vel)[4][3],
                            double (&foot_pos)[4][3], double (&foot_vel)[4][3]);

    double ox_;
    double oy_;
    double lc_;
    double lt_;
    double d_;
    std::vector<ContactDetector> contacts_;
};
}  // namespace legkilo
#endif  // LEG_KILO_KINEMATICS_H