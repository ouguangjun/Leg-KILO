#include "kinematics.h"

Kinematic::Kinematic(){
    currTime = 0;
    lastTime = 0;
    eye3.setIdentity();

    F_x.setZero();
    F_x.setIdentity();

    Q.setZero();
    // Q.block<3,3>(3,3) = 0.01*eye3;
    // Q.block<3,3>(6,6) = 0.01*eye3;
    Q.block<3,3>(9,9) = PROCESS_NOISE_b_a * eye3;
    Q.block<3,3>(12,12) = PROCESS_NOISE_b_w *eye3;
    Q.block<3,3>(18,18) = PROCESS_NOISE_IMU_a *eye3;
    Q.block<3,3>(21,21) = PROCESS_NOISE_IMU_w *eye3;
    Q.block<3,3>(24,24) = PROCESS_NOISE_PFOOT *eye3;
    Q.block<3,3>(27,27) = PROCESS_NOISE_PFOOT *eye3;
    Q.block<3,3>(30,30) = PROCESS_NOISE_PFOOT *eye3;
    Q.block<3,3>(33,33) = PROCESS_NOISE_PFOOT *eye3;

    H.setZero();    //! 
    H.block<3,3>(0,9) = eye3;
    H.block<3,3>(0,18) = eye3;
    H.block<3,3>(3,12) = eye3;
    H.block<3,3>(3,21) = eye3;
    H.block<3,3>(18,6) = eye3;
    H.block<3,3>(21,6) = eye3;
    H.block<3,3>(24,6) = eye3;
    H.block<3,3>(27,6) = eye3;
    H(30,26) = 1;
    H(31,29) = 1;
    H(32,32) = 1;
    H(33,35) = 1;

    R.setZero();
    R.block<3,3>(0,0) = MEASURE_NOISE_IMU_a * eye3;
    R.block<3,3>(3,3) = MEASURE_NOISE_IMU_w * eye3;
    R.block<3,3>(6,6) = MEASURE_NOISE_P * eye3;
    R.block<3,3>(9,9) = MEASURE_NOISE_P * eye3;
    R.block<3,3>(12,12) = MEASURE_NOISE_P * eye3;
    R.block<3,3>(15,15) = MEASURE_NOISE_P * eye3;
    R.block<3,3>(18,18) = MEASURE_NOISE_V * eye3;
    R.block<3,3>(21,21) = MEASURE_NOISE_V * eye3;
    R.block<3,3>(24,24) = MEASURE_NOISE_V * eye3;
    R.block<3,3>(27,27) = MEASURE_NOISE_V * eye3;
    R(30,30) = MEASURE_NOISE_Pi_z;
    R(31,31) = MEASURE_NOISE_Pi_z;
    R(32,32) = MEASURE_NOISE_Pi_z;
    R(33,33) = MEASURE_NOISE_Pi_z;

    P.setIdentity();
    Pbar.setIdentity();
    J.setIdentity();

    last_rot.setIdentity();

    needInit = 1;
    initCount = 0;
    useImuOrientation = 1;
    foot_pos_rel_init.setZero();
    imu_acc_init.setZero();
    imu_ang_vel_init.setZero();
    footContactHeightSmooth.setZero();
    footContactHeightInit.setZero();
    footContactHeightInitBool.assign(4, 1);
    footContactHeightInitBool_ = 1;

    CHDcheck.assign(4, 0);
    CHDvalue.assign(4, 0.0);
    CHDnoise.assign(4, 0.0);



    file_name_eskf = "/home/jason/legkilov2/eskf.csv";
    firstTime_eskf = 1684320000;
    outfile_eskf.open(file_name_eskf, std::ios::out | std::ios::trunc);
}

void Kinematic::setParam(const ros::NodeHandle& nh){
    nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
    nh.param<double>("lio_sam/legOffsetX", ox, 0.1881 );
    nh.param<double>("lio_sam/legOffsetY", oy, 0.04675 );
    nh.param<double>("lio_sam/legCalfLength", lc, 0.213 );
    nh.param<double>("lio_sam/legThighLength", lt, 0.213 );
    nh.param<double>("lio_sam/legThighOffset", ot, 0.08 );
}

void Kinematic::initStateHandler(){
    ++initCount;
    imu_acc_init += (imu_acc - imu_acc_init) / initCount;
    imu_ang_vel_init += (imu_ang_vel - imu_ang_vel_init) / initCount;
    foot_pos_rel_init += (foot_pos_rel - foot_pos_rel_init) / initCount;

    if(initCount < 10){
        return;
    }else{
        x.grav = - imu_acc_init /  imu_acc_init.norm() * imuGravity;
        x.bw = imu_ang_vel_init;
        x.foot_FL = foot_pos_rel_init.block<3, 1>(0, 0);
        x.foot_FR = foot_pos_rel_init.block<3, 1>(0, 1);
        x.foot_RL = foot_pos_rel_init.block<3, 1>(0, 2);
        x.foot_RR = foot_pos_rel_init.block<3, 1>(0, 3);

        for(int i = 0; i < 4; ++i){
            footContactHeightSmooth(i) = foot_pos_rel_init(2, i);
        }

        needInit = 0;
    }
}

void Kinematic::msgProcessing(){
    //getInfoFromMsg(msg);

    currTime = currStamp.toSec();
    deltaT = lastTime == 0 ? 0.002 : currTime-lastTime;
    lastTime = currTime;

    static bool is_firstone = true;
    if(is_firstone){
        last_rot = root_rot_mat;
        is_firstone = false;
    }
    root_rot_mat_2 = last_rot.transpose() * root_rot_mat;
    imu_ang_vel = Log(root_rot_mat_2)/deltaT;
    last_rot = root_rot_mat;
}

void Kinematic::stateForward(){
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        Q.block<3,3>(24+i*3,24+i*3) = (1 + (1 - estimated_contacts[i]) * 1e3) * deltaT*PROCESS_NOISE_PFOOT * eye3;
    }

    F_x.block<3,3>(0,0) = Exp(-x.imu_w*deltaT);
    F_x.block<3,3>(0,21) = deltaT*eye3;
    F_x.block<3,3>(3,6) = deltaT*eye3;
    F_x.block<3,3>(6,0) = -x.rot * skew(x.imu_a) * deltaT;
    F_x.block<3,3>(6,15) = deltaT*eye3; 
    F_x.block<3,3>(6,18) = x.rot * deltaT;
    F_x.block<3,3>(12,12) = 0 * eye3;

    P = F_x * Pbar * F_x.transpose() + Q; 

    x.rot = x.rot * Exp(x.imu_w * deltaT) ;
    x.pos = x.pos +x.vel * deltaT;
    x.vel = x.vel+ (x.rot * x.imu_a + x.grav)*deltaT;
}

void Kinematic::stateUpdate(){
    if(footContactHeightInitBool_){
        int check = 0;
        for(int i = 0; i < 4; ++i){
            if(footContactHeightInitBool[i] && CHDcheck[i]){
                footContactHeightInit(i) = CHDvalue[i];
                footContactHeightInitBool[i] = 0;
            }
            if(!footContactHeightInitBool[i]) check++;
        }
        if(check == 4) footContactHeightInitBool_ = 0;
    }else{
        for(int i = 0; i < 4; ++i){
            if(CHDcheck[i]){
                footContactHeightSmooth(i) = 0.5 * footContactHeightSmooth(i) + 
                                             0.5 * (foot_pos_rel_init(2, i) + CHDvalue[i] - footContactHeightInit(i));
            }
        }
    }

    //std::cout << " chd smooth:   " << std::endl << footContactHeightSmooth << std::endl;
    // std::cout << " chd value:  "  << CHDvalue[0] << " " <<  CHDvalue[1] << " " << CHDvalue[2] << " " << CHDvalue[3] << std::endl;
    // std::cout << " chd noise:  " << CHDnoise[0] << " " <<  CHDnoise[1] << " " << CHDnoise[2] << " " << CHDnoise[3] << std::endl;


    
    Eigen::Matrix3d rot_T = x.rot.transpose();
    Eigen::Vector3d H31_61 = rot_T * (x.foot_FL - x.pos);
    H.block<3,3>(6,0) = skew(H31_61);
    H31_61 = rot_T * (x.foot_FR - x.pos);
    H.block<3,3>(9,0) = skew(H31_61);
    H31_61 = rot_T * (x.foot_RL - x.pos);
    H.block<3,3>(12,0) = skew(H31_61);
    H31_61 = rot_T * (x.foot_RR - x.pos);
    H.block<3,3>(15,0) = skew(H31_61);

    H.block<3,3>(6,3) = -rot_T; H.block<3,3>(6,24) = rot_T;
    H.block<3,3>(9,3) = -rot_T; H.block<3,3>(9,27) = rot_T;
    H.block<3,3>(12,3) = -rot_T; H.block<3,3>(12,30) = rot_T;
    H.block<3,3>(15,3) = -rot_T; H.block<3,3>(15,33) = rot_T;

    H.block<3,3>(18,0) = - x.rot * skew(skew(x.imu_w)*foot_pos_rel.block<3,1>(0,0)+foot_vel_rel.block<3,1>(0,0));
    H.block<3,3>(21,0) = - x.rot * skew(skew(x.imu_w)*foot_pos_rel.block<3,1>(0,1)+foot_vel_rel.block<3,1>(0,1));
    H.block<3,3>(24,0) = - x.rot * skew(skew(x.imu_w)*foot_pos_rel.block<3,1>(0,2)+foot_vel_rel.block<3,1>(0,2));
    H.block<3,3>(27,0) = - x.rot * skew(skew(x.imu_w)*foot_pos_rel.block<3,1>(0,3)+foot_vel_rel.block<3,1>(0,3));

    H.block<3,3>(18,21) = -x.rot * skew(foot_pos_rel.block<3,1>(0,0));
    H.block<3,3>(21,21) = -x.rot * skew(foot_pos_rel.block<3,1>(0,1));
    H.block<3,3>(24,21) = -x.rot * skew(foot_pos_rel.block<3,1>(0,2));
    H.block<3,3>(27,21) = -x.rot * skew(foot_pos_rel.block<3,1>(0,3));

    error_y.segment<3>(0) = imu_acc -  x.imu_a - x.ba;
    error_y.segment<3>(3) = imu_ang_vel - x.imu_w - x.bw;
    //error_y.segment<3>(3) = imu_ang_vel - x.imu_w;
    error_y.segment<3>(6) = foot_pos_rel.block<3,1>(0,0) - rot_T * (x.foot_FL - x.pos);
    error_y.segment<3>(9) = foot_pos_rel.block<3,1>(0,1)- rot_T * (x.foot_FR - x.pos);
    error_y.segment<3>(12) = foot_pos_rel.block<3,1>(0,2) - rot_T * (x.foot_RL - x.pos);
    error_y.segment<3>(15) = foot_pos_rel.block<3,1>(0,3) - rot_T * (x.foot_RR - x.pos);
    error_y.segment<3>(18) = - x.vel - x.rot * (skew(x.imu_w)*foot_pos_rel.block<3,1>(0,0)+foot_vel_rel.block<3,1>(0,0));
    error_y.segment<3>(21) = - x.vel - x.rot * (skew(x.imu_w)*foot_pos_rel.block<3,1>(0,1)+foot_vel_rel.block<3,1>(0,1));
    error_y.segment<3>(24) = - x.vel - x.rot * (skew(x.imu_w)*foot_pos_rel.block<3,1>(0,2)+foot_vel_rel.block<3,1>(0,2));
    error_y.segment<3>(27) = - x.vel - x.rot * (skew(x.imu_w)*foot_pos_rel.block<3,1>(0,3)+foot_vel_rel.block<3,1>(0,3));

    error_y(30) =   estimated_contacts[0] * footContactHeightSmooth(0) + (1.0-estimated_contacts[0])*(x.pos(2)+foot_pos_rel(2,0))- x.foot_FL(2);
    error_y(31) =   estimated_contacts[1] * footContactHeightSmooth(1) + (1.0-estimated_contacts[1])*(x.pos(2)+foot_pos_rel(2,1))- x.foot_FR(2);
    error_y(32) =   estimated_contacts[2] * footContactHeightSmooth(2) + (1.0-estimated_contacts[2])*(x.pos(2)+foot_pos_rel(2,2))- x.foot_RL(2);
    error_y(33) =   estimated_contacts[3] * footContactHeightSmooth(3) + (1.0-estimated_contacts[3])*(x.pos(2)+foot_pos_rel(2,3))- x.foot_RR(2);


    for (int i = 0; i < NUM_LEG; ++i) 
    {
        R.block<3,3>(18+i*3,18+i*3) = (1 + (1 - estimated_contacts[i]) * 1e4) * MEASURE_NOISE_V * eye3;
        double footContactHeightNoise = CHDcheck[i] ? 100 * CHDnoise[i] : MEASURE_NOISE_Pi_z;
        R(30+i,30+i) = (1 + (1 - estimated_contacts[i]) * 1e4) * footContactHeightNoise;

    }

    for(int i =0; i<3;++i)
    {
        if(fabs(imu_acc[i]) > 40 ){
            R(i,i) = 1e4;
        }
    }
    
    S =  H * P * H.transpose() + R;
    K = (P*H.transpose())* S.inverse();

    delta_x  = K*error_y; 
    Pbar = P - K*H*P;

    x.rot = x.rot * Exp(delta_x.segment<3>(0));
    x.pos = x.pos + delta_x.segment<3>(3);
    x.vel = x.vel + delta_x.segment<3>(6);
    x.ba = x.ba + delta_x.segment<3>(9);
    //x.ba = x.ba ;

    x.bw = x.bw + delta_x.segment<3>(12);
    //x.bw = x.bw;
    x.grav = x.grav;
    x.imu_a = x.imu_a  +  delta_x.segment<3>(18);
    x.imu_w = x.imu_w +  delta_x.segment<3>(21);
    //x.imu_w = x.imu_w
    x.foot_FL = x.foot_FL +  delta_x.segment<3>(24);
    x.foot_FR = x.foot_FR +  delta_x.segment<3>(27);
    x.foot_RL = x.foot_RL +  delta_x.segment<3>(30);
    x.foot_RR = x.foot_RR + delta_x.segment<3>(33);
    

    estimated_root_pos = x.pos;    
    estimated_root_vel = x.vel;

    outfile_eskf << currTime-firstTime_eskf <<","<< x.pos[0] <<","<<x.pos[1] <<","<<x.pos[2]  <<","
                    <<x.rot(0,0) <<"," <<x.rot(0,1) <<","<<x.rot(0,2) <<","
                    <<x.rot(1,0) <<","<<x.rot(1,1) <<","<<x.rot(1,2) <<","
                    <<x.rot(2,0) <<","<<x.rot(2,1) <<","<<x.rot(2,2) <<std::endl;

}


void Kinematic::pubLegESKF(ros::Publisher& pubEskf){


        Eigen::Quaterniond q = useImuOrientation ? root_rot_quat : Eigen::Quaterniond(x.rot);
        q.normalize(); 

        nav_msgs::Odometry ori_state;
        ori_state.header.stamp = currStamp;
        ori_state.header.frame_id = "eskf_odom";
        ori_state.child_frame_id = "eskf_base_link_3d";
        ori_state.pose.pose.orientation.x = q.x();
        ori_state.pose.pose.orientation.y = q.y();
        ori_state.pose.pose.orientation.z = q.z();
        ori_state.pose.pose.orientation.w = q.w();

        ori_state.pose.pose.position.x = x.pos[0];
        ori_state.pose.pose.position.y = x.pos[1];
        ori_state.pose.pose.position.z = x.pos[2];

        ori_state.twist.twist.linear.x = x.vel[0];
        ori_state.twist.twist.linear.y = x.vel[1];
        ori_state.twist.twist.linear.z = x.vel[2];

        ori_state.twist.twist.angular.x = x.imu_w[0];
        ori_state.twist.twist.angular.y = x.imu_w[1];
        ori_state.twist.twist.angular.z = x.imu_w[2];
        pubEskf.publish(ori_state);

}

void Kinematic::caculateFootPosVel(const unitree_legged_msgs::HighState::ConstPtr &msg){
    for(int i = 0; i < 4; ++i){
        int lfoot = -1, ffoot = -1;
        if(i < 2) ffoot = 1;
        if(i == 0 || i == 2) lfoot = 1;

        double s1 = sin(foot_angle(0, i));
        double s2 = sin(foot_angle(1, i));
        double s23 = sin(foot_angle(1, i) + foot_angle(2, i));

        double c1 = cos(foot_angle(0, i));
        double c2 = cos(foot_angle(1, i));
        double c23 = cos(foot_angle(1, i) + foot_angle(2, i));

        foot_pos_rel(0, i) = - lt * s2 - lc * s23 + ffoot * ox;
        foot_pos_rel(1, i) = lfoot * ot * c1 + lc * s1 * c23 + lt * c2 * s1 + lfoot * oy;
        foot_pos_rel(2, i) = lfoot * ot * s1 - lc * c1 * c23 - lt * c1 * c2;

        Eigen::Matrix3d J;
        J(0, 0) = 0.0;
        J(0, 1) = - lc * c23 - lt * c2;
        J(0, 2) = - lc * c23;
        J(1, 0) = lt * c1 * c2 - lfoot * ot * s1 + lc * c1 * c23;
        J(1, 1) = - s1 * (lc * s23 + lt * s2);
        J(1, 2) = - lc * s23 * s1;
        J(2, 0) = lt * c2 * s1 + lfoot * ot * c1 + lc * s1 * c23;
        J(2, 1) = c1 * (lc * s23 + lt * s2);
        J(2, 2) = lc * s23 * c1; 

        foot_vel_rel(0, i) =                                  J(0, 1) * foot_angle_vel(1, i) + J(0, 2) * foot_angle_vel(2, i);
        foot_vel_rel(1, i) = J(1, 0) * foot_angle_vel(0, i) + J(1, 1) * foot_angle_vel(1, i) + J(1, 2) * foot_angle_vel(2, i);
        foot_vel_rel(2, i) = J(2, 0) * foot_angle_vel(0, i) + J(2, 1) * foot_angle_vel(1, i) + J(2, 2) * foot_angle_vel(2, i);

    }
}

bool Kinematic::getInfoFromMsg(const unitree_legged_msgs::HighState::ConstPtr &msg){
        
        if (msg->imu.accelerometer[2] == 0) return false;

        // if(curMsg.imu.accelerometer[0] == msg->imu.accelerometer[0]) return false;
        // curMsg = *msg;

        Eigen::Vector3d euler(msg->imu.rpy[2],msg->imu.rpy[1],msg->imu.rpy[0]);  // 对应 z y x
        root_rot_mat = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());

        foot_angle << msg->motorState[3].q, msg->motorState[0].q, msg->motorState[9].q, msg->motorState[6].q, 
                      msg->motorState[4].q, msg->motorState[1].q, msg->motorState[10].q, msg->motorState[7].q, 
                      msg->motorState[5].q, msg->motorState[2].q, msg->motorState[11].q, msg->motorState[8].q;

        foot_angle_vel << msg->motorState[3].dq, msg->motorState[0].dq, msg->motorState[9].dq, msg->motorState[6].dq, 
                          msg->motorState[4].dq, msg->motorState[1].dq, msg->motorState[10].dq, msg->motorState[7].dq, 
                          msg->motorState[5].dq, msg->motorState[2].dq, msg->motorState[11].dq, msg->motorState[8].dq;


        caculateFootPosVel(msg);

        imu_acc << msg->imu.accelerometer[0], msg->imu.accelerometer[1], msg->imu.accelerometer[2];
        imu_acc = imu_acc /imu_acc.norm() * imuGravity;
        imu_ang_vel << msg->imu.gyroscope[0],  msg->imu.gyroscope[1],  msg->imu.gyroscope[2];

        foot_force << msg->footForce[1],  msg->footForce[0], msg->footForce[3], msg->footForce[2];

        root_pos << msg->position[0], msg->position[1], msg->position[2];
        root_vel << msg->velocity[0], msg->velocity[1], msg->velocity[2];

        root_rot_quat.x() = msg->imu.quaternion[0];
        root_rot_quat.y() = msg->imu.quaternion[1];
        root_rot_quat.z() = msg->imu.quaternion[2];
        root_rot_quat.w() = msg->imu.quaternion[3];


        currStamp = msg->stamp;

        for(int i=0; i< NUM_LEG; i++){
            if(foot_force(i) > 200) estimated_contacts[i] = 1;
            else estimated_contacts[i] = 0;
        }

        return true;
}


Eigen::Matrix3d Kinematic::skew(Eigen::Vector3d imu_ang_vel_)
    {
        Eigen::Matrix3d skewresult;
        skewresult << 0, -imu_ang_vel_[2], imu_ang_vel_[1],
                                    imu_ang_vel_[2], 0, -imu_ang_vel_[0],
                                     -imu_ang_vel_[1], imu_ang_vel_[0], 0;
        return skewresult;
    }


Eigen::Matrix3d Kinematic::Exp(const Eigen::Vector3d& r){
    Eigen::Matrix3d expr;
    double theta = r.norm();
    if(theta < 1e-12){
        expr = Eigen::Matrix3d::Identity();
    }
    else{
        Eigen::Matrix3d skew_ = skew(r / theta);
        expr = Eigen::Matrix3d::Identity() + sin(theta) * skew_ + (1 - cos(theta)) * skew_ * skew_;
    }
    return expr;
}

Eigen::Vector3d Kinematic::Log(const Eigen::Matrix3d& R){
    double theta = (R.trace() > 3 - 1e-6) ? 0 : acos((R.trace() - 1) / 2);
    //double theta =  acos((R.trace() - 1) / 2);

    Eigen::Vector3d r(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return fabs(theta) < 0.001 ? (0.5 * r) : (0.5 * theta / sin(theta) * r);
    //return 0.5 * theta / sin(theta) * r;

}   

Eigen::Matrix3d Kinematic::A_(const Eigen::Vector3d &u){
    Eigen::Matrix3d result;
    Eigen::Matrix3d skew_u = skew(u);
    double u_norm = u.norm();
    double a = 0.5 * u_norm * cos(0.5*u_norm) / sin(0.5 * u_norm);
    result = eye3  - 0.5 * skew_u + ((1-a)/(u_norm * u_norm)) *skew_u * skew_u ;

    return result;
}