#pragma once
#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <istream>
#include <streambuf>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>



struct state_eskf
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);

	Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d bw = Eigen::Vector3d(0,0,0);

	Eigen::Vector3d grav = Eigen::Vector3d(0,0,-9.81);

	Eigen::Vector3d imu_a = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d imu_w = Eigen::Vector3d(0,0,0);

	Eigen::Vector3d foot_FL= Eigen::Vector3d(0,0,0);
	Eigen::Vector3d foot_FR = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d foot_RL = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d foot_RR = Eigen::Vector3d(0,0,0);

};

#define NUM_LEG 4   
#define STATE_SIZE 36   
#define MEAS_SIZE 34   

#define PROCESS_NOISE_b_a 0.001    
#define PROCESS_NOISE_b_w 0.000000000000001    
#define PROCESS_NOISE_IMU_a 0.001    
#define PROCESS_NOISE_IMU_w 0.001    
#define PROCESS_NOISE_PFOOT 0.01    

#define MEASURE_NOISE_IMU_a 0.001    
#define MEASURE_NOISE_IMU_w 0.001     
#define MEASURE_NOISE_P 0.001   
#define MEASURE_NOISE_V  0.1  
#define MEASURE_NOISE_Pi_z 0.05   

class Kinematic{
    public:
        Eigen::Matrix<double, 3, 3>  root_rot_mat;  
        Eigen::Matrix<double, 3, 3>  root_rot_mat_2;  
        Eigen::Quaterniond root_rot_quat;
        unitree_legged_msgs::HighState curMsg;

        Eigen::Matrix<double, 3, NUM_LEG>  foot_pos_rel;  
        Eigen::Vector3d  imu_acc;  
        Eigen::Vector3d  imu_ang_vel;  
        Eigen::Matrix<double, 1, NUM_LEG>  foot_force; 
        Eigen::Matrix<double, 3, NUM_LEG>  foot_vel_rel;  
        Eigen::Matrix<double, 3, NUM_LEG>  foot_angle;  
        Eigen::Matrix<double, 3, NUM_LEG>  foot_angle_vel;  

        Eigen::Vector3d root_pos;
        Eigen::Vector3d root_vel;
         uint8_t movement_mode;
  
        Eigen::Vector3d estimated_root_pos;
        Eigen::Vector3d estimated_root_vel;

        Eigen::Vector4d footContactHeightSmooth;
        Eigen::Vector4d footContactHeightInit;
        std::vector<int> footContactHeightInitBool;
        int footContactHeightInitBool_;
       

        state_eskf x;
        Eigen::Matrix<double,STATE_SIZE,1>delta_x;
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; 
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; 
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F_x ; 
        //Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> F_w ; 
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; 

        Eigen::Matrix<double, MEAS_SIZE, 1> y; 
        Eigen::Matrix<double, MEAS_SIZE, 1> yhat; 
        Eigen::Matrix<double, MEAS_SIZE, 1> error_y; 
        Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y;
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> H; 
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SH; 
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; 


        Eigen::Matrix<double, 3, 3> eye3; 
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; 
        Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; 
        Eigen::Matrix<double,STATE_SIZE,STATE_SIZE> J;

        Eigen::Matrix<double,3,3> last_rot;

        int estimated_contacts[4];

        ros::Time currStamp;
        double currTime;
        double lastTime;

        double curPcStamp;

        bool kdtreeInit = false;

        std::vector<int> CHDcheck;
        std::vector<double> CHDvalue;
        std::vector<double> CHDnoise;


        Eigen::Affine3f lidarOdomAffine;

        int needInit;
        int initCount;
        Eigen::Matrix<double, 3, NUM_LEG>  foot_pos_rel_init; 
        Eigen::Vector3d  imu_acc_init;  
        Eigen::Vector3d  imu_ang_vel_init;
        int useImuOrientation;

        float imuGravity;
        double ox;
        double oy;
        double ot;
        double lc;
        double lt;

        std::string file_name_eskf;
        std::ofstream outfile_eskf;
        double firstTime_eskf;

        double deltaT;

        Kinematic();

        void setParam(const ros::NodeHandle& nh);

        void initStateHandler();

        void msgProcessing();

        void pubLegESKF(ros::Publisher& pubEskf);

        void caculateFootPosVel(const unitree_legged_msgs::HighState::ConstPtr & msg);

        bool getInfoFromMsg(const unitree_legged_msgs::HighState::ConstPtr &msg);

        void stateForward();
        
        void stateUpdate();
        
        Eigen::Matrix3d skew(Eigen::Vector3d imu_ang_vel_);

        Eigen::Matrix3d Exp(const Eigen::Vector3d& r);

        Eigen::Vector3d Log(const Eigen::Matrix3d& R);

        Eigen::Matrix3d A_(const Eigen::Vector3d &u);

};