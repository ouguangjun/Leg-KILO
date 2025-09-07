#include "interface/ros1/ros_interface.h"

#include <iomanip>
#include <iostream>
#include <utility>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/KILO.h"
#include "preprocess/kinematics.h"
#include "preprocess/lidar_processing.h"

namespace legkilo {

const bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); }

#define THREAD_SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

RosInterface::RosInterface(ros::NodeHandle& nh) : nh_(nh) {
    LOG(INFO) << "Ros Interface is being Constructed";
    pub_odom_world_ = nh_.advertise<nav_msgs::Odometry>("/Odomtry", 10000);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 10000);
    pub_pointcloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10000);
    pub_pointcloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 10000);
    if (pub_joint_tf_enable_) { pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10000); }

    odom_world_.header.frame_id = "camera_init";
    odom_world_.child_frame_id = "base";
    path_world_.header.frame_id = "camera_init";
    path_world_.header.stamp = ros::Time::now();
    pose_path_.header.frame_id = "camera_init";
}

RosInterface::~RosInterface() {
    LOG(INFO) << "Ros Interface is being Destructed";

    // Shutdown subscribers first to prevent new callbacks
    sub_lidar_raw_.shutdown();
    sub_imu_raw_.shutdown();
    sub_kinematic_raw_.shutdown();

    // Wait a moment for any ongoing callbacks to complete
    usleep(100000);

    // Stop threads
    if (lidar_thread_ && lidar_thread_->joinable()) {
        lidar_thread_->join();
        LOG(INFO) << "Lidar thread stopped";
    }
    if (imu_thread_ && imu_thread_->joinable()) {
        imu_thread_->join();
        LOG(INFO) << "IMU thread stopped";
    }
    if (kinematic_thread_ && kinematic_thread_->joinable()) {
        kinematic_thread_->join();
        LOG(INFO) << "Kinematic thread stopped";
    }
}

bool RosInterface::initParamAndReset(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    /* Topic and options*/
    options::kLidarTopic = yaml_helper.get<std::string>("lidar_topic");
    options::kImuUse = yaml_helper.get<bool>("only_imu_use", true);
    options::kKinAndImuUse = static_cast<bool>(!options::kImuUse);
    options::kRedundancy = yaml_helper.get<bool>("redundancy", false);
    if (options::kImuUse) { options::kImuTopic = yaml_helper.get<std::string>("imu_topic"); }
    if (options::kKinAndImuUse) { options::kKinematicTopic = yaml_helper.get<std::string>("kinematic_topic"); }

    /* Odometry core (KILO) */
    kilo_ = std::make_unique<KILO>(config_file);

    /* kinematics*/
    Kinematics::Config kinematics_config;
    kinematics_config.leg_offset_x = yaml_helper.get<double>("leg_offset_x");
    kinematics_config.leg_offset_y = yaml_helper.get<double>("leg_offset_y");
    kinematics_config.leg_calf_length = yaml_helper.get<double>("leg_calf_length");
    kinematics_config.leg_thigh_length = yaml_helper.get<double>("leg_thigh_length");
    kinematics_config.leg_thigh_offset = yaml_helper.get<double>("leg_thigh_offset");
    kinematics_config.contact_force_threshold_up = yaml_helper.get<double>("contact_force_threshold_up");
    kinematics_config.contact_force_threshold_down = yaml_helper.get<double>("contact_force_threshold_down");
    kinematics_ = std::make_unique<Kinematics>(kinematics_config);

    /* lidar processing*/
    LidarProcessing::Config lidar_process_config;
    lidar_process_config.blind_ = yaml_helper.get<float>("blind");
    lidar_process_config.filter_num_ = yaml_helper.get<int>("filter_num");
    lidar_process_config.time_scale_ = yaml_helper.get<double>("time_scale");
    lidar_process_config.point_stamp_correct_ = yaml_helper.get<bool>("point_stamp_correct", false);
    lidar_process_config.lidar_type_ = static_cast<common::LidarType>(yaml_helper.get<int>("lidar_type"));
    lidar_processing_ = std::make_unique<LidarProcessing>(lidar_process_config);

    /* Visualizaition*/
    pub_joint_tf_enable_ = yaml_helper.get<bool>("pub_joint_tf_enable");

    return true;
}

void RosInterface::rosInit(const std::string& config_file) {
    this->initParamAndReset(config_file);
    this->subscribeLidar();

    if (options::kImuUse) { this->subscribeImu(); }

    if (options::kKinAndImuUse) { this->subscribeKinematicImu(); }
}

void RosInterface::subscribeLidar() {
    this->lidar_thread_ = std::unique_ptr<std::thread>(new std::thread(&RosInterface::lidarLoop, this));
    THREAD_SLEEP(100);
}

void RosInterface::subscribeImu() {
    this->imu_thread_ = std::unique_ptr<std::thread>(new std::thread(&RosInterface::imuLoop, this));
    THREAD_SLEEP(100);
}

void RosInterface::subscribeKinematicImu() {
    this->kinematic_thread_ = std::unique_ptr<std::thread>(new std::thread(&RosInterface::kinematicImuLoop, this));
    THREAD_SLEEP(100);
}

void RosInterface::lidarLoop() {
    LOG(INFO) << "Lidar Loop Begin";

    ros::NodeHandle nh(nh_, "lidar_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_lidar_raw_ =
        nh.subscribe<sensor_msgs::PointCloud2>(options::kLidarTopic, 1000, &RosInterface::lidarCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load()) { queue.callAvailable(ros::WallDuration(0.2)); }
}

void RosInterface::imuLoop() {
    LOG(INFO) << "IMU Loop Begin";

    ros::NodeHandle nh(nh_, "imu_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_imu_raw_ = nh.subscribe(options::kImuTopic, 10000, &RosInterface::imuCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load()) { queue.callAvailable(ros::WallDuration(0.1)); }
}

void RosInterface::kinematicImuLoop() {
    LOG(INFO) << "Kinematic Loop Begin";

    ros::NodeHandle nh(nh_, "kinematic_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_kinematic_raw_ = nh.subscribe(options::kKinematicTopic, 10000, &RosInterface::kinematicImuCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load()) { queue.callAvailable(ros::WallDuration(0.1)); }
}

void RosInterface::lidarCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    static double last_scan_time = msg->header.stamp.toSec();

    Timer::measure("Lidar Processing", [&, this]() {
        if (msg->header.stamp.toSec() < last_scan_time) {
            LOG(WARNING) << "Time inconsistency detected in Lidar data stream";
            lidar_cache_.clear();
        }

        common::LidarScan lidar_scan;
        lidar_processing_->processing(msg, lidar_scan);
        lidar_cache_.push_back(lidar_scan);
        last_scan_time = msg->header.stamp.toSec();
    });

    last_scan_time = msg->header.stamp.toSec();
    return;
}

void RosInterface::imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) {
    static sensor_msgs::Imu last_imu_msg;
    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu(*msg));

    if (options::kRedundancy) {
        if (imu_msg->linear_acceleration.z == last_imu_msg.linear_acceleration.z &&
            imu_msg->angular_velocity.z == last_imu_msg.angular_velocity.z) {
            last_imu_msg = *imu_msg;
            return;
        }
    }

    double timestamp = imu_msg->header.stamp.toSec();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_imu_) {
            LOG(WARNING) << "Time inconsistency detected in Imu data stream";
            imu_cache_.clear();
        }

        imu_cache_.push_back(imu_msg);
        last_imu_msg = *imu_msg;
        last_timestamp_imu_ = timestamp;
    }
    return;
}

void RosInterface::kinematicImuCallBack(const unitree_legged_msgs::HighState::ConstPtr& msg) {
    static unitree_legged_msgs::HighState last_highstate_msg;
    unitree_legged_msgs::HighStatePtr highstate_msg(new unitree_legged_msgs::HighState(*msg));

    if (options::kRedundancy) {
        if (highstate_msg->imu.accelerometer[2] == last_highstate_msg.imu.accelerometer[2] &&
            highstate_msg->imu.gyroscope[2] == last_highstate_msg.imu.gyroscope[2]) {
            last_highstate_msg = *highstate_msg;
            return;
        }
    }

    double timestamp = highstate_msg->stamp.toSec();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timestamp < last_timestamp_kin_imu_) {
            LOG(WARNING) << "Time inconsistency detected in Kin. Imu data stream";
            kin_imu_cache_.clear();
        }

        common::KinImuMeas kin_imu_meas;

        kinematics_->processing(*highstate_msg, kin_imu_meas);

        kin_imu_cache_.push_back(kin_imu_meas);
        last_timestamp_kin_imu_ = timestamp;
        last_highstate_msg = *highstate_msg;
    }

    if (pub_joint_tf_enable_) {
        static std::vector<std::string> joint_names = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = last_highstate_msg.stamp;
        joint_state.name = joint_names;
        const auto& motor = last_highstate_msg.motorState;
        joint_state.position = {
            motor[0].q, motor[1].q, motor[2].q, motor[3].q, motor[4].q,  motor[5].q,
            motor[6].q, motor[7].q, motor[8].q, motor[9].q, motor[10].q, motor[11].q,
        };
        joint_state.velocity = {
            motor[0].dq, motor[1].dq, motor[2].dq, motor[3].dq, motor[4].dq,  motor[5].dq,
            motor[6].dq, motor[7].dq, motor[8].dq, motor[9].dq, motor[10].dq, motor[11].dq,
        };
        pub_joint_state_.publish(joint_state);
    }
    return;
}

bool RosInterface::syncPackage() {
    static bool lidar_push_ = false;

    std::lock_guard<std::mutex> lk(mutex_);

    // pack lidar and  imu
    if (options::kImuUse) {
        if (lidar_cache_.empty() || imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_) { return false; }

        double imu_time = imu_cache_.front()->header.stamp.toSec();
        measure_.imus_.clear();
        while ((!imu_cache_.empty()) && (imu_time < lidar_end_time_)) {
            imu_time = imu_cache_.front()->header.stamp.toSec();
            if (imu_time > lidar_end_time_) break;
            measure_.imus_.push_back(imu_cache_.front());
            imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    // pack lidar, kin. and  imu
    if (options::kKinAndImuUse) {
        if (lidar_cache_.empty() || kin_imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_kin_imu_ < lidar_end_time_) { return false; }

        double kin_imu_time = kin_imu_cache_.front().time_stamp_;
        measure_.kin_imus_.clear();
        while ((!kin_imu_cache_.empty()) && (kin_imu_time < lidar_end_time_)) {
            kin_imu_time = kin_imu_cache_.front().time_stamp_;
            if (kin_imu_time > lidar_end_time_) break;
            measure_.kin_imus_.push_back(kin_imu_cache_.front());
            kin_imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    throw std::runtime_error("Error sync package");
    return false;
}

void RosInterface::publishOdomTFPath(double end_time) {
    // odometry
    odom_world_.header.stamp = ros::Time().fromSec(end_time);
    odom_world_.pose.pose.position.x = kilo_->position()(0);
    odom_world_.pose.pose.position.y = kilo_->position()(1);
    odom_world_.pose.pose.position.z = kilo_->position()(2);
    q_eigen_ = Eigen::Quaterniond(kilo_->getRotMatrix());
    odom_world_.pose.pose.orientation.w = q_eigen_.w();
    odom_world_.pose.pose.orientation.x = q_eigen_.x();
    odom_world_.pose.pose.orientation.y = q_eigen_.y();
    odom_world_.pose.pose.orientation.z = q_eigen_.z();
    pub_odom_world_.publish(odom_world_);

    // tf
    transform_.setOrigin(tf::Vector3(odom_world_.pose.pose.position.x, odom_world_.pose.pose.position.y,
                                     odom_world_.pose.pose.position.z));
    q_tf_.setW(odom_world_.pose.pose.orientation.w);
    q_tf_.setX(odom_world_.pose.pose.orientation.x);
    q_tf_.setY(odom_world_.pose.pose.orientation.y);
    q_tf_.setZ(odom_world_.pose.pose.orientation.z);
    transform_.setRotation(q_tf_);
    br_.sendTransform(tf::StampedTransform(transform_, odom_world_.header.stamp, "camera_init", "base"));

    // path
    pose_path_.header.stamp = odom_world_.header.stamp;
    pose_path_.pose = odom_world_.pose.pose;
    path_world_.poses.push_back(pose_path_);
    pub_path_.publish(path_world_);
}

void RosInterface::publishPointcloudWorld(double end_time) {
    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(*cloud_down_world_, pcl_msg);
    pcl_msg.header.stamp = ros::Time().fromSec(end_time);
    pcl_msg.header.frame_id = "camera_init";
    pub_pointcloud_world_.publish(pcl_msg);
}

void RosInterface::runReset() {
    cloud_raw_.reset(new PointCloudType());
    cloud_down_body_.reset(new PointCloudType());
    cloud_down_world_.reset(new PointCloudType());

    success_pts_size = 0;
}

void RosInterface::run() {
    if (!this->syncPackage()) return;
    this->runReset();

    cloud_raw_ = measure_.lidar_scan_.cloud_;
    double end_time = measure_.lidar_scan_.lidar_end_time_;
    // Delegate odometry to KILO (initialization, downsampling, predict/update, map update)
    if (!kilo_->process(measure_, cloud_down_body_, cloud_down_world_, success_pts_size)) {
        LOG(ERROR) << "KILO processing failed";
        return;
    }

    LOG(INFO) << "pcl raw size:  " << cloud_raw_->points.size()
              << "  pcl down size: " << cloud_down_body_->points.size();
    LOG(INFO) << "useful pcl percent :  " << 100 * ((double)(success_pts_size) / cloud_down_body_->points.size())
              << " %";

    this->publishOdomTFPath(end_time);
    this->publishPointcloudWorld(end_time);

    return;
}

}  // namespace legkilo
