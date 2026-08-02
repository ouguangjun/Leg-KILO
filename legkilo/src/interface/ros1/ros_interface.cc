#include "interface/ros1/ros_interface.h"

#include <iomanip>
#include <iostream>
#include <utility>

#include <ros/callback_queue.h>

#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/backend/backend.h"
#include "core/slam/frontend/KILO.h"
#include "preprocess/kinematics.h"
#include "preprocess/lidar_processing.h"
#include "viewer/viewer_slam_interface.h"

namespace legkilo {

const bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); }

#define THREAD_SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

RosInterface::RosInterface(ros::NodeHandle& nh, ViewerSlamInterface* viewer_interface)
    : nh_(nh), viewer_interface_(viewer_interface) {
    LOG(INFO) << "Ros Interface is being Constructed";
    pub_odom_world_ = nh_.advertise<ros_compat::OdometryMsg>("/Odometry", 5);
    pub_path_ = nh_.advertise<ros_compat::PathMsg>("/path", 5);
    pub_pointcloud_world_ = nh_.advertise<ros_compat::PointCloud2Msg>("/cloud_registered", 5);
    pub_pointcloud_body_ = nh_.advertise<ros_compat::PointCloud2Msg>("/cloud_registered_body", 5);
    if (pub_joint_tf_enable_) { pub_joint_state_ = nh_.advertise<ros_compat::JointStateMsg>("/joint_states", 5); }
}

RosInterface::~RosInterface() {
    LOG(INFO) << "Ros Interface is being Destructed";

    this->stop();
}

bool RosInterface::initParamAndReset(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    /* Topic and options*/
    options::kLidarTopic = yaml_helper.get<std::string>("lidar_topic");
    options::kSensorType = common::parseSensorType(yaml_helper.get<std::string>("sensor_type"));
    options::kRedundancy = yaml_helper.get<bool>("redundancy", false);
    if (options::useImu()) { options::kImuTopic = yaml_helper.get<std::string>("imu_topic"); }
    if (options::useKinematics()) { options::kKinematicTopic = yaml_helper.get<std::string>("kinematic_topic"); }

    /* frontend Odometry (KILO) */
    kilo_ = std::make_unique<KILO>(config_file);

    /* backend */
    backend_ = std::make_unique<Backend>(config_file);
    backend_->setViewerInterface(viewer_interface_);
    backend_->start();

    /* kinematics*/
    if (options::useKinematics()) {
        Kinematics::Config kinematics_config;
        kinematics_config.leg_offset_x = yaml_helper.get<double>("leg_offset_x");
        kinematics_config.leg_offset_y = yaml_helper.get<double>("leg_offset_y");
        kinematics_config.leg_calf_length = yaml_helper.get<double>("leg_calf_length");
        kinematics_config.leg_thigh_length = yaml_helper.get<double>("leg_thigh_length");
        kinematics_config.leg_thigh_offset = yaml_helper.get<double>("leg_thigh_offset");
        kinematics_config.contact_force_threshold_up = yaml_helper.get<double>("contact_force_threshold_up");
        kinematics_config.contact_force_threshold_down = yaml_helper.get<double>("contact_force_threshold_down");
        kinematics_ = std::make_unique<Kinematics>(kinematics_config);
    }
    /* lidar processing*/
    LidarProcessing::Config lidar_process_config;
    lidar_process_config.min_range_ = yaml_helper.get<float>("min_range", 0.1f);
    lidar_process_config.max_range_ = yaml_helper.get<float>("max_range", 100.0f);
    lidar_process_config.filter_num_ = yaml_helper.get<int>("filter_num", 1);
    lidar_process_config.time_scale_ = yaml_helper.get<double>("time_scale");
    lidar_process_config.lidar_type_ = static_cast<common::LidarType>(yaml_helper.get<int>("lidar_type"));
    lidar_processing_ = std::make_unique<LidarProcessing>(lidar_process_config);

    /* Visualizaition*/
    pub_joint_tf_enable_ = options::useKinematics() ? yaml_helper.get<bool>("pub_joint_tf_enable", false) : false;

    return true;
}

void RosInterface::init(const std::string& config_file) {
    this->initParamAndReset(config_file);
    this->subscribeLidar();

    if (options::useImu()) { this->subscribeImu(); }

    if (options::useKinematics()) { this->subscribeKinematicImu(); }
}

void RosInterface::stop() {
    options::FLAG_EXIT.store(true);
    this->stopSlam();
}

void RosInterface::stopSlam() {
    if (slam_stopped_.exchange(true, std::memory_order_acq_rel)) return;

    // Shutdown subscribers first to prevent new callbacks
    sub_lidar_raw_.shutdown();
    sub_imu_raw_.shutdown();
    sub_kinematic_raw_.shutdown();

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

    if (backend_) {
        backend_->stop();
        LOG(INFO) << "Backend stopped";
    }
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
    if (lidar_processing_->getLidarType() == common::LidarType::LIVOX) {
        this->sub_lidar_raw_ = nh.subscribe<ros_compat::LivoxCustomMsg>(options::kLidarTopic, 1000,
                                                                        &RosInterface::livoxLidarCallBack, this);
    } else {
        this->sub_lidar_raw_ =
            nh.subscribe<ros_compat::PointCloud2Msg>(options::kLidarTopic, 1000, &RosInterface::lidarCallBack, this);
    }

    while (ros::ok() && !options::FLAG_EXIT.load() && !slam_stopped_.load(std::memory_order_acquire)) {
        queue.callAvailable(ros::WallDuration(0.2));
    }
}

void RosInterface::imuLoop() {
    LOG(INFO) << "IMU Loop Begin";

    ros::NodeHandle nh(nh_, "imu_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_imu_raw_ = nh.subscribe(options::kImuTopic, 10000, &RosInterface::imuCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load() && !slam_stopped_.load(std::memory_order_acquire)) {
        queue.callAvailable(ros::WallDuration(0.1));
    }
}

void RosInterface::kinematicImuLoop() {
    LOG(INFO) << "Kinematic Loop Begin";

    ros::NodeHandle nh(nh_, "kinematic_sub");
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);
    this->sub_kinematic_raw_ = nh.subscribe(options::kKinematicTopic, 10000, &RosInterface::kinematicImuCallBack, this);

    while (ros::ok() && !options::FLAG_EXIT.load() && !slam_stopped_.load(std::memory_order_acquire)) {
        queue.callAvailable(ros::WallDuration(0.1));
    }
}

void RosInterface::lidarCallBack(const ros_compat::PointCloud2MsgConstPtr& msg) {
    const double scan_time = ros_compat::toSec(msg->header.stamp);
    common::LidarScan lidar_scan;

    Timer::measure("Lidar Processing", [&, this]() { lidar_processing_->processing(msg, lidar_scan); });

    std::lock_guard<std::mutex> lock(mutex_);
    static double last_scan_time = scan_time;
    if (scan_time < last_scan_time) {
        LOG(WARNING) << "Time inconsistency detected in Lidar data stream";
        lidar_cache_.clear();
    }

    lidar_cache_.push_back(std::move(lidar_scan));
    last_scan_time = scan_time;
}

void RosInterface::livoxLidarCallBack(const ros_compat::LivoxCustomMsgConstPtr& msg) {
    common::LidarScan lidar_scan;

    Timer::measure("Lidar Processing", [&, this]() { lidar_processing_->processing(msg, lidar_scan); });

    const double scan_time = lidar_scan.lidar_begin_time_;
    std::lock_guard<std::mutex> lock(mutex_);
    static double last_scan_time = scan_time;
    if (scan_time < last_scan_time) {
        LOG(WARNING) << "Time inconsistency detected in Livox data stream";
        lidar_cache_.clear();
    }

    lidar_cache_.push_back(std::move(lidar_scan));
    last_scan_time = scan_time;
}

void RosInterface::imuCallBack(const ros_compat::ImuMsgConstPtr& msg) {
    static ros_compat::ImuMsg last_imu_msg;
    ros_compat::ImuMsgPtr imu_msg(new ros_compat::ImuMsg(*msg));

    if (options::kRedundancy) {
        if (imu_msg->linear_acceleration.z == last_imu_msg.linear_acceleration.z &&
            imu_msg->angular_velocity.z == last_imu_msg.angular_velocity.z) {
            last_imu_msg = *imu_msg;
            return;
        }
    }

    double timestamp = ros_compat::toSec(imu_msg->header.stamp);
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

void RosInterface::kinematicImuCallBack(const ros_compat::HighStateMsgConstPtr& msg) {
    static ros_compat::HighStateMsg last_highstate_msg;
    ros_compat::HighStateMsgPtr highstate_msg(new ros_compat::HighStateMsg(*msg));

    if (options::kRedundancy) {
        if (highstate_msg->imu.accelerometer[2] == last_highstate_msg.imu.accelerometer[2] &&
            highstate_msg->imu.gyroscope[2] == last_highstate_msg.imu.gyroscope[2]) {
            last_highstate_msg = *highstate_msg;
            return;
        }
    }

    double timestamp = ros_compat::toSec(highstate_msg->stamp);
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
        ros_compat::JointStateMsg joint_state;
        joint_state.header.stamp = last_highstate_msg.stamp;
        joint_state.name = joint_names;
        const auto& motor = ros_compat::motorStates(last_highstate_msg);
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

    // pack lidar only
    if (options::useLidarOnly()) {
        if (lidar_cache_.empty()) return false;

        measure_.lidar_scan_ = lidar_cache_.front();
        measure_.imus_.clear();
        measure_.kin_imus_.clear();
        lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
        lidar_push_ = false;
        lidar_cache_.pop_front();
        return true;
    }

    // pack lidar and  imu
    if (options::useImu()) {
        if (lidar_cache_.empty() || imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_) { return false; }

        double imu_time = ros_compat::toSec(imu_cache_.front()->header.stamp);
        measure_.imus_.clear();
        measure_.kin_imus_.clear();
        while ((!imu_cache_.empty()) && (imu_time < lidar_end_time_)) {
            imu_time = ros_compat::toSec(imu_cache_.front()->header.stamp);
            if (imu_time > lidar_end_time_) break;
            measure_.imus_.push_back(imu_cache_.front());
            imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    // pack lidar, kin. and  imu
    if (options::useKinematics()) {
        if (lidar_cache_.empty() || kin_imu_cache_.empty()) return false;

        if (!lidar_push_) {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_kin_imu_ < lidar_end_time_) { return false; }

        double kin_imu_time = kin_imu_cache_.front().time_stamp_;
        measure_.imus_.clear();
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

void RosInterface::publishOdomTFPath(double end_time, const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) {
    // odometry
    ros_compat::OdometryMsg odom_world;
    odom_world.header.stamp = ros_compat::fromSec(end_time);
    odom_world.header.frame_id = "camera_init";
    odom_world.child_frame_id = "base";
    odom_world.pose.pose.position.x = pos(0);
    odom_world.pose.pose.position.y = pos(1);
    odom_world.pose.pose.position.z = pos(2);
    Eigen::Quaterniond q_eigen(rot);
    odom_world.pose.pose.orientation.w = q_eigen.w();
    odom_world.pose.pose.orientation.x = q_eigen.x();
    odom_world.pose.pose.orientation.y = q_eigen.y();
    odom_world.pose.pose.orientation.z = q_eigen.z();
    pub_odom_world_.publish(odom_world);

    // tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
    tf::Quaternion q_tf;
    q_tf.setW(q_eigen.w());
    q_tf.setX(q_eigen.x());
    q_tf.setY(q_eigen.y());
    q_tf.setZ(q_eigen.z());
    transform.setRotation(q_tf);
    br.sendTransform(tf::StampedTransform(transform, odom_world.header.stamp, "camera_init", "base"));

    // path
    static ros_compat::PathMsg path_world;
    ros_compat::PoseStampedMsg pose_path;
    pose_path.header.stamp = odom_world.header.stamp;
    pose_path.header.frame_id = "camera_init";
    pose_path.pose = odom_world.pose.pose;
    path_world.poses.push_back(pose_path);
    path_world.header.stamp = odom_world.header.stamp;
    path_world.header.frame_id = "camera_init";
    pub_path_.publish(path_world);
}

void RosInterface::publishPointcloudWorld(double end_time, const CloudPtr& cloud_world) {
    if (!cloud_world) return;
    ros_compat::PointCloud2Msg pcl_msg;
    pcl::toROSMsg(*cloud_world, pcl_msg);
    pcl_msg.header.stamp = ros_compat::fromSec(end_time);
    pcl_msg.header.frame_id = "camera_init";
    pub_pointcloud_world_.publish(pcl_msg);
}

void RosInterface::run() {
    if (slam_stopped_.load(std::memory_order_acquire)) return;
    if (!this->syncPackage()) return;

    ProcessResult process_result = kilo_->process(measure_);
    LOG_EVERY_N(INFO, 20) << process_result;
    if (!process_result.valid) return;

    double end_time = measure_.lidar_scan_.lidar_end_time_;
    backend_->addFrame(process_result.cloud_body, makeIsometry3d(kilo_->getRotImu(), kilo_->getPosImu()), end_time,
                       process_result.match_types);
    CloudPtr cloud_world = process_result.cloud_world;

    this->publishOdomTFPath(end_time, kilo_->getPosImu(), kilo_->getRotImu());
    this->publishPointcloudWorld(end_time, cloud_world);

    return;
}

}  // namespace legkilo
