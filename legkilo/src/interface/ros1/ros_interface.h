// SPDX-License-Identifier: MIT
// @file ros_interface.h
// @brief ROS1 publishers/subscribers and data flow glue.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_ROS_INTERFACE_H
#define LEG_KILO_ROS_INTERFACE_H

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "interface/common/options.h"
#include "interface/common/ros_compat.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace legkilo {
class Kinematics;
class LidarProcessing;
class KILO;
class Backend;
class ViewerSlamInterface;
}  // namespace legkilo

namespace legkilo {

class RosInterface {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RosInterface() = delete;
    RosInterface(ros::NodeHandle& nh, ViewerSlamInterface* viewer_interface);
    ~RosInterface();

    void init(const std::string& config_file);
    void run();
    void stopSlam();
    void stop();

    bool isSlamStopped() const { return slam_stopped_.load(std::memory_order_acquire); }

   private:
    bool initParamAndReset(const std::string& config_file);
    void subscribeLidar();
    void subscribeKinematicImu();
    void subscribeImu();
    void lidarLoop();
    void imuLoop();
    void kinematicImuLoop();
    void lidarCallBack(const ros_compat::PointCloud2MsgConstPtr& msg);
    void livoxLidarCallBack(const ros_compat::LivoxCustomMsgConstPtr& msg);
    void imuCallBack(const ros_compat::ImuMsgConstPtr& msg);
    void kinematicImuCallBack(const ros_compat::HighStateMsgConstPtr& msg);
    bool syncPackage();
    void publishOdomTFPath(double end_time, const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot);
    void publishPointcloudWorld(double end_time, const CloudPtr& cloud_world);

    ros::NodeHandle& nh_;

    // subscriber
    ros::Subscriber sub_lidar_raw_;
    ros::Subscriber sub_imu_raw_;
    ros::Subscriber sub_kinematic_raw_;

    // publisher
    ros::Publisher pub_pointcloud_body_;
    ros::Publisher pub_pointcloud_world_;
    ros::Publisher pub_path_;
    ros::Publisher pub_odom_world_;
    ros::Publisher pub_joint_state_;

    // sub thread
    std::unique_ptr<std::thread> lidar_thread_;
    std::unique_ptr<std::thread> imu_thread_;
    std::unique_ptr<std::thread> kinematic_thread_;

    // module
    std::unique_ptr<LidarProcessing> lidar_processing_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<KILO> kilo_;
    std::unique_ptr<Backend> backend_;
    ViewerSlamInterface* viewer_interface_ = nullptr;

    // measure
    std::deque<common::LidarScan> lidar_cache_;
    std::deque<ros_compat::ImuMsgPtr> imu_cache_;
    std::deque<common::KinImuMeas> kin_imu_cache_;
    common::MeasGroup measure_;

    // sync package
    std::mutex mutex_;
    double last_timestamp_imu_;
    double last_timestamp_kin_imu_;
    double lidar_end_time_;

    // initialization
    bool init_flag_ = true;

    std::atomic_bool slam_stopped_{false};

    // vis
    bool pub_joint_tf_enable_ = true;
};

}  // namespace legkilo
#endif  // LEG_KILO_ROS_INTERFACE_H
