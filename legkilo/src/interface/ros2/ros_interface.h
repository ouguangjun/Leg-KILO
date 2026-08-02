// SPDX-License-Identifier: MIT
// @file ros_interface.h
// @brief ROS2 publishers/subscribers and data flow glue.
#ifndef LEG_KILO_ROS2_INTERFACE_H
#define LEG_KILO_ROS2_INTERFACE_H

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "interface/common/options.h"
#include "interface/common/ros_compat.h"

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
    RosInterface(const std::shared_ptr<rclcpp::Node>& node, ViewerSlamInterface* viewer_interface);
    ~RosInterface();

    void init(const std::string& config_file);
    void run();
    void stopSlam();
    void stop();

    bool isSlamStopped() const { return slam_stopped_.load(std::memory_order_acquire); }

   private:
    bool initParamAndReset(const std::string& config_file);
    void createPublishers();
    void subscribeLidar();
    void subscribeKinematicImu();
    void subscribeImu();
    void lidarLoop();
    void imuLoop();
    void kinematicImuLoop();
    void lidarCallBack(ros_compat::PointCloud2MsgConstPtr msg);
    void livoxLidarCallBack(ros_compat::LivoxCustomMsgConstPtr msg);
    void imuCallBack(ros_compat::ImuMsgConstPtr msg);
    void kinematicImuCallBack(ros_compat::HighStateMsgConstPtr msg);
    bool syncPackage();
    void publishOdomTFPath(double end_time, const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot);
    void publishPointcloudWorld(double end_time, const CloudPtr& cloud_world);

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Node> lidar_node_;
    std::shared_ptr<rclcpp::Node> imu_node_;
    std::shared_ptr<rclcpp::Node> kinematic_node_;

    rclcpp::Subscription<ros_compat::PointCloud2Msg>::SharedPtr sub_lidar_raw_;
    rclcpp::Subscription<ros_compat::LivoxCustomMsg>::SharedPtr sub_livox_lidar_raw_;
    rclcpp::Subscription<ros_compat::ImuMsg>::SharedPtr sub_imu_raw_;
    rclcpp::Subscription<ros_compat::HighStateMsg>::SharedPtr sub_kinematic_raw_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> lidar_executor_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> imu_executor_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> kinematic_executor_;

    rclcpp::Publisher<ros_compat::PointCloud2Msg>::SharedPtr pub_pointcloud_body_;
    rclcpp::Publisher<ros_compat::PointCloud2Msg>::SharedPtr pub_pointcloud_world_;
    rclcpp::Publisher<ros_compat::PathMsg>::SharedPtr pub_path_;
    rclcpp::Publisher<ros_compat::OdometryMsg>::SharedPtr pub_odom_world_;
    rclcpp::Publisher<ros_compat::JointStateMsg>::SharedPtr pub_joint_state_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<LidarProcessing> lidar_processing_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<KILO> kilo_;
    std::unique_ptr<Backend> backend_;
    std::unique_ptr<std::thread> lidar_thread_;
    std::unique_ptr<std::thread> imu_thread_;
    std::unique_ptr<std::thread> kinematic_thread_;
    ViewerSlamInterface* viewer_interface_ = nullptr;

    std::deque<common::LidarScan> lidar_cache_;
    std::deque<ros_compat::ImuMsgPtr> imu_cache_;
    std::deque<common::KinImuMeas> kin_imu_cache_;
    common::MeasGroup measure_;

    std::mutex mutex_;
    double last_timestamp_imu_ = 0.0;
    double last_timestamp_kin_imu_ = 0.0;
    double lidar_end_time_ = 0.0;

    ros_compat::PathMsg path_world_;

    bool init_flag_ = true;
    std::atomic_bool slam_stopped_{false};
    bool pub_joint_tf_enable_ = true;
};

}  // namespace legkilo
#endif  // LEG_KILO_ROS2_INTERFACE_H
