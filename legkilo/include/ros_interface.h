#ifndef LEG_KILO_ROS_INTERFACE_H
#define LEG_KILO_ROS_INTERFACE_H

#include <iostream>
#include <string>
#include <thread>
#include <deque>
#include <mutex>
#include <utility>
#include <iomanip>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <unitree_legged_msgs/HighState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/filters/voxel_grid.h>

#include "yaml_helper.hpp"
#include "common.hpp"
#include "options.h"
#include "lidar_processing.h"
#include "timer_utils.hpp"
// #include "voxel_grid.hpp"
#include "eskf.h"
#include "state_initial.hpp"
#include "voxel_map.h"
#include "kinematics.h"


namespace legkilo{

class RosInterface{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RosInterface() = delete;
    RosInterface(ros::NodeHandle& nh);
    ~RosInterface();
    
    void rosInit(const std::string& config_file);
    void run();

private:
    bool initParamAndReset(const std::string& config_file);
    void subscribeLidar();
    void subscribeKinematicImu();
    void subscribeImu();
    void lidarLoop();
    void imuLoop();
    void kinematicImuLoop();
    void lidarCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg);
    void kinematicImuCallBack(const unitree_legged_msgs::HighState::ConstPtr& msg);
    bool syncPackage();
    void initStateAndMap();
    bool predictUpdateImu(const sensor_msgs::ImuPtr& imu);
    bool predictUpdatePoint(double current_time, size_t idx_i, size_t idx_j);
    bool predictUpdateKinImu(const common::KinImuMeas& kin_imu);
    void runReset();
    void cloudLidarToWorld(const CloudPtr& cloud_lidar, CloudPtr& cloud_imu);
    void pointLidarToImu(const PointType& point_lidar, PointType& point_imu);
    void pointLidarToWorld(const PointType& point_lidar, PointType& point_world);
    void publishOdomTFPath(double end_time);
    void publishPointcloudWorld(double end_time);
    void publishPointcloudBody(double end_time);  // without undistort
    
    ros::NodeHandle& nh_;

    // subscriber
    ros::Subscriber sub_lidar_raw_;
    ros::Subscriber sub_imu_raw_;
    ros::Subscriber sub_kinematic_raw_;

    //publisher
    ros::Publisher pub_pointcloud_body_;
    ros::Publisher pub_pointcloud_world_;
    ros::Publisher pub_path_;
    ros::Publisher pub_odom_world_;

    nav_msgs::Odometry odom_world_;
    nav_msgs::Path path_world_;
    tf::TransformBroadcaster br_;
    tf::Transform transform_;
    tf::Quaternion q_tf_;
    Eigen::Quaterniond q_eigen_;
    geometry_msgs::PoseStamped pose_path_;

    // sub thread
    std::unique_ptr<std::thread> lidar_thread_;
    std::unique_ptr<std::thread> imu_thread_;
    std::unique_ptr<std::thread> kinematic_thread_;

    // module
    std::unique_ptr<ESKF> eskf_;
    std::unique_ptr<LidarProcessing> lidar_processing_;
    // std::unique_ptr<VoxelGrid> voxel_grid_;
    std::unique_ptr<StateInitial> state_initial_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<VoxelMapManager> map_manager_;
    pcl::VoxelGrid<PointType> voxel_grid_;

    // meaure
    std::deque<common::LidarScan> lidar_cache_;
    std::deque<sensor_msgs::Imu::Ptr> imu_cache_;
    std::deque<common::KinImuMeas> kin_imu_cache_;
    common::MeasGroup measure_;

    //sync package
    std::mutex mutex_;
    double last_timestamp_imu_;
    double last_timestamp_kin_imu_;
    double lidar_end_time_;

    // initialization
    double init_time_ = 0.1;
    bool init_flag_ = true;

    // pcl
    CloudPtr cloud_raw_;
    CloudPtr cloud_down_body_;
    CloudPtr cloud_down_world_;

    // eskf
    double gravity_;
    double acc_norm_;
    double last_state_predict_time_;
    double last_state_update_time_;

    //sensor param
    Mat3D ext_rot_;
    Vec3D ext_t_;
    double satu_acc_;
    double satu_gyr_;

    // LOG
    size_t success_pts_size;
};

} // namespace legkilo
#endif // LEG_KILO_ROS_INTERFACE_H