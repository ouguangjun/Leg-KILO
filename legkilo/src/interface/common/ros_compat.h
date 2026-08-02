// SPDX-License-Identifier: MIT
// @file ros_compat.h
// @brief Thin ROS1/ROS2 compatibility aliases shared by the codebase.
#ifndef LEG_KILO_INTERFACE_COMMON_ROS_COMPAT_H_
#define LEG_KILO_INTERFACE_COMMON_ROS_COMPAT_H_

#include <cmath>
#include <cstdint>

#include "interface/common/ros_build_config.h"

#if LEGKILO_USE_ROS1

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <unitree_legged_msgs/HighState.h>

namespace legkilo {
namespace ros_compat {

using Time = ros::Time;
using ImuMsg = sensor_msgs::Imu;
using ImuMsgPtr = sensor_msgs::ImuPtr;
using ImuMsgConstPtr = sensor_msgs::Imu::ConstPtr;
using JointStateMsg = sensor_msgs::JointState;
using PointCloud2Msg = sensor_msgs::PointCloud2;
using PointCloud2MsgConstPtr = sensor_msgs::PointCloud2::ConstPtr;
using OdometryMsg = nav_msgs::Odometry;
using PathMsg = nav_msgs::Path;
using PoseStampedMsg = geometry_msgs::PoseStamped;
using TransformStampedMsg = geometry_msgs::TransformStamped;
using HighStateMsg = unitree_legged_msgs::HighState;
using HighStateMsgPtr = unitree_legged_msgs::HighStatePtr;
using HighStateMsgConstPtr = unitree_legged_msgs::HighState::ConstPtr;
using LivoxCustomMsg = livox_ros_driver::CustomMsg;
using LivoxCustomMsgPtr = livox_ros_driver::CustomMsgPtr;
using LivoxCustomMsgConstPtr = livox_ros_driver::CustomMsg::ConstPtr;
using LivoxCustomPoint = livox_ros_driver::CustomPoint;

inline double toSec(const ros::Time& stamp) { return stamp.toSec(); }

inline ros::Time fromSec(const double time_sec) { return ros::Time().fromSec(time_sec); }

inline const auto& footForces(const HighStateMsg& msg) { return msg.footForce; }

inline const auto& motorStates(const HighStateMsg& msg) { return msg.motorState; }

}  // namespace ros_compat
}  // namespace legkilo

#elif LEGKILO_USE_ROS2

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree_legged_msgs/msg/high_state.hpp>

namespace legkilo {
namespace ros_compat {

using Time = builtin_interfaces::msg::Time;
using ImuMsg = sensor_msgs::msg::Imu;
using ImuMsgPtr = sensor_msgs::msg::Imu::SharedPtr;
using ImuMsgConstPtr = sensor_msgs::msg::Imu::ConstSharedPtr;
using JointStateMsg = sensor_msgs::msg::JointState;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PointCloud2MsgConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using OdometryMsg = nav_msgs::msg::Odometry;
using PathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
using HighStateMsg = unitree_legged_msgs::msg::HighState;
using HighStateMsgPtr = unitree_legged_msgs::msg::HighState::SharedPtr;
using HighStateMsgConstPtr = unitree_legged_msgs::msg::HighState::ConstSharedPtr;
using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg;
using LivoxCustomMsgPtr = livox_ros_driver2::msg::CustomMsg::SharedPtr;
using LivoxCustomMsgConstPtr = livox_ros_driver2::msg::CustomMsg::ConstSharedPtr;
using LivoxCustomPoint = livox_ros_driver2::msg::CustomPoint;

inline double toSec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

inline builtin_interfaces::msg::Time fromSec(const double time_sec) {
    builtin_interfaces::msg::Time stamp;
    double integral = 0.0;
    const double fractional = std::modf(time_sec, &integral);
    stamp.sec = static_cast<int32_t>(integral);
    stamp.nanosec = static_cast<uint32_t>(std::llround(fractional * 1e9));
    if (stamp.nanosec >= 1000000000u) {
        ++stamp.sec;
        stamp.nanosec -= 1000000000u;
    }
    return stamp;
}

inline const auto& footForces(const HighStateMsg& msg) { return msg.foot_force; }

inline const auto& motorStates(const HighStateMsg& msg) { return msg.motor_state; }

}  // namespace ros_compat
}  // namespace legkilo

#endif

#endif  // LEG_KILO_INTERFACE_COMMON_ROS_COMPAT_H_
