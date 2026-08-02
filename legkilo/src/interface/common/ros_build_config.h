// SPDX-License-Identifier: MIT
// @file ros_build_config.h
// @brief Compile-time ROS version selection for LegKilo.
#ifndef LEG_KILO_INTERFACE_COMMON_ROS_BUILD_CONFIG_H_
#define LEG_KILO_INTERFACE_COMMON_ROS_BUILD_CONFIG_H_

#define LEGKILO_ROS1 1
#define LEGKILO_ROS2 2

#ifndef LEGKILO_ROS_VERSION
#define LEGKILO_ROS_VERSION LEGKILO_ROS1
#endif

#if LEGKILO_ROS_VERSION == LEGKILO_ROS1
#define LEGKILO_USE_ROS1 1
#define LEGKILO_USE_ROS2 0
#elif LEGKILO_ROS_VERSION == LEGKILO_ROS2
#define LEGKILO_USE_ROS1 0
#define LEGKILO_USE_ROS2 1
#else
#error "Unsupported LEGKILO_ROS_VERSION value"
#endif

#endif  // LEG_KILO_INTERFACE_COMMON_ROS_BUILD_CONFIG_H_
