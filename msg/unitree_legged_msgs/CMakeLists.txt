cmake_minimum_required(VERSION 3.16)
project(unitree_legged_msgs)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/LegKiloRosVersion.cmake)
legkilo_resolve_ros_version(LEGKILO_RESOLVED_ROS_VERSION)
message(STATUS "unitree_legged_msgs ROS version: ${LEGKILO_RESOLVED_ROS_VERSION}")

set(UNITREE_ROS2_MSG_FILES
  msg_ros2/MotorCmd.msg
  msg_ros2/MotorState.msg
  msg_ros2/BmsCmd.msg
  msg_ros2/BmsState.msg
  msg_ros2/Cartesian.msg
  msg_ros2/IMU.msg
  msg_ros2/LED.msg
  msg_ros2/LowCmd.msg
  msg_ros2/LowState.msg
  msg_ros2/HighCmd.msg
  msg_ros2/HighState.msg
)

if(LEGKILO_RESOLVED_ROS_VERSION STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
  )

  add_message_files(
    FILES
    MotorCmd.msg
    MotorState.msg
    BmsCmd.msg
    BmsState.msg
    Cartesian.msg
    IMU.msg
    LED.msg
    LowCmd.msg
    LowState.msg
    HighCmd.msg
    HighState.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
  )

  install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${UNITREE_ROS2_MSG_FILES}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
