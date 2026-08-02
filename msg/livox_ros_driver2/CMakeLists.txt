cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/LegKiloRosVersion.cmake)
legkilo_resolve_ros_version(LEGKILO_RESOLVED_ROS_VERSION)
message(STATUS "livox_ros_driver2 message package ROS version: ${LEGKILO_RESOLVED_ROS_VERSION}")

set(LIVOX_ROS2_MSG_FILES
  msg/CustomPoint.msg
  msg/CustomMsg.msg
)

if(LEGKILO_RESOLVED_ROS_VERSION STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(
    FILES
    CustomPoint.msg
    CustomMsg.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${LIVOX_ROS2_MSG_FILES}
    DEPENDENCIES std_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
