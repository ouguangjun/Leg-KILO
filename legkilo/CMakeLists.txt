cmake_minimum_required(VERSION 3.5)
project(legkilo)

set(CMAKE_BUILD_TYPE "Release") 


message(INFO "CMake build type:  ${CMAKE_BUILD_TYPE}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions  -ggdb ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

message(INFO "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(CPU_N)
message(INFO "Processer number:  ${CPU_N}")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)
# find_package(TBB REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    unitree_legged_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
    # ${TBB_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS  roscpp  std_msgs sensor_msgs nav_msgs unitree_legged_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

set(HEADERS
    include/common.hpp
    include/glog_utils.hpp
    include/timer_utils.hpp
    include/yaml_helper.hpp
    include/state_initial.hpp
    include/voxel_map.h
    include/lidar_processing.h
    include/ros_interface.h
    include/options.h
    include/eskf.h
    include/kinematics.h
)

set(SOURCES
    src/lidar_processing.cc
    src/ros_interface.cc
    src/options.cc
    src/eskf.cc
    src/voxel_map.cc
    src/kinematics.cc
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    # ${TBB_LIBRARIES}
    yaml-cpp
    glog
    gflags
)

add_executable(${PROJECT_NAME}_node
    src/leg_kilo_node.cc
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}  
)

