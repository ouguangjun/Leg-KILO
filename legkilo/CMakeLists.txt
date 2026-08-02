cmake_minimum_required(VERSION 3.16)
project(legkilo)

set(CMAKE_BUILD_TYPE "Release") 

option(LEGKILO_BUILD_TESTS "Build test utilities under legkilo/test" OFF)
option(IRIDESCENCE_VIEWER "Build Iridescence viewer" ON)
option(SMALL_GICP_ONLY_HEADER "Use only header files for small GICP" ON)
option(LEGKILO_MARCH_NATIVE "Use -march=native for host CPU optimization" ON)

message(STATUS "CMake build type: ${CMAKE_BUILD_TYPE}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
if(LEGKILO_MARCH_NATIVE)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/LegKiloRosVersion.cmake)
legkilo_resolve_ros_version(LEGKILO_RESOLVED_ROS_VERSION)
message(STATUS "LegKilo ROS version: ${LEGKILO_RESOLVED_ROS_VERSION}")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Ceres REQUIRED)

# for ubuntu 18.04, please update gcc/g++ to 9, and then unzip oneapi-tbb-2022.2.0-lin in thirdparty folder,
# then check TBB2018_INCLUDE_DIR is correct and set UBUNTU1804_TBB "ON"
option(UBUNTU1804_TBB "use custom tbb package in ubuntu18.04" OFF)
if(UBUNTU1804_TBB)
    set(CUSTOM_TBB_DIR "${PROJECT_SOURCE_DIR}/thirdparty/oneapi-tbb-2022.2.0-lin/oneapi-tbb-2022.2.0")
    set(TBB2022_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB2022_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.8")
    include_directories(${TBB2022_INCLUDE_DIR})
    link_directories(${TBB2022_LIBRARY_DIR})
else()
    find_package(TBB REQUIRED)
    include_directories(${TBB_INCLUDE_DIRS})
    link_directories(${TBB_LIBRARY_DIRS})
endif ()

if(IRIDESCENCE_VIEWER)
  include(cmake/iridescence.cmake)
endif()
include(cmake/small_gicp.cmake)
include(cmake/nanoflann.cmake)
include(cmake/kiss_matcher.cmake)

include_directories(
    src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${Glog_INCLUDE_DIRS}
    ${Ceres_INCLUDE_DIRS}
)

if(LEGKILO_RESOLVED_ROS_VERSION STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
      sensor_msgs
      nav_msgs
      geometry_msgs
      tf
      pcl_ros
      pcl_conversions
      unitree_legged_msgs
      livox_ros_driver
  )

  include_directories(${catkin_INCLUDE_DIRS})

  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs tf pcl_ros pcl_conversions unitree_legged_msgs livox_ros_driver
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS src
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(unitree_legged_msgs REQUIRED)
  find_package(livox_ros_driver2 REQUIRED)
endif()

set(CORE_SOURCES
    # Preprocessing modules
    src/preprocess/lidar_processing.cc
    src/preprocess/kinematics.cc
    
    # frontend
    src/core/slam/frontend/State.cc
    src/core/slam/frontend/KILO.cc
    src/core/slam/frontend/eskf.cc
    src/core/slam/frontend/gaussian_voxel_map.cc

    # backend
    src/core/slam/backend/backend.cc
    src/core/slam/backend/submap.cc
    src/core/slam/backend/factor_graph/factor_graph.cc
    src/core/slam/backend/loop_closure/loop_detector/euclidean_method.cc
    src/core/slam/backend/loop_closure/loop_closure.cc

    #tool
    src/core/slam/tool/slam_result_saver.cc
    src/core/slam/tool/tiled_map.cc
    src/interface/common/options.cc

    # Viewer
    src/viewer/viewer_modal/save_slam_result_modal.cc
    src/viewer/viewer_base.cc
    src/viewer/viewer_slam_interface.cc
)

if(LEGKILO_RESOLVED_ROS_VERSION STREQUAL "1")
  set(ROS_INTERFACE_SOURCES src/interface/ros1/ros_interface.cc)
  set(NODE_SOURCE src/apps/leg_kilo_node.cc)
else()
  set(ROS_INTERFACE_SOURCES src/interface/ros2/ros_interface.cc)
  set(NODE_SOURCE src/apps/leg_kilo_node_ros2.cc)
endif()

add_library(${PROJECT_NAME}.lib ${CORE_SOURCES} ${ROS_INTERFACE_SOURCES})
target_compile_definitions(${PROJECT_NAME}.lib PUBLIC LEGKILO_ROS_VERSION=${LEGKILO_RESOLVED_ROS_VERSION})
target_link_libraries(${PROJECT_NAME}.lib
    PUBLIC
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    Boost::filesystem
    Boost::system
    ceres
    yaml-cpp
    glog
    gflags
    tbb
    PRIVATE
    kiss_matcher
)

if(LEGKILO_RESOLVED_ROS_VERSION STREQUAL "1")
  target_link_libraries(${PROJECT_NAME}.lib PUBLIC ${catkin_LIBRARIES})
else()
  ament_target_dependencies(${PROJECT_NAME}.lib
      PUBLIC
      rclcpp
      std_msgs
      sensor_msgs
      nav_msgs
	      geometry_msgs
	      tf2
	      tf2_ros
	      pcl_conversions
	      unitree_legged_msgs
	      livox_ros_driver2
	  )
endif()

if(IRIDESCENCE_VIEWER)
    target_link_libraries(${PROJECT_NAME}.lib PUBLIC iridescence)
endif()

if(NOT SMALL_GICP_ONLY_HEADER)
  add_dependencies(${PROJECT_NAME}.lib small_gicp)
endif()

add_executable(${PROJECT_NAME}_node ${NODE_SOURCE})
target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
    ${PROJECT_NAME}.lib
)

if(LEGKILO_RESOLVED_ROS_VERSION STREQUAL "2")
  install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY config rviz
    DESTINATION share/${PROJECT_NAME}
  )

  install(DIRECTORY launch_ros2/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  ament_package()
endif()

if(LEGKILO_BUILD_TESTS)
  add_subdirectory(test)
endif()
