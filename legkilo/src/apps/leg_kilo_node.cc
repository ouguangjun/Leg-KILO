// SPDX-License-Identifier: MIT
// @file leg_kilo_node.cc
// @brief Main node for Leg KILO SLAM system(ros1).
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#include <csignal>
#include <memory>

#include <unistd.h>

#include <pcl/console/print.h>

#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "interface/ros1/ros_interface.h"
#include "viewer/viewer_slam_interface.h"

DEFINE_string(config_file, "config/leg_fusion.yaml", "Path to the YAML file");
namespace {
volatile std::sig_atomic_t shutdown_requested = 0;
}

void sigHandle(int sig) { shutdown_requested = 1; }

int main(int argc, char** argv) {
    ros::init(argc, argv, "legkilo");
    ros::NodeHandle nh("legkilo");

    signal(SIGINT, sigHandle);

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::unique_ptr<legkilo::Logging> logging = std::make_unique<legkilo::Logging>(argc, argv, "logs", 20);

    if (FLAGS_config_file.empty()) {
        LOG(ERROR) << "YAML configuration file path not provided. Use --config_path=<path>.";
        return -1;
    }

    auto viewer_interface = std::make_unique<legkilo::ViewerSlamInterface>(FLAGS_config_file);
    viewer_interface->start();

    std::unique_ptr<legkilo::RosInterface> ros_interface_node =
        std::make_unique<legkilo::RosInterface>(nh, viewer_interface.get());

    ros_interface_node->init(FLAGS_config_file);

    LOG(INFO) << "Leg KILO Node Starts";

    ros::Rate running_rate(5000);
    ros::Rate idle_rate(10);
    while (ros::ok() && !shutdown_requested && !viewer_interface->shouldTerminate()) {
        if (!ros_interface_node->isSlamStopped()) { ros_interface_node->run(); }

        if (!ros_interface_node->isSlamStopped() && viewer_interface->consumeSLAMTerminateRequest()) {
            ros_interface_node->stopSlam();
            LOG(INFO) << "SLAM stopped by viewer request";
        }

        ros_interface_node->isSlamStopped() ? idle_rate.sleep() : running_rate.sleep();
    }

    ros_interface_node->stop();
    viewer_interface->stop();

    ros_interface_node.reset();
    viewer_interface.reset();
    LOG(INFO) << "RosInterface destroyed";
    LOG(INFO) << "Leg KILO Node Ends";
    legkilo::Timer::logAllAverTime();
    logging->flushLogFiles();
    return 0;
}
