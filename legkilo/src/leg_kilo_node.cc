#include <csignal>
#include <memory>

#include <unistd.h>

#include "glog_utils.hpp"
#include "ros_interface.h"

DEFINE_string(config_file, "config/leg_fusion.yaml", "Path to the YAML file");
void sigHandle(int sig) {
    legkilo::options::FLAG_EXIT = true;
    LOG(INFO) << "catch sig " << sig << "  FLAG_EXIT = True";
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "legkilo");
    ros::NodeHandle nh("legkilo");

    signal(SIGINT, sigHandle);

    std::unique_ptr<legkilo::Logging> logging(new legkilo::Logging(argc, argv, "logs"));
    std::unique_ptr<legkilo::RosInterface> ros_interface_node = std::make_unique<legkilo::RosInterface>(nh);

    if (FLAGS_config_file.empty()) {
        LOG(ERROR) << "YAML configuration file path not provided. Use --config_path=<path>.";
        return -1;
    }

    ros_interface_node->rosInit(FLAGS_config_file);

    LOG(INFO) << "Leg KILO Node Starts";

    ros::Rate rate(5000);
    while (ros::ok() && !legkilo::options::FLAG_EXIT) {
        ros_interface_node->run();
        rate.sleep();
    }

    LOG(INFO) << "Leg KILO Node Ends";
    legkilo::Timer::logAllAverTime();
    logging->flushLogFiles();
    return 0;
}