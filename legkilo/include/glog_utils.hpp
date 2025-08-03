#ifndef LEG_KILO_GLOG_UTILS_H
#define LEG_KILO_GLOG_UTILS_H

#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace legkilo {

class Logging {
   public:
    Logging(int argc, char** argv, std::string log_dir);
    ~Logging();
    bool createLogFile(std::string dir);
    void flushLogFiles();
};

Logging::Logging(int argc, char** argv, std::string log_dir) {
    log_dir = std::string(ROOT_DIR) + log_dir;
    if (!createLogFile(log_dir)) { throw std::runtime_error("Create Log File Failed"); }

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_log_dir = log_dir;

    std::cout << "\033[33m"
              << "GLOG ON"
              << "\033[0m" << std::endl;
}

Logging::~Logging() {
    google::ShutdownGoogleLogging();
    std::cout << "\033[33m"
              << "GLOG OFF"
              << "\033[0m" << std::endl;
}

bool Logging::createLogFile(std::string dir) {
    if (!fs::exists(dir)) {
        std::cout << "Creating Log File" << std::endl;
        try {
            if (fs::create_directory(dir)) {
                std::cout << "Folder created successfully" << std::endl;
                return true;
            } else {
                std::cerr << "Failed to create folder" << std::endl;
                return false;
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }

    return true;
}

void Logging::flushLogFiles() {
    google::FlushLogFiles(google::INFO);
    google::FlushLogFiles(google::WARNING);
    google::FlushLogFiles(google::ERROR);
}

}  // namespace legkilo
#endif  // LEG_KILO_GLOG_UTILS_H
