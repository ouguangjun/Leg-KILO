// SPDX-License-Identifier: MIT
// @file glog_utils.hpp
// @brief GFlags/GLog initialization and log file helpers.
// @author Ou Guangjun
// @created 2024-12-17
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_GLOG_UTILS_H
#define LEG_KILO_GLOG_UTILS_H

#include <algorithm>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace legkilo {

class Logging {
   public:
    Logging(int argc, char** argv, std::string log_dir, int keep_per_level = 20);
    ~Logging();
    bool createLogFile(std::string dir);
    void flushLogFiles();

   private:
    void pruneOldLogs(const std::string& dir);
    int keep_per_level_ = 20;
};

inline Logging::Logging(int argc, char** argv, std::string log_dir, int keep_per_level) {
    keep_per_level_ = keep_per_level;
    log_dir = std::string(ROOT_DIR) + log_dir;
    if (!createLogFile(log_dir)) { throw std::runtime_error("Create Log File Failed"); }

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_log_dir = log_dir;

    try {
        pruneOldLogs(log_dir);
    } catch (const std::exception& e) { LOG(WARNING) << "prune logs failed: " << e.what(); }

    std::cout << "\033[33m"
              << "GLOG ON"
              << "\033[0m" << std::endl;
}

inline Logging::~Logging() {
    google::ShutdownGoogleLogging();
    std::cout << "\033[33m"
              << "GLOG OFF"
              << "\033[0m" << std::endl;
}

inline bool Logging::createLogFile(std::string dir) {
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

inline void Logging::flushLogFiles() {
    google::FlushLogFiles(google::INFO);
    google::FlushLogFiles(google::WARNING);
    google::FlushLogFiles(google::ERROR);
}

inline void Logging::pruneOldLogs(const std::string& dir) {
    if (keep_per_level_ <= 0) return;
    if (!fs::exists(dir) || !fs::is_directory(dir)) return;

    const char* levels[4] = {".log.INFO.", ".log.WARNING.", ".log.ERROR.", ".log.FATAL."};
    int total_deleted = 0;
    std::time_t oldest_deleted = std::time(nullptr);
    bool has_deleted = false;
    for (const char* lv : levels) {
        std::vector<fs::path> files;
        for (fs::directory_iterator it(dir); it != fs::directory_iterator(); ++it) {
            if (!fs::is_regular_file(it->path())) continue;
            const std::string name = it->path().filename().string();
            if (name.find(lv) != std::string::npos) { files.emplace_back(it->path()); }
        }
        if (static_cast<int>(files.size()) <= keep_per_level_) continue;
        std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) {
            std::time_t ta = 0, tb = 0;
            try {
                ta = fs::last_write_time(a);
                tb = fs::last_write_time(b);
            } catch (...) { return a.string() > b.string(); }
            return ta > tb;  // newest first
        });
        for (size_t i = static_cast<size_t>(keep_per_level_); i < files.size(); ++i) {
            try {
                // Track oldest timestamp among deleted
                try {
                    std::time_t tdel = fs::last_write_time(files[i]);
                    if (!has_deleted || tdel < oldest_deleted) {
                        oldest_deleted = tdel;
                        has_deleted = true;
                    }
                } catch (...) {}
                fs::remove(files[i]);
                total_deleted++;
            } catch (...) {
                // ignore deletion failures
            }
        }
    }
    if (has_deleted) {
        std::time_t now = std::time(nullptr);
        long days = static_cast<long>((now - oldest_deleted) / (60 * 60 * 24));
        LOG(INFO) << "Pruned " << total_deleted << " old log files (keep per level = " << keep_per_level_
                  << ") — oldest deleted about " << days << " days old.";
    } else {
        LOG(INFO) << "No old log files to prune (keep per level = " << keep_per_level_ << ").";
    }
}

}  // namespace legkilo
#endif  // LEG_KILO_GLOG_UTILS_H
