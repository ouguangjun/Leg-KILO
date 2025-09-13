#ifndef LEG_KILO_TRAJECTORY_SAVER_HPP
#define LEG_KILO_TRAJECTORY_SAVER_HPP

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "common/eigen_types.hpp"

namespace legkilo {

namespace fs = boost::filesystem;

// Save trajectory in TUM format:
// timestamp tx ty tz qx qy qz qw
class TrajectorySaver {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectorySaver() {
        const std::string dir = std::string(ROOT_DIR) + "result/traj/";
        if (!ensureDir(dir)) { throw std::runtime_error("Create Trajectory Directory Failed"); }
        filepath_ = makeFilepathWithNow(dir);
        ofs_.open(filepath_, std::ios::out | std::ios::trunc);
        if (!ofs_.is_open()) { throw std::runtime_error("Open Trajectory File Failed: " + filepath_); }
        LOG(INFO) << "Trajectory file: " << filepath_;
    }

    ~TrajectorySaver() {
        flush();
        if (ofs_.is_open()) { ofs_.close(); }
    }

    // Write one TUM line. Rotation is body->world.
    void write(double timestamp, const Mat3D& rot, const Vec3D& pos) {
        if (!ofs_.is_open()) return;
        std::lock_guard<std::mutex> lk(mutex_);
        Eigen::Quaterniond q(rot);
        // TUM: timestamp tx ty tz qx qy qz qw
        ofs_ << std::fixed << std::setprecision(9) << timestamp << " " << pos.x() << " " << pos.y() << " " << pos.z()
             << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }

    void flush() {
        std::lock_guard<std::mutex> lk(mutex_);
        ofs_.flush();
    }

   private:
    bool ensureDir(const std::string& dir) {
        try {
            if (!fs::exists(dir)) { return fs::create_directories(dir); }
            return true;
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }

    std::string makeFilepathWithNow(const std::string& dir) const {
        const auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        const std::tm* tm_ptr = std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(tm_ptr, "%Y%m%d_%H%M%S");
        return (fs::path(dir) / (std::string("traj_") + oss.str() + ".tum")).string();
    }

   private:
    std::mutex mutex_;
    std::ofstream ofs_;
    std::string filepath_;
};

}  // namespace legkilo

#endif  // LEG_KILO_TRAJECTORY_SAVER_HPP
