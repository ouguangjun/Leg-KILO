#ifndef LEG_KILO_PCD_SAVER_HPP
#define LEG_KILO_PCD_SAVER_HPP

#include <chrono>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include "common/pcl_types.h"

namespace legkilo {

namespace fs = boost::filesystem;

class PcdSaver {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit PcdSaver(int frames_per_file = 100, double voxel_leaf_size = 0.1)
        : frames_per_file_(frames_per_file), voxel_leaf_size_(voxel_leaf_size) {
        if (frames_per_file_ <= 0) frames_per_file_ = 100;
        if (voxel_leaf_size_ <= 0.0) voxel_leaf_size_ = 0.1;
        initSessionDir();
        buffer_.reset(new PointCloudType);
        worker_ = std::thread(&PcdSaver::workerLoop, this);
    }

    ~PcdSaver() {
        {
            std::lock_guard<std::mutex> lk(mutex_);
            stopping_ = true;
        }
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

    // Save cloud as binary-compressed PCD with auto-increment id.
    void save(const CloudConstPtr& cloud) {
        if (!cloud || cloud->empty()) return;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            queue_.push_back(cloud);
        }
        cv_.notify_one();
    }

   private:
    void initSessionDir() {
        const std::string base = std::string(ROOT_DIR) + "result/PCD/";
        ensureDir(base);
        const std::string stamp = nowString();
        session_dir_ = (fs::path(base) / stamp).string();
        ensureDir(session_dir_);
        LOG(INFO) << "PCD session dir: " << session_dir_;
    }

    void ensureDir(const std::string& dir) {
        try {
            if (!fs::exists(dir)) { fs::create_directories(dir); }
        } catch (const fs::filesystem_error& e) {
            throw std::runtime_error(std::string("Create Directory Failed: ") + e.what());
        }
    }

    std::string nowString() const {
        const auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        const std::tm* tm_ptr = std::localtime(&t);
        std::ostringstream oss;
        oss << std::put_time(tm_ptr, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    std::string makeFilename(size_t id) const {
        std::ostringstream oss;
        oss << "cloud_" << std::setfill('0') << std::setw(6) << id << ".pcd";
        return oss.str();
    }

    void workerLoop() {
        while (true) {
            std::deque<CloudConstPtr> local_queue;
            {
                std::unique_lock<std::mutex> lk(mutex_);
                cv_.wait(lk, [&] { return stopping_ || !queue_.empty(); });
                local_queue.swap(queue_);
            }

            for (const auto& cloud : local_queue) {
                if (!cloud || cloud->empty()) continue;
                *buffer_ += *cloud;
                ++frames_in_buffer_;
                if (frames_in_buffer_ >= frames_per_file_) { downsampleAndSave(); }
            }

            if (stopping_) {
                if (buffer_ && !buffer_->empty()) { downsampleAndSave(); }
                break;
            }
        }
    }

    void downsampleAndSave() {
        if (!buffer_ || buffer_->empty()) return;
        PointCloudType cloud_ds;
        pcl::VoxelGrid<PointType> vg;
        vg.setLeafSize(static_cast<float>(voxel_leaf_size_), static_cast<float>(voxel_leaf_size_),
                       static_cast<float>(voxel_leaf_size_));
        vg.setInputCloud(buffer_);
        vg.filter(cloud_ds);

        const std::string filename = makeFilename(file_id_);
        const std::string filepath = (fs::path(session_dir_) / filename).string();
        const int ret = pcl::io::savePCDFileBinaryCompressed(filepath, cloud_ds);
        if (ret != 0) {
            LOG(WARNING) << "Failed to save PCD: " << filepath;
        } else {
            LOG(INFO) << "Saved PCD: " << filepath << ", points: " << cloud_ds.size();
        }
        // Reset buffer
        buffer_->clear();
        frames_in_buffer_ = 0;
        ++file_id_;
    }

   private:
    std::string session_dir_;
    size_t file_id_ = 0;
    int frames_per_file_ = 100;
    double voxel_leaf_size_ = 0.1;
    int frames_in_buffer_ = 0;
    CloudPtr buffer_;

    // threading
    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<CloudConstPtr> queue_;
    bool stopping_ = false;
};

}  // namespace legkilo

#endif  // LEG_KILO_PCD_SAVER_HPP
