// SPDX-License-Identifier: MIT
// @file backend.h
// @brief Backend processing
// @author Ou Guangjun
// @created 2026-01-28
// @maintainer ouguangjun98@gmail.com

#ifndef LEGKILO_BACKEND_H
#define LEGKILO_BACKEND_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "common/pcl_types.h"
#include "core/slam/backend/loop_closure/loop_closure.h"
#include "core/slam/backend/submap.h"

namespace legkilo {
class ViewerSlamInterface;
class FactorGraph;
}  // namespace legkilo

namespace legkilo {
using NodeType = int64_t;
using IDPoses = std::vector<std::pair<NodeType, Eigen::Isometry3d>>;
using IDPosesPtr = std::shared_ptr<IDPoses>;

class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct FramePacket {
        CloudPtr cloud;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        double timestamp = 0.0;
        LidarMatchTypesPtr match_types = nullptr;
    };

    struct Config {
        double kf_trans_threshold = 0.3;          // Translation threshold for keyframe selection [m]
        double kf_degree_threshold = 5.0;         // Rotation threshold for keyframe selection [deg]
        size_t kf_max_num_submap = 50;            // Maximum number of keyframes in the submap
        double kf_max_dist_submap = 5.0;          // Maximum distance for begin/end keyframe in the submap [m]
        double submap_idle_finish_timeout = 1.0;  // finish current submap when idle for [sec]
    };

    explicit Backend(const std::string& yaml_file);

    ~Backend();

    void addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose, double timestamp,
                  LidarMatchTypesPtr match_types = nullptr);

    void start();

    void stop();

    void setViewerInterface(ViewerSlamInterface* viewer_interface);

   private:
    void workerLoop();

    bool isKeyFrame(const Eigen::Isometry3d& current_pose);

    bool isSubmapFinished(const SubmapPtr& submap) const;

    void finalizeCurrentSubmap();

    void fetchAndApplyLoopClosures();

    void updateRecordedSubmaps();

   private:
    Config config_;

    // submaps
    std::mutex mutex_finished_submaps_;
    std::map<NodeType, SubmapPtr> finished_submaps_;
    SubmapPtr current_submap_;

    // threading
    std::mutex mutex_queue_;  // protects access to frame_queue_
    std::condition_variable cv_;
    std::deque<FramePacket> frame_queue_;
    std::thread worker_thread_;
    std::atomic<bool> stopping_{false};

    // viewer
    ViewerSlamInterface* viewer_interface_ = nullptr;

    // factor graph
    std::unique_ptr<FactorGraph> factor_graph_;

    // loop closure
    std::unique_ptr<LoopClosure> loop_closure_;

    // other running variables
    bool is_first_frame_ = true;
    Eigen::Isometry3d last_kf_pose_;
};

}  // namespace legkilo

#endif  // LEGKILO_BACKEND_H
