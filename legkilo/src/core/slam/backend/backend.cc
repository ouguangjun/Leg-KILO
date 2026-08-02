#include "core/slam/backend/backend.h"

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "common/file_io.hpp"
#include "common/timer_utils.hpp"
#include "common/voxel_grid.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/backend/factor_graph/factor_graph.h"
#include "core/slam/tool/slam_result_recorder.h"
#include "viewer/viewer_slam_interface.h"

namespace legkilo {

Backend::Backend(const std::string& yaml_file) {
    stopping_.store(false, std::memory_order_release);
    is_first_frame_ = true;

    YamlHelper yaml_helper(yaml_file);
    config_.kf_trans_threshold = yaml_helper.get<double>("kf_trans_threshold", 0.3);
    config_.kf_degree_threshold = yaml_helper.get<double>("kf_degree_threshold", 5.0);
    config_.kf_max_num_submap = yaml_helper.get<size_t>("kf_max_num_submap", 50);
    config_.kf_max_dist_submap = yaml_helper.get<double>("kf_max_dist_submap", 5.0);
    config_.submap_idle_finish_timeout = std::max(yaml_helper.get<double>("submap_idle_finish_timeout", 1.0), 0.1);

    std::string temp_result_save_folder =
        std::string(ROOT_DIR) + "result/" + yaml_helper.get<std::string>("temp_result_save_folder", "temp") + "/";
    file_io::ensureDirectory(temp_result_save_folder);
    file_io::clearDirectoryContents(temp_result_save_folder);
    Submap::setSavePathFolder(temp_result_save_folder);
    SLAMResultRecorder::initialize(temp_result_save_folder, yaml_file);

    factor_graph_ = std::make_unique<FactorGraph>(yaml_file);

    bool loop_closure_enabled = yaml_helper.get<bool>("loop_closure_enable", true);
    if (loop_closure_enabled) loop_closure_ = std::make_unique<LoopClosure>(yaml_file);
}

Backend::~Backend() {
    this->stop();
    LOG(INFO) << "Backend is destroyed";
}

void Backend::addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose, double timestamp,
                       LidarMatchTypesPtr match_types) {
    if (!frame || frame->empty()) return;

    std::lock_guard<std::mutex> lock(mutex_queue_);
    frame_queue_.push_back({frame, pose, timestamp, match_types});
    cv_.notify_one();
}

void Backend::start() {
    if (loop_closure_) loop_closure_->start();
    worker_thread_ = std::thread(&Backend::workerLoop, this);
}

void Backend::stop() {
    stopping_.store(true, std::memory_order_release);
    cv_.notify_one();
    if (worker_thread_.joinable()) worker_thread_.join();
    if (loop_closure_) loop_closure_->stop();
}

void Backend::setViewerInterface(ViewerSlamInterface* viewer_interface) { viewer_interface_ = viewer_interface; }

void Backend::workerLoop() {
    const auto idle_timeout = std::chrono::milliseconds(static_cast<int>(config_.submap_idle_finish_timeout * 1000));
    while (true) {
        std::deque<FramePacket> local_queue;
        bool timed_out = false;
        {
            std::unique_lock<std::mutex> lock(mutex_queue_);
            timed_out = !cv_.wait_for(
                lock, idle_timeout, [&] { return stopping_.load(std::memory_order_acquire) || !frame_queue_.empty(); });
            local_queue.swap(frame_queue_);
        }

        for (const auto& frame : local_queue) {
            const auto& current_cloud = frame.cloud;
            const auto& current_pose = frame.pose;
            const auto& current_match_types = frame.match_types;

            if (!current_submap_) current_submap_ = std::make_shared<Submap>();  // first frame of a new submap

            if (viewer_interface_) {
                viewer_interface_->insertCurrentKeyframe(current_submap_->getId(), current_pose,
                                                         pcl_utils::PclCloudToVecCloud(current_cloud),
                                                         current_match_types);
            }

            const bool is_keyframe = this->isKeyFrame(current_pose);
            SLAMResultRecorder::recordFrontendFrame(frame.timestamp, current_submap_->getId(), current_pose,
                                                    is_keyframe);

            if (!is_keyframe) continue;

            current_submap_->addFrame(current_cloud, current_pose);

            if (this->isSubmapFinished(current_submap_)) { this->finalizeCurrentSubmap(); }
        }

        if (timed_out || stopping_.load(std::memory_order_acquire)) { this->finalizeCurrentSubmap(); }

        if (loop_closure_) this->fetchAndApplyLoopClosures();

        if (viewer_interface_) viewer_interface_->refreshDrawables();

        if (stopping_.load(std::memory_order_acquire)) {
            SLAMResultRecorder::flush();
            break;
        }
    }
}

bool Backend::isKeyFrame(const Eigen::Isometry3d& current_pose) {
    if (is_first_frame_) {
        is_first_frame_ = false;
        last_kf_pose_ = current_pose;
        return true;
    }
    Eigen::Isometry3d relative_pose = last_kf_pose_.inverse() * current_pose;
    double trans_diff = relative_pose.translation().norm();
    double rot_diff = Eigen::AngleAxisd(relative_pose.rotation()).angle() * (180.0 / M_PI);  // in degrees

    if (trans_diff > config_.kf_trans_threshold || rot_diff > config_.kf_degree_threshold) {
        last_kf_pose_ = current_pose;
        return true;
    }

    return false;
}

bool Backend::isSubmapFinished(const SubmapPtr& submap) const {
    bool is_finished = false;

    is_finished |= submap->getNumFrames() >= config_.kf_max_num_submap;

    is_finished |= submap->getBeginEndFrameDistance() >= config_.kf_max_dist_submap;

    return is_finished;
}

void Backend::finalizeCurrentSubmap() {
    if (!current_submap_) return;
    if (current_submap_->getNumFrames() == 0) {
        LOG(WARNING) << "Skip empty submap " << current_submap_->getId() << " during finalize";
        current_submap_.reset();
        return;
    }

    current_submap_->setFinished(true, 0.1);

    {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        finished_submaps_.insert({current_submap_->getId(), current_submap_});
    }

    if (viewer_interface_) {
        static constexpr float kViewerSubmapVoxelResolution = 0.1f;
        VoxelGrid viewer_voxel_filter(kViewerSubmapVoxelResolution, SamplingMode::MedianRepresentative);
        CloudPtr viewer_cloud = pcl_utils::makeCloud<PointType>();
        viewer_voxel_filter.filter(current_submap_->getCloud(), viewer_cloud);

        viewer_interface_->insertFinishedSubmap(current_submap_->getId(),
                                                {current_submap_->getOriginOpti(), current_submap_->getFramePoses(),
                                                 pcl_utils::PclCloudToVecCloud(viewer_cloud)});
    }

    factor_graph_->addNode(current_submap_->getId(), current_submap_->getOriginOpti());

    Timer::measure("FactorGraph Optimize: ", [&]() { factor_graph_->optimize(); });

    IDPosesPtr optimized_poses = std::make_shared<IDPoses>();

    {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        for (auto& [id, submap] : finished_submaps_) {
            Eigen::Isometry3d optimized_pose;
            factor_graph_->getPose(id, optimized_pose);
            submap->updateOriginOpti(optimized_pose);
            optimized_poses->emplace_back(id, optimized_pose);
        }
    }

    this->updateRecordedSubmaps();

    if (viewer_interface_) { viewer_interface_->updateFinishedSubmapPose(optimized_poses); }

    if (loop_closure_) loop_closure_->insert(optimized_poses);

    current_submap_->releaseCloud();  // release cloud to save memory
    current_submap_.reset();
}

void Backend::fetchAndApplyLoopClosures() {
    auto loops = loop_closure_->fetchVerified();
    if (loops.empty()) return;

    for (const auto& lc : loops) {
        factor_graph_->addLoopClosureEdge(lc.i, lc.j, lc.meas);
        SLAMResultRecorder::recordLoopEdge(lc.i, lc.j, lc.meas);
        if (viewer_interface_) viewer_interface_->insertEdge(lc.i, lc.j);
    }

    Timer::measure("FactorGraph Optimize: ", [&]() { factor_graph_->optimize(); });

    IDPosesPtr optimized_poses = std::make_shared<IDPoses>();

    {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        for (auto& [id, submap] : finished_submaps_) {
            Eigen::Isometry3d optimized_pose;
            factor_graph_->getPose(id, optimized_pose);
            submap->updateOriginOpti(optimized_pose);
            optimized_poses->emplace_back(id, optimized_pose);
        }
    }

    this->updateRecordedSubmaps();

    if (viewer_interface_) { viewer_interface_->updateFinishedSubmapPose(optimized_poses); }
}

void Backend::updateRecordedSubmaps() {
    Timer::measure("SLAM Result Recorder: ", [&]() {
        std::unique_lock<std::mutex> lock(mutex_finished_submaps_);
        for (const auto& [id, submap] : finished_submaps_) {
            if (!submap) continue;
            SLAMResultRecorder::updateSubmap(id, submap->getPCDPath(), submap->getOrigin(), submap->getOriginOpti());
        }
    });
}

}  // namespace legkilo
