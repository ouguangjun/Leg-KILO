// SPDX-License-Identifier: MIT
// @file viewer_slam_interface.h
// @brief SLAM viewer interface.
// @author Ou Guangjun
// @created 2026-02-07
// @maintainer ouguangjun98@gmail.com
// This file is inspired by the design of https://github.com/koide3/glim (MIT License).

#include "viewer/viewer_base.h"

#include <map>
#include <memory>
#include <unordered_set>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"

namespace guik {
class ProgressModal;
}

namespace legkilo {
class SaveSLAMResultModal;
}

namespace legkilo {

class ViewerSlamInterface : public ViewerBase {
   public:
    using NodeType = int64_t;
    using Pose = Eigen::Isometry3d;
    using IDPoses = std::vector<std::pair<NodeType, Pose>>;
    using IDPosesPtr = std::shared_ptr<IDPoses>;
    using CloudVec = std::vector<Eigen::Vector3f>;
    using CloudVecPtr = std::shared_ptr<CloudVec>;
    using KeyframePoses = std::vector<Pose, Eigen::aligned_allocator<Pose>>;  // from frontend
    using SubmapPosesCloud = std::tuple<Pose, KeyframePoses, CloudVecPtr>;    // <submap pose, frontend keyframe poses,
                                                                              // cloud>, submap pose from backend
    using EdgePair = UnorderedIntPairKey;

    ViewerSlamInterface(const std::string& config_path);

    ~ViewerSlamInterface();

    void insertFinishedSubmap(NodeType id, SubmapPosesCloud submap_pose_cloud);

    void insertCurrentKeyframe(NodeType id, const Pose& pose, CloudVecPtr keyframe_cloud,
                               LidarMatchTypesPtr match_types = nullptr);

    void insertEdge(NodeType id1, NodeType id2);

    void updateFinishedSubmapPose(IDPosesPtr submap_poses);

    void refreshDrawables();

    bool consumeSLAMTerminateRequest() { return request_slam_terminate_.exchange(false); }

    std::string name() const override { return "SLAM Viewer"; };

   protected:
    void workerLoop() override;

    void menuCallback() override;

    void runModal() override;

    void displayPanelCallback() override;

    using ViewerBase::kill_switch_;
    using ViewerBase::request_to_terminate_;
    using ViewerBase::task_deque_;
    using ViewerBase::task_mutex_;

   private:
    void updateCamera(double delta_seconds);

    void updateTransformFrontend2Backend();

    void requestSLAMTerminate() { request_slam_terminate_.store(true); }

   private:
    mutable std::mutex mutex_full_;  // accessing following data

    std::atomic_bool request_slam_terminate_;

    std::map<NodeType, SubmapPosesCloud> finished_submaps_;
    std::unordered_set<EdgePair, EdgePair::Hasher, EdgePair::Equal> finished_edges_;
    std::pair<NodeType, KeyframePoses> current_submap_poses_;
    std::pair<Pose, CloudVecPtr> current_keyframe_;
    LidarMatchTypesPtr current_keyframe_match_types_;

    // modal
    std::unique_ptr<SaveSLAMResultModal> save_slam_result_modal_;

    // visualization flags
    bool draw_finished_submaps_;
    bool draw_current_submap_;
    bool draw_trajectory_;
    bool draw_edges_;
    bool follow_current_;
    bool rotate_trajectory_;
    int max_submaps_display_;

    double point_alpha_;
    double point_size_;
    float rotate_speed_deg_per_sec_;
    std::string result_save_path_;

    // temporary variables
    bool is_finished_drawable_update_;
    bool is_current_keyframe_update_;
    std::vector<Eigen::Vector3f> traj_finished_;
    Pose T_frontend_backend_;  // Transformation from frontend to backend
};

}  // namespace legkilo
