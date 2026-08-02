// SPDX-License-Identifier: MIT
// @file submap.h
// @brief Submap representation
// @author Ou Guangjun
// @created 2026-01-28
// @maintainer ouguangjun98@gmail.com
#ifndef LEGKILO_SUBMAP_H
#define LEGKILO_SUBMAP_H

#include <memory>
#include <vector>

#include "common/pcl_types.h"
namespace legkilo {

class Submap {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using NodeType = int64_t;

    Submap() : id_(generateGlobalId()), finished_(false), cloud_sum_(new PointCloudType()) {}

    void addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose);

    void setFinished(bool downsample = true, double resolution = 0.1);

    bool isFinished() const { return finished_; }

    NodeType getId() const { return id_; }

    size_t getNumFrames() const { return frame_poses_.size(); }

    Eigen::Isometry3d getOrigin() const { return origin_; }

    Eigen::Isometry3d getOriginOpti() const { return origin_opti_; }

    std::string getPCDPath() const {
        if (save_path_folder.empty()) return "";
        if (save_path_folder.back() == '/') return save_path_folder + "submap_" + std::to_string(id_) + ".pcd";
        return save_path_folder + "/submap_" + std::to_string(id_) + ".pcd";
    }

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getFramePoses() const {
        return frame_poses_;
    }

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getRelatedFramePoses() const {
        return related_frame_poses_;
    }

    double getBeginEndFrameDistance() const;

    void updateOriginOpti(const Eigen::Isometry3d& new_origin) { origin_opti_ = new_origin; }

    CloudPtr getCloud() { return cloud_sum_ ? cloud_sum_ : this->loadPCD(); }

    void releaseCloud();

    static void setSavePathFolder(const std::string& dir) { save_path_folder = dir; }

   private:
    bool savePCD();

    CloudPtr loadPCD();

    static NodeType generateGlobalId();

   private:
    NodeType id_;

    bool finished_;  // Whether the submap is finished

    CloudPtr cloud_sum_;  // Sum of point clouds

    Eigen::Isometry3d origin_;  // Initial origin pose

    Eigen::Isometry3d origin_opti_;  // Optimized origin pose

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        related_frame_poses_;  // Relative poses of frames

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
        frame_poses_;  // frontend absolute poses of frames

    inline static std::string save_path_folder = "tmp";
};
using SubmapPtr = std::shared_ptr<Submap>;

}  // namespace legkilo

#endif  // LEGKILO_SUBMAP_H
