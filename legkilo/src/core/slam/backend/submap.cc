#include "core/slam/backend/submap.h"

#include <exception>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "common/voxel_grid.hpp"

namespace legkilo {

void Submap::addFrame(const CloudPtr& frame, const Eigen::Isometry3d& pose) {
    if (finished_) return;
    if (!frame || frame->empty()) return;

    if (this->getNumFrames() == 0) {
        origin_ = pose;
        origin_opti_ = pose;
    }

    Eigen::Isometry3d relative_pose = origin_.inverse() * pose;
    PointCloudType transformed_frame;
    pcl::transformPointCloud(*frame, transformed_frame, relative_pose.matrix().cast<float>());
    *cloud_sum_ += transformed_frame;

    related_frame_poses_.emplace_back(relative_pose);
    frame_poses_.emplace_back(pose);
}

void Submap::setFinished(bool downsample, double resolution) {
    if (finished_) return;
    finished_ = true;

    if (downsample && cloud_sum_) {
        VoxelGrid voxel_filter(resolution, SamplingMode::MedianRepresentative);
        CloudPtr cloud_filtered = pcl_utils::makeCloud<PointType>();
        voxel_filter.filter(cloud_sum_, cloud_filtered);
        cloud_sum_ = cloud_filtered;
    }

    this->savePCD();
}

void Submap::releaseCloud() { cloud_sum_.reset(); }

bool Submap::savePCD() {
    if (!cloud_sum_ || cloud_sum_->empty() || save_path_folder.empty()) return false;
    const std::string file_path = save_path_folder + "/submap_" + std::to_string(id_) + ".pcd";
    try {
        return pcl::io::savePCDFileBinaryCompressed(file_path, *cloud_sum_) == 0;
    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to save submap PCD: " << file_path << ", error: " << e.what();
        return false;
    }
}

CloudPtr Submap::loadPCD() {
    if (save_path_folder.empty()) return nullptr;
    if (!cloud_sum_) {
        const std::string file_path = save_path_folder + "/submap_" + std::to_string(id_) + ".pcd";
        cloud_sum_ = pcl_utils::makeCloud<PointType>();
        if (pcl::io::loadPCDFile(file_path, *cloud_sum_) != 0) cloud_sum_.reset();
    }
    return cloud_sum_;
}

Submap::NodeType Submap::generateGlobalId() {
    static NodeType global_id = 0;
    return global_id++;
}

double Submap::getBeginEndFrameDistance() const {
    if (related_frame_poses_.size() < 2) return 0.0;
    const Eigen::Vector3d begin_pos = related_frame_poses_.front().translation();
    const Eigen::Vector3d end_pos = related_frame_poses_.back().translation();
    return (end_pos - begin_pos).norm();
}

}  // namespace legkilo
