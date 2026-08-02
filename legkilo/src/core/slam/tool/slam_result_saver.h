// SPDX-License-Identifier: MIT
// @file save_slam_result.h
// @brief SLAM Result Saver interface.
// @author Ou Guangjun
// @created 2026-03-29
// @maintainer ouguangjun98@gmail.com

#ifndef LEGKILO_SAVE_SLAM_RESULT_H
#define LEGKILO_SAVE_SLAM_RESULT_H

#include <Eigen/Geometry>

#include <string>
#include <unordered_map>
#include <vector>

namespace guik {
class ProgressInterface;
}

namespace legkilo {

enum class CoordType { IMU = 0, LIDAR = 1 };

enum class TrajectoryFormat { TUM = 0, KITTI = 1 };

enum class TrajectoryType { FRONTEND = 0, BACKEND = 1, ALL = 2 };

enum class GlobalMapSizeType { TILED = 0, SINGLE = 1, SUBMAP = 2 };

class SLAMResultSaver {
   public:
    SLAMResultSaver();

    ~SLAMResultSaver() = default;

    bool run(guik::ProgressInterface& progress_interface);

    void setInputFolder(const std::string& folder_path) { input_result_folder_ = folder_path; }

    void setResultFolder(const std::string& folder_path) { save_result_folder_ = folder_path; }

    void setTrajectoryEnable(CoordType coord_type, TrajectoryFormat format, TrajectoryType type);

    void setGlobalMapEnable(CoordType coord_type, GlobalMapSizeType size_type, double tile_size = 50.0,
                            bool downsample_enable = false, double voxel_resolution = 0.2);

    void setSLAMRunningResultEnable();

   private:
    struct FrameRecord {
        uint64_t frame_id = 0;
        double timestamp = 0.0;
        int64_t submap_id = -1;
        bool is_keyframe = false;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    };

    struct SubmapRecord {
        int64_t submap_id = -1;
        std::string pcd_path;
        Eigen::Isometry3d frontend_origin = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d backend_origin = Eigen::Isometry3d::Identity();
    };

    struct StampedPose {
        double timestamp = 0.0;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    };

    struct LoopEdgeRecord {
        uint64_t edge_id = 0;
        int64_t i = -1;
        int64_t j = -1;
        Eigen::Isometry3d meas = Eigen::Isometry3d::Identity();
    };

    void prepareOutputFolder() const;

    void loadRuntimeRecords();

    void copyConfigSnapshot(std::vector<std::string>& exported_files) const;

    void saveTrajectories(std::vector<std::string>& exported_files) const;

    void saveGlobalMap(std::vector<std::string>& exported_files) const;

    void saveSLAMRunningResults(std::vector<std::string>& exported_files) const;

    void saveSubmapGlobalMap(std::vector<std::string>& exported_files) const;

    void saveSingleGlobalMap(std::vector<std::string>& exported_files) const;

    void saveTiledGlobalMap(std::vector<std::string>& exported_files) const;

    void writeDocuments(const std::vector<std::string>& exported_files) const;

    std::vector<StampedPose> buildFrontendTrajectory() const;

    std::vector<StampedPose> buildBackendTrajectory() const;

   private:
    std::string input_result_folder_ = "";

    std::string save_result_folder_ = "";

    // trajectory
    bool traj_enable_ = false;
    CoordType traj_coord_type_ = CoordType::IMU;
    TrajectoryFormat traj_format_ = TrajectoryFormat::TUM;
    TrajectoryType traj_type_ = TrajectoryType::ALL;

    // global map
    bool global_map_enable_ = false;
    CoordType global_map_coord_type_ = CoordType::IMU;
    GlobalMapSizeType global_map_size_type_ = GlobalMapSizeType::TILED;
    double global_map_tile_size_ = 50.0;
    bool global_map_downsample_enable_ = false;
    double global_map_voxel_resolution_ = 0.2;

    // slam running variables
    bool slam_running_enable_ = false;

    std::vector<FrameRecord> frames_;

    std::unordered_map<int64_t, SubmapRecord> submaps_;

    std::vector<LoopEdgeRecord> loop_edges_;

    Eigen::Isometry3d T_imu_lidar_ = Eigen::Isometry3d::Identity();
};

}  // namespace legkilo
#endif  // LEGKILO_SAVE_SLAM_RESULT_H
