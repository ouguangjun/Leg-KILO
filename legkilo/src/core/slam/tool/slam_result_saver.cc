#include "core/slam/tool/slam_result_saver.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <list>
#include <sstream>
#include <stdexcept>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include "common/file_io.hpp"
#include "common/math_utils.hpp"
#include "common/voxel_grid.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/tool/tiled_map.h"
#include "guik/progress_interface.hpp"

namespace legkilo {

namespace {
namespace fs = boost::filesystem;

std::string joinPath(const std::string& dir, const std::string& name) {
    if (dir.empty()) return name;
    if (dir.back() == '/') return dir + name;
    return dir + "/" + name;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) cols.push_back(item);
    return cols;
}

Eigen::Isometry3d parsePose(const std::vector<std::string>& cols, size_t start_idx) {
    if (cols.size() < start_idx + 7) { throw std::runtime_error("Invalid pose record"); }

    const Vec3D t(std::stod(cols[start_idx]), std::stod(cols[start_idx + 1]), std::stod(cols[start_idx + 2]));
    Eigen::Quaterniond q(std::stod(cols[start_idx + 6]), std::stod(cols[start_idx + 3]), std::stod(cols[start_idx + 4]),
                         std::stod(cols[start_idx + 5]));
    q.normalize();
    return makeIsometry3d(q.toRotationMatrix(), t);
}

std::string coordTypeToString(CoordType coord_type) { return coord_type == CoordType::IMU ? "imu" : "lidar"; }

std::string formatToString(TrajectoryFormat format) { return format == TrajectoryFormat::TUM ? "tum" : "kitti"; }

std::string trajectoryTypeToString(TrajectoryType type) {
    switch (type) {
        case TrajectoryType::FRONTEND: return "frontend";
        case TrajectoryType::BACKEND: return "backend";
        case TrajectoryType::ALL: return "all";
    }
    return "unknown";
}

std::string globalMapSizeTypeToString(GlobalMapSizeType type) {
    switch (type) {
        case GlobalMapSizeType::TILED: return "tiled";
        case GlobalMapSizeType::SINGLE: return "single";
        case GlobalMapSizeType::SUBMAP: return "submap";
    }
    return "unknown";
}

Eigen::Isometry3d loadImuToLidar(const std::string& config_path) {
    if (config_path.empty() || !fs::exists(config_path)) { return Eigen::Isometry3d::Identity(); }

    YamlHelper yaml_helper(config_path);
    const auto ext_t = yaml_helper.get<std::vector<double>>("extrinsic_T");
    const auto ext_r = yaml_helper.get<std::vector<double>>("extrinsic_R");

    Vec3D t;
    Mat3D r;
    t << VEC_FROM_ARRAY(ext_t);
    r << MAT_FROM_ARRAY(ext_r);
    return makeIsometry3d(r, t);
}

Eigen::Isometry3d convertPose(const Eigen::Isometry3d& pose_imu, CoordType coord_type,
                              const Eigen::Isometry3d& T_imu_lidar) {
    if (coord_type == CoordType::IMU) return pose_imu;
    return pose_imu * T_imu_lidar;
}

std::string makeTrajectoryFilename(TrajectoryType type, CoordType coord_type, TrajectoryFormat format) {
    return trajectoryTypeToString(type) + "_" + coordTypeToString(coord_type) + "_" + formatToString(format) + ".txt";
}

std::string normalizePath(const std::string& path) { return fs::absolute(fs::path(path)).lexically_normal().string(); }

bool isPathPrefix(const fs::path& prefix, const fs::path& path) {
    auto prefix_it = prefix.begin();
    auto path_it = path.begin();
    for (; prefix_it != prefix.end() && path_it != path.end(); ++prefix_it, ++path_it) {
        if (*prefix_it != *path_it) return false;
    }
    return prefix_it == prefix.end();
}

CloudPtr loadCloud(const std::string& filepath) {
    if (!fs::exists(filepath)) { throw std::runtime_error("Failed to find point cloud: " + filepath); }

    CloudPtr cloud = pcl_utils::makeCloud<PointType>();
    if (pcl::io::loadPCDFile(filepath, *cloud) != 0) {
        throw std::runtime_error("Failed to load point cloud: " + filepath);
    }
    return cloud;
}

void saveCloud(const std::string& filepath, const CloudPtr& cloud) {
    if (!cloud) { throw std::runtime_error("Point cloud is null: " + filepath); }
    if (pcl::io::savePCDFileBinaryCompressed(filepath, *cloud) != 0) {
        throw std::runtime_error("Failed to save point cloud: " + filepath);
    }
}

CloudPtr transformCloud(const CloudPtr& cloud, const Eigen::Isometry3d& pose) {
    CloudPtr transformed = pcl_utils::makeCloud<PointType>();
    if (!cloud || cloud->empty()) return transformed;
    pcl::transformPointCloud(*cloud, *transformed, pose.matrix().cast<float>());
    return transformed;
}

CloudPtr downsampleCloud(const CloudPtr& cloud, double resolution) {
    if (!cloud || cloud->empty()) return pcl_utils::makeCloud<PointType>();
    VoxelGrid voxel_filter(static_cast<float>(std::max(0.05, resolution)), SamplingMode::MedianRepresentative);
    CloudPtr cloud_downsampled = pcl_utils::makeCloud<PointType>();
    voxel_filter.filter(cloud, cloud_downsampled);
    return cloud_downsampled;
}

Eigen::Isometry3d makeGlobalMapTransform(const Eigen::Isometry3d& backend_origin, CoordType coord_type,
                                         const Eigen::Isometry3d& T_imu_lidar) {
    if (coord_type == CoordType::IMU) return backend_origin;
    return T_imu_lidar.inverse() * backend_origin;
}

void writePoseCsv(std::ofstream& ofs, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond q(pose.rotation());
    ofs << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << "," << q.x()
        << "," << q.y() << "," << q.z() << "," << q.w();
}

TiledMap::TileKey makeTileKey(double x, double y, double tile_size) {
    return {static_cast<int>(std::floor(x / tile_size)), static_cast<int>(std::floor(y / tile_size))};
}

void writeTileScheme(const std::string& filepath, CoordType coord_type, double tile_size, bool downsample_enable,
                     double voxel_resolution) {
    std::ofstream ofs(filepath, std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) { throw std::runtime_error("Failed to write tile scheme: " + filepath); }

    ofs << "tile_size: " << tile_size << "\n";
    ofs << "coord_type: " << coordTypeToString(coord_type) << "\n";
    ofs << "cache_tile_num: 20\n";
    ofs << "downsample_enable: " << (downsample_enable ? "true" : "false") << "\n";
    ofs << "voxel_resolution: " << voxel_resolution << "\n";
}

void copyFileOverwrite(const std::string& src, const std::string& dst) {
    if (!fs::exists(src)) { throw std::runtime_error("Failed to find file: " + src); }
    fs::copy_file(src, dst, fs::copy_option::overwrite_if_exists);
}

}  // namespace

SLAMResultSaver::SLAMResultSaver() = default;

void SLAMResultSaver::setGlobalMapEnable(CoordType coord_type, GlobalMapSizeType size_type, double tile_size,
                                         bool downsample_enable, double voxel_resolution) {
    global_map_enable_ = true;
    global_map_coord_type_ = coord_type;
    global_map_size_type_ = size_type;
    global_map_tile_size_ = std::max(20.0, tile_size);
    global_map_downsample_enable_ = downsample_enable;
    global_map_voxel_resolution_ = std::max(0.05, voxel_resolution);
}

void SLAMResultSaver::setTrajectoryEnable(CoordType coord_type, TrajectoryFormat format, TrajectoryType type) {
    traj_enable_ = true;
    traj_coord_type_ = coord_type;
    traj_format_ = format;
    traj_type_ = type;
}

void SLAMResultSaver::setSLAMRunningResultEnable() { slam_running_enable_ = true; }

bool SLAMResultSaver::run(guik::ProgressInterface& progress_interface) {
    try {
        progress_interface.set_title("Saving SLAM Result");
        progress_interface.set_maximum(6);

        progress_interface.set_text("Preparing output folder");
        prepareOutputFolder();
        progress_interface.increment();

        progress_interface.set_text("Loading runtime records");
        if (traj_enable_ || global_map_enable_ || slam_running_enable_) { loadRuntimeRecords(); }
        progress_interface.increment();

        std::vector<std::string> exported_files;
        copyConfigSnapshot(exported_files);

        progress_interface.set_text("Saving trajectories");
        saveTrajectories(exported_files);
        progress_interface.increment();

        progress_interface.set_text("Saving global map");
        saveGlobalMap(exported_files);
        progress_interface.increment();

        progress_interface.set_text("Saving SLAM runtime results");
        saveSLAMRunningResults(exported_files);
        progress_interface.increment();

        progress_interface.set_text("Writing documents");
        writeDocuments(exported_files);
        progress_interface.increment();

        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Save SLAM Result failed: " << e.what();
        return false;
    }
}

void SLAMResultSaver::prepareOutputFolder() const {
    if (input_result_folder_.empty()) { throw std::runtime_error("Input result folder is empty"); }
    if (save_result_folder_.empty()) { throw std::runtime_error("Output result folder is empty"); }

    const fs::path input_path = fs::path(normalizePath(input_result_folder_));
    const fs::path output_path = fs::path(normalizePath(save_result_folder_));
    if (input_path == output_path || isPathPrefix(output_path, input_path)) {
        throw std::runtime_error("Output folder cannot be the same as or parent of input result folder");
    }

    file_io::ensureDirectory(save_result_folder_);
    file_io::clearDirectoryContents(save_result_folder_);
}

void SLAMResultSaver::loadRuntimeRecords() {
    frames_.clear();
    submaps_.clear();
    loop_edges_.clear();

    const std::string frames_path = joinPath(input_result_folder_, "frontend_frames.csv");
    std::ifstream frames_ifs(frames_path);
    if (!frames_ifs.is_open()) {
        if (traj_enable_) { throw std::runtime_error("Failed to open frame record: " + frames_path); }
    } else {
        std::string line;
        std::getline(frames_ifs, line);
        while (std::getline(frames_ifs, line)) {
            if (line.empty()) continue;
            const auto cols = splitCsvLine(line);
            if (cols.size() < 11) continue;

            FrameRecord frame;
            frame.frame_id = static_cast<uint64_t>(std::stoull(cols[0]));
            frame.timestamp = std::stod(cols[1]);
            frame.submap_id = std::stoll(cols[2]);
            frame.is_keyframe = std::stoi(cols[3]) != 0;
            frame.pose = parsePose(cols, 4);
            frames_.push_back(frame);
        }
    }

    const std::string submaps_path = joinPath(input_result_folder_, "submaps.csv");
    std::ifstream submaps_ifs(submaps_path);
    if (!submaps_ifs.is_open()) {
        if (global_map_enable_ || slam_running_enable_) {
            throw std::runtime_error("Failed to open submap record: " + submaps_path);
        }
    } else {
        std::string line;
        std::getline(submaps_ifs, line);
        while (std::getline(submaps_ifs, line)) {
            if (line.empty()) continue;
            const auto cols = splitCsvLine(line);
            if (cols.size() < 16) continue;

            SubmapRecord submap;
            submap.submap_id = std::stoll(cols[0]);
            submap.pcd_path = cols[1];
            submap.frontend_origin = parsePose(cols, 2);
            submap.backend_origin = parsePose(cols, 9);
            submaps_.insert({submap.submap_id, submap});
        }
    }

    const std::string loop_edges_path = joinPath(input_result_folder_, "loop_edges.csv");
    std::ifstream loop_edges_ifs(loop_edges_path);
    if (loop_edges_ifs.is_open()) {
        std::string line;
        std::getline(loop_edges_ifs, line);
        while (std::getline(loop_edges_ifs, line)) {
            if (line.empty()) continue;
            const auto cols = splitCsvLine(line);
            if (cols.size() < 10) continue;

            LoopEdgeRecord edge;
            edge.edge_id = static_cast<uint64_t>(std::stoull(cols[0]));
            edge.i = std::stoll(cols[1]);
            edge.j = std::stoll(cols[2]);
            edge.meas = parsePose(cols, 3);
            loop_edges_.push_back(edge);
        }
    }

    T_imu_lidar_ = loadImuToLidar(joinPath(input_result_folder_, "config.yaml"));
}

void SLAMResultSaver::copyConfigSnapshot(std::vector<std::string>& exported_files) const {
    const std::string input_config = joinPath(input_result_folder_, "config.yaml");
    if (!fs::exists(input_config)) return;

    const std::string configs_dir = joinPath(save_result_folder_, "configs");
    file_io::ensureDirectory(configs_dir);
    const std::string output_config = joinPath(configs_dir, "slam.yaml");
    fs::copy_file(input_config, output_config, fs::copy_option::overwrite_if_exists);
    exported_files.push_back("configs/slam.yaml");
}

void SLAMResultSaver::saveTrajectories(std::vector<std::string>& exported_files) const {
    if (!traj_enable_) return;
    if (frames_.empty()) { throw std::runtime_error("No frontend frame records found"); }

    const std::string trajectories_dir = joinPath(save_result_folder_, "trajectories");
    file_io::ensureDirectory(trajectories_dir);

    auto save_trajectory = [&](TrajectoryType type, const std::vector<StampedPose>& trajectory) {
        const std::string filename = makeTrajectoryFilename(type, traj_coord_type_, traj_format_);
        const std::string filepath = joinPath(trajectories_dir, filename);
        std::ofstream ofs(filepath, std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) { throw std::runtime_error("Failed to open trajectory file: " + filepath); }

        if (traj_format_ == TrajectoryFormat::TUM) {
            for (const auto& entry : trajectory) {
                const Eigen::Quaterniond q(entry.pose.rotation());
                ofs << std::fixed << std::setprecision(9) << entry.timestamp << " " << entry.pose.translation().x()
                    << " " << entry.pose.translation().y() << " " << entry.pose.translation().z() << " " << q.x() << " "
                    << q.y() << " " << q.z() << " " << q.w() << "\n";
            }
        } else {
            for (const auto& entry : trajectory) {
                const auto& T = entry.pose.matrix();
                ofs << std::fixed << std::setprecision(9) << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " "
                    << T(0, 3) << " " << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " " << T(2, 0)
                    << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << "\n";
            }

            std::ofstream ts_ofs(filepath.substr(0, filepath.size() - 4) + "_timestamps.txt",
                                 std::ios::out | std::ios::trunc);
            if (!ts_ofs.is_open()) { throw std::runtime_error("Failed to open KITTI timestamp file: " + filepath); }
            for (const auto& entry : trajectory) {
                ts_ofs << std::fixed << std::setprecision(9) << entry.timestamp << "\n";
            }
            exported_files.push_back("trajectories/" + filename.substr(0, filename.size() - 4) + "_timestamps.txt");
        }

        exported_files.push_back("trajectories/" + filename);
    };

    if (traj_type_ == TrajectoryType::FRONTEND || traj_type_ == TrajectoryType::ALL) {
        save_trajectory(TrajectoryType::FRONTEND, buildFrontendTrajectory());
    }

    if (traj_type_ == TrajectoryType::BACKEND || traj_type_ == TrajectoryType::ALL) {
        save_trajectory(TrajectoryType::BACKEND, buildBackendTrajectory());
    }
}

void SLAMResultSaver::saveGlobalMap(std::vector<std::string>& exported_files) const {
    if (!global_map_enable_) return;
    if (submaps_.empty()) { throw std::runtime_error("No submap records found"); }

    switch (global_map_size_type_) {
        case GlobalMapSizeType::SUBMAP: saveSubmapGlobalMap(exported_files); break;
        case GlobalMapSizeType::SINGLE: saveSingleGlobalMap(exported_files); break;
        case GlobalMapSizeType::TILED: saveTiledGlobalMap(exported_files); break;
    }
}

void SLAMResultSaver::saveSLAMRunningResults(std::vector<std::string>& exported_files) const {
    if (!slam_running_enable_) return;
    if (submaps_.empty()) { throw std::runtime_error("No submap records found"); }

    const std::string runtime_dir = joinPath(save_result_folder_, "slam_runtime");
    const std::string submaps_dir = joinPath(runtime_dir, "submaps");
    file_io::ensureDirectory(runtime_dir);
    file_io::ensureDirectory(submaps_dir);

    std::vector<SubmapRecord> submap_records;
    submap_records.reserve(submaps_.size());
    for (const auto& [id, submap] : submaps_) { submap_records.push_back(submap); }
    std::sort(submap_records.begin(), submap_records.end(),
              [](const SubmapRecord& lhs, const SubmapRecord& rhs) { return lhs.submap_id < rhs.submap_id; });

    {
        std::ofstream ofs(joinPath(runtime_dir, "submap_index.csv"), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) { throw std::runtime_error("Failed to write runtime submap index"); }
        ofs << "submap_id,tx,ty,tz,qx,qy,qz,qw,pcd_path\n";

        for (const auto& submap : submap_records) {
            const std::string filename = "submap_" + std::to_string(submap.submap_id) + ".pcd";
            copyFileOverwrite(submap.pcd_path, joinPath(submaps_dir, filename));
            ofs << submap.submap_id << ",";
            writePoseCsv(ofs, submap.backend_origin);
            ofs << ",submaps/" << filename << "\n";
        }
    }

    {
        std::ofstream ofs(joinPath(runtime_dir, "loop_edges.csv"), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) { throw std::runtime_error("Failed to write runtime loop edges"); }
        ofs << "edge_id,i,j,tx,ty,tz,qx,qy,qz,qw\n";
        for (const auto& edge : loop_edges_) {
            ofs << edge.edge_id << "," << edge.i << "," << edge.j << ",";
            writePoseCsv(ofs, edge.meas);
            ofs << "\n";
        }
    }

    {
        const std::string config_path = joinPath(input_result_folder_, "config.yaml");
        std::ofstream ofs(joinPath(runtime_dir, "graph_summary.yaml"), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) { throw std::runtime_error("Failed to write graph summary"); }

        YamlHelper yaml(config_path);
        ofs << "odom_near_factor_weight: " << yaml.get<double>("odom_near_factor_weight", 30.0) << "\n";
        ofs << "odom_next_factor_weight: " << yaml.get<double>("odom_next_factor_weight", 15.0) << "\n";
        ofs << "loopclosure_factor_weight: " << yaml.get<double>("loopclosure_factor_weight", 50.0) << "\n";
        ofs << "loopclosure_loss_scale: " << yaml.get<double>("loopclosure_loss_scale", 5.0) << "\n";
    }

    {
        std::ofstream ofs(joinPath(runtime_dir, "README.md"), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) { throw std::runtime_error("Failed to write runtime README"); }

        ofs << "# SLAM Runtime Result\n\n";
        ofs << "This folder stores intermediate backend results for offline map editing.\n\n";
        ofs << "## Files\n\n";
        ofs << "- `submap_index.csv`: optimized backend pose of each submap and its copied local point cloud path.\n";
        ofs << "- `submaps/`: copied local submap point clouds from the runtime tmp folder.\n";
        ofs << "- `loop_edges.csv`: verified loop-closure edges only, stored as relative pose `T_i_j`.\n";
        ofs << "- `graph_summary.yaml`: factor-graph and loop-closure weights used by the backend.\n";
    }

    exported_files.push_back("slam_runtime/submap_index.csv");
    exported_files.push_back("slam_runtime/loop_edges.csv");
    exported_files.push_back("slam_runtime/graph_summary.yaml");
    exported_files.push_back("slam_runtime/README.md");
    exported_files.push_back("slam_runtime/submaps/");
}

void SLAMResultSaver::saveSubmapGlobalMap(std::vector<std::string>& exported_files) const {
    const std::string submaps_dir = joinPath(save_result_folder_, "submaps_global");
    const std::string clouds_dir = joinPath(submaps_dir, "clouds");
    file_io::ensureDirectory(clouds_dir);

    std::vector<SubmapRecord> submap_records;
    submap_records.reserve(submaps_.size());
    for (const auto& [id, submap] : submaps_) { submap_records.push_back(submap); }
    std::sort(submap_records.begin(), submap_records.end(),
              [](const SubmapRecord& lhs, const SubmapRecord& rhs) { return lhs.submap_id < rhs.submap_id; });

    std::ofstream ofs(joinPath(submaps_dir, "submap_index.csv"), std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) { throw std::runtime_error("Failed to write submap index"); }
    ofs << "submap_id,point_num,tx,ty,tz,qx,qy,qz,qw,pcd_path\n";

    for (const auto& submap : submap_records) {
        const Eigen::Isometry3d map_transform =
            makeGlobalMapTransform(submap.backend_origin, global_map_coord_type_, T_imu_lidar_);
        CloudPtr cloud_global = transformCloud(loadCloud(submap.pcd_path), map_transform);

        const std::string filename = "submap_" + std::to_string(submap.submap_id) + ".pcd";
        saveCloud(joinPath(clouds_dir, filename), cloud_global);

        ofs << submap.submap_id << "," << cloud_global->size() << ",";
        writePoseCsv(ofs, map_transform);
        ofs << "," << "clouds/" << filename << "\n";
    }

    exported_files.push_back("submaps_global/submap_index.csv");
    exported_files.push_back("submaps_global/clouds/");
}

void SLAMResultSaver::saveSingleGlobalMap(std::vector<std::string>& exported_files) const {
    const std::string global_map_dir = joinPath(save_result_folder_, "global_map");
    file_io::ensureDirectory(global_map_dir);

    CloudPtr cloud_global = pcl_utils::makeCloud<PointType>();

    std::vector<SubmapRecord> submap_records;
    submap_records.reserve(submaps_.size());
    for (const auto& [id, submap] : submaps_) { submap_records.push_back(submap); }
    std::sort(submap_records.begin(), submap_records.end(),
              [](const SubmapRecord& lhs, const SubmapRecord& rhs) { return lhs.submap_id < rhs.submap_id; });

    for (const auto& submap : submap_records) {
        const Eigen::Isometry3d map_transform =
            makeGlobalMapTransform(submap.backend_origin, global_map_coord_type_, T_imu_lidar_);
        CloudPtr cloud_transformed = transformCloud(loadCloud(submap.pcd_path), map_transform);
        *cloud_global += *cloud_transformed;
    }

    if (global_map_downsample_enable_) { cloud_global = downsampleCloud(cloud_global, global_map_voxel_resolution_); }

    saveCloud(joinPath(global_map_dir, "global_map.pcd"), cloud_global);

    std::ofstream ofs(joinPath(global_map_dir, "summary.yaml"), std::ios::out | std::ios::trunc);
    if (!ofs.is_open()) { throw std::runtime_error("Failed to write global map summary"); }
    ofs << "coord_type: " << coordTypeToString(global_map_coord_type_) << "\n";
    ofs << "downsample_enable: " << (global_map_downsample_enable_ ? "true" : "false") << "\n";
    ofs << "voxel_resolution: " << global_map_voxel_resolution_ << "\n";
    ofs << "point_num: " << cloud_global->size() << "\n";

    exported_files.push_back("global_map/global_map.pcd");
    exported_files.push_back("global_map/summary.yaml");
}

void SLAMResultSaver::saveTiledGlobalMap(std::vector<std::string>& exported_files) const {
    const std::string tiles_dir = joinPath(save_result_folder_, "map_tiles");
    file_io::ensureDirectory(tiles_dir);

    TiledMap tiled_map;
    tiled_map.setTileSize(global_map_tile_size_);
    tiled_map.setOutputFolder(tiles_dir);

    std::unordered_map<TiledMap::TileKey, TiledMap::TilePtr, TiledMap::TileKeyHash> active_tiles;
    std::unordered_map<TiledMap::TileKey, std::list<TiledMap::TileKey>::iterator, TiledMap::TileKeyHash> lru_iters;
    std::list<TiledMap::TileKey> lru_list;

    auto touch_tile = [&](const TiledMap::TileKey& key, const TiledMap::TilePtr& tile) {
        if (active_tiles.find(key) == active_tiles.end()) { active_tiles.insert({key, tile}); }

        const auto iter = lru_iters.find(key);
        if (iter != lru_iters.end()) { lru_list.erase(iter->second); }
        lru_list.push_back(key);
        lru_iters[key] = std::prev(lru_list.end());
    };

    auto evict_oldest = [&]() {
        if (lru_list.empty()) return;
        const TiledMap::TileKey oldest_key = lru_list.front();
        lru_list.pop_front();
        lru_iters.erase(oldest_key);

        const auto active_iter = active_tiles.find(oldest_key);
        if (active_iter == active_tiles.end()) return;

        if (!tiled_map.flushTile(active_iter->second)) {
            throw std::runtime_error("Failed to flush tile before eviction");
        }
        tiled_map.releaseTile(active_iter->second);
        active_tiles.erase(active_iter);
    };

    std::vector<SubmapRecord> submap_records;
    submap_records.reserve(submaps_.size());
    for (const auto& [id, submap] : submaps_) { submap_records.push_back(submap); }
    std::sort(submap_records.begin(), submap_records.end(),
              [](const SubmapRecord& lhs, const SubmapRecord& rhs) { return lhs.submap_id < rhs.submap_id; });

    for (const auto& submap : submap_records) {
        const Eigen::Isometry3d map_transform =
            makeGlobalMapTransform(submap.backend_origin, global_map_coord_type_, T_imu_lidar_);
        CloudPtr cloud_global = transformCloud(loadCloud(submap.pcd_path), map_transform);

        std::unordered_map<TiledMap::TileKey, CloudPtr, TiledMap::TileKeyHash> tile_chunks;
        for (const auto& pt : cloud_global->points) {
            const TiledMap::TileKey key = makeTileKey(pt.x, pt.y, global_map_tile_size_);
            auto& chunk = tile_chunks[key];
            if (!chunk) chunk = pcl_utils::makeCloud<PointType>();
            chunk->points.push_back(pt);
        }

        for (const auto& [key, chunk] : tile_chunks) {
            TiledMap::TilePtr tile = tiled_map.findTile(key);
            if (!tile) tile = tiled_map.getOrCreateTile(key);
            if (!tiled_map.loadTile(tile)) { throw std::runtime_error("Failed to load tile for update"); }
            touch_tile(key, tile);
            tiled_map.insertPoints(key, chunk);
            while (active_tiles.size() > 20) { evict_oldest(); }
        }
    }

    for (const auto& [key, tile] : active_tiles) {
        if (!tiled_map.flushTile(tile)) { throw std::runtime_error("Failed to flush final tile"); }
        tiled_map.releaseTile(tile);
    }

    if (global_map_downsample_enable_) {
        for (const auto& tile : tiled_map.getAllTiles()) {
            if (!tiled_map.loadTile(tile)) { throw std::runtime_error("Failed to load tile for downsampling"); }
            tile->cloud = downsampleCloud(tile->cloud, global_map_voxel_resolution_);
            tile->point_num = tile->cloud ? tile->cloud->size() : 0;
            tile->dirty = true;
            if (!tiled_map.flushTile(tile)) { throw std::runtime_error("Failed to save downsampled tile"); }
            tiled_map.releaseTile(tile);
        }
    }

    writeTileScheme(joinPath(tiles_dir, "tile_scheme.yaml"), global_map_coord_type_, global_map_tile_size_,
                    global_map_downsample_enable_, global_map_voxel_resolution_);
    tiled_map.writeIndex(joinPath(tiles_dir, "tile_index.csv"));

    exported_files.push_back("map_tiles/tile_scheme.yaml");
    exported_files.push_back("map_tiles/tile_index.csv");
    exported_files.push_back("map_tiles/clouds/");
}

void SLAMResultSaver::writeDocuments(const std::vector<std::string>& exported_files) const {
    {
        std::ofstream ofs(joinPath(save_result_folder_, "README.md"), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) { throw std::runtime_error("Failed to write README.md"); }

        ofs << "# SLAM Result\n\n";
        ofs << "This folder stores exported results from `" << input_result_folder_ << "`.\n\n";

        ofs << "## Trajectories\n\n";
        if (traj_enable_) {
            ofs << "- Coordinate: `" << coordTypeToString(traj_coord_type_) << "`\n";
            ofs << "- Format: `" << formatToString(traj_format_) << "`\n";
            ofs << "- Type: `" << trajectoryTypeToString(traj_type_) << "`\n";
            ofs << "- Frontend poses come from per-LiDAR-frame frontend IMU poses.\n";
            ofs << "- Backend poses are reconstructed from optimized submap poses.\n";
        } else {
            ofs << "- Trajectory export disabled.\n";
        }

        ofs << "\n## Global Map\n\n";
        if (global_map_enable_) {
            ofs << "- Mode: `" << globalMapSizeTypeToString(global_map_size_type_) << "`\n";
            ofs << "- Coordinate: `" << coordTypeToString(global_map_coord_type_) << "`\n";
            if (global_map_size_type_ == GlobalMapSizeType::TILED) {
                ofs << "- Tile size: `" << global_map_tile_size_ << "`\n";
                ofs << "- Active tile cache: `20`\n";
            }
            if (global_map_size_type_ != GlobalMapSizeType::SUBMAP) {
                ofs << "- Downsample: `" << (global_map_downsample_enable_ ? "enabled" : "disabled") << "`\n";
                if (global_map_downsample_enable_) {
                    ofs << "- Voxel resolution: `" << global_map_voxel_resolution_ << "`\n";
                }
            }
        } else {
            ofs << "- Global map export disabled.\n";
        }

        ofs << "\n## Runtime Result\n\n";
        if (slam_running_enable_) {
            ofs << "- Exported to `slam_runtime/` for offline editing.\n";
            ofs << "- Includes copied submap point clouds, optimized submap poses, verified loop edges, and graph "
                   "weights.\n";
        } else {
            ofs << "- Runtime intermediate result export disabled.\n";
        }

        ofs << "\n## Files\n\n";
        for (const auto& filepath : exported_files) ofs << "- `" << filepath << "`\n";
    }

    {
        std::ofstream ofs(joinPath(save_result_folder_, "manifest.yaml"), std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) { throw std::runtime_error("Failed to write manifest.yaml"); }

        ofs << "version: 1\n";
        ofs << "input_dir: " << input_result_folder_ << "\n";
        ofs << "trajectory:\n";
        ofs << "  enabled: " << (traj_enable_ ? "true" : "false") << "\n";
        ofs << "  coord_type: " << coordTypeToString(traj_coord_type_) << "\n";
        ofs << "  format: " << formatToString(traj_format_) << "\n";
        ofs << "  type: " << trajectoryTypeToString(traj_type_) << "\n";
        ofs << "global_map:\n";
        ofs << "  enabled: " << (global_map_enable_ ? "true" : "false") << "\n";
        ofs << "  coord_type: " << coordTypeToString(global_map_coord_type_) << "\n";
        ofs << "  size_type: " << globalMapSizeTypeToString(global_map_size_type_) << "\n";
        ofs << "  tile_size: " << global_map_tile_size_ << "\n";
        ofs << "  downsample_enable: " << (global_map_downsample_enable_ ? "true" : "false") << "\n";
        ofs << "  voxel_resolution: " << global_map_voxel_resolution_ << "\n";
        ofs << "runtime_result:\n";
        ofs << "  requested: " << (slam_running_enable_ ? "true" : "false") << "\n";
        ofs << "  exported: " << (slam_running_enable_ ? "true" : "false") << "\n";
        ofs << "files:\n";
        for (const auto& filepath : exported_files) ofs << "  - " << filepath << "\n";
    }
}

std::vector<SLAMResultSaver::StampedPose> SLAMResultSaver::buildFrontendTrajectory() const {
    std::vector<StampedPose> trajectory;
    trajectory.reserve(frames_.size());

    for (const auto& frame : frames_) {
        trajectory.push_back({frame.timestamp, convertPose(frame.pose, traj_coord_type_, T_imu_lidar_)});
    }

    return trajectory;
}

std::vector<SLAMResultSaver::StampedPose> SLAMResultSaver::buildBackendTrajectory() const {
    std::vector<StampedPose> trajectory;
    trajectory.reserve(frames_.size());

    size_t missing_submap_count = 0;
    for (const auto& frame : frames_) {
        const auto it = submaps_.find(frame.submap_id);
        if (it == submaps_.end()) {
            ++missing_submap_count;
            continue;
        }

        const auto& submap = it->second;
        const Eigen::Isometry3d relative_pose = submap.frontend_origin.inverse() * frame.pose;
        const Eigen::Isometry3d pose_backend_imu = submap.backend_origin * relative_pose;
        trajectory.push_back({frame.timestamp, convertPose(pose_backend_imu, traj_coord_type_, T_imu_lidar_)});
    }

    if (missing_submap_count > 0) {
        LOG(WARNING) << "Skipped " << missing_submap_count << " frames because matching submap poses were not found";
    }

    return trajectory;
}

}  // namespace legkilo
