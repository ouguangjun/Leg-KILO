// SPDX-License-Identifier: MIT
// @file KILO.h
// @brief KILO SLAM orchestrator and pipeline interfaces.
// @author Ou Guangjun
// @created 2025-09-08
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_CORE_SLAM_KILO_H_
#define LEG_KILO_CORE_SLAM_KILO_H_

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/kernel.hpp"
#include "common/math_utils.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"
#include "interface/common/ros_compat.h"
namespace legkilo {
class ESKF;
class StateInitial;
class VoxelGrid;
class GaussianVoxelMap;
struct GaussPoint;
using GaussCloud = std::vector<GaussPoint, Eigen::aligned_allocator<GaussPoint>>;
}  // namespace legkilo

namespace legkilo {

struct ProcessResult {
    bool valid = false;
    size_t total_pts_size = 0;
    size_t down_pts_size = 0;
    size_t success_pts_size = 0;  // total effective points (p2p + ndt blocks)
    size_t p2p_count = 0;         // effective P2P points
    size_t ndt_count = 0;         // effective NDT points (blocks)
    CloudPtr cloud_world = nullptr;
    CloudPtr cloud_body = nullptr;
    CloudPtr cloud_lidar = nullptr;
    LidarMatchTypesPtr match_types = nullptr;

    friend std::ostream& operator<<(std::ostream& os, const ProcessResult& result);
};

class KILO {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit KILO(const std::string& config_file);
    ~KILO();

    ProcessResult process(common::MeasGroup& measure);

    Vec3D getPosImu() const;

    Mat3D getRotImu() const;

    Vec3D getPosLidar() const;

    Mat3D getRotLidar() const;

   private:
    void initializeFromYaml(const std::string& config_file);

    std::shared_ptr<GaussCloud> initializeMap(const CloudPtr& cloud_lidar);

    Mat3D computeWorldPointCov(const Vec3D& point_body, const Mat3D& cov_body, const Mat3D& rot, const Mat3D& rot_cov,
                               const Mat3D& pos_cov) const;

    void backpropagate(const GaussCloud& cloud_world, GaussCloud& cloud_body) const;

    bool eskfPredict(double current_time);

    bool predictUpdateImu(const ros_compat::ImuMsgPtr& imu);

    bool predictUpdateKinImu(const common::KinImuMeas& kin_imu);

    std::pair<size_t, size_t> predictUpdatePoint(const double current_time, const size_t idx_i, const size_t idx_j,
                                                 const CloudPtr& cloud_lidar_pcl, GaussCloud& cloud_body,
                                                 GaussCloud& cloud_world, LidarMatchTypes* match_types);

    std::pair<size_t, size_t> predictUpdateCloud(const GaussCloud& cloud_body, GaussCloud& cloud_world,
                                                 LidarMatchTypes* match_types);

    void resetLoPredictInput(double current_time);

    void updateLoPredictInput(double current_time);

   private:
    // Modules
    std::unique_ptr<ESKF> eskf_;
    std::unique_ptr<StateInitial> state_initial_;
    std::unique_ptr<VoxelGrid> voxel_grid_;
    std::unique_ptr<GaussianVoxelMap> gaussian_voxel_map_;

    // ESKF state/noise
    common::SensorType sensor_type_ = common::SensorType::LIO;
    bool two_step_lidar_eskf_ = true;
    bool p2p_enable_ = true;
    bool ndt_enable_ = false;
    bool use_state_covariance_for_lidar_points_ = true;
    int ieskf_max_iterations_ = 3;
    double gravity_ = 9.81;
    double acc_norm_ = 1.0;
    float range_noise_ = 0.1;
    float degree_noise_ = 0.04;
    double imu_acc_meas_noise_ = 0.0;
    double imu_gyr_meas_noise_ = 0.0;
    double kin_meas_noise_ = 0.0;
    CauchyKernel p2plane_kernel_{1.0};
    CauchyKernel ndt_kernel_{1.0};
    double last_state_predict_time_ = 0.0;
    double last_state_update_time_ = 0.0;
    bool init_flag_ = true;

    // LO-mode predict input estimated from the last two corrected states.
    Vec3D lo_predict_world_vel_ = Vec3D::Zero();
    Vec3D lo_predict_gyr_ = Vec3D::Zero();
    Mat3D lo_last_corrected_rot_ = Mat3D::Identity();
    Vec3D lo_last_corrected_pos_ = Vec3D::Zero();
    double lo_last_corrected_time_ = 0.0;
    bool lo_has_corrected_state_ = false;

    // Extrinsics and downsampling
    Mat3D ext_rot_ = Mat3D::Identity();
    Vec3D ext_t_ = Vec3D::Zero();
};

}  // namespace legkilo

#endif  // LEG_KILO_CORE_SLAM_KILO_H_
