#include "core/slam/frontend/KILO.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <utility>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include "common/glog_utils.hpp"
#include "common/math_utils.hpp"
#include "common/timer_utils.hpp"
#include "common/voxel_grid.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/frontend/eskf.h"
#include "core/slam/frontend/gaussian_voxel_map.h"
#include "preprocess/state_initial.hpp"

namespace legkilo {

namespace {
inline bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); }

inline double robustVarianceScale(double weight) {
    static constexpr double kMinWeight = 1e-6;
    return 1.0 / std::max(weight, kMinWeight);
}

constexpr GaussianVoxelMap::ResidualNoise kFirstStepLidarNoise{1.0, 1e-2, 10.0, 1e-2};
constexpr GaussianVoxelMap::ResidualNoise kSecondStepLidarNoise{1.0, 1e-3, 10.0, 1e-2};
}  // namespace

std::ostream& operator<<(std::ostream& os, const ProcessResult& result) {
    os << "Lidar ProcessResult: {"
       << " valid: " << result.valid << ", total_pts_size: " << result.total_pts_size
       << ", down_pts_size: " << result.down_pts_size << ", success_pts_size: " << result.success_pts_size
       << ", p2p_count: " << result.p2p_count << ", ndt_count: " << result.ndt_count << "}";
    return os;
}

KILO::KILO(const std::string& config_file) { this->initializeFromYaml(config_file); }

KILO::~KILO() = default;

Vec3D KILO::getPosImu() const { return eskf_->getPos(); }

Mat3D KILO::getRotImu() const { return eskf_->getRot(); }

Vec3D KILO::getPosLidar() const { return eskf_->getPos() + eskf_->getRot() * ext_t_; }

Mat3D KILO::getRotLidar() const { return eskf_->getRot() * ext_rot_; }

void KILO::initializeFromYaml(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    // Mode
    sensor_type_ = common::parseSensorType(yaml_helper.get<std::string>("sensor_type"));
    LOG(INFO) << "SensorType = " << common::toString(sensor_type_);

    // ESKF
    EskfProcessNoise eskf_noise;
    const double vel_process_cov = yaml_helper.get<double>("vel_process_cov", 20.0);
    const double imu_acc_process_cov = yaml_helper.get<double>("imu_acc_process_cov", 500.0);
    const double imu_gyr_process_cov = yaml_helper.get<double>("imu_gyr_process_cov", 1000.0);
    eskf_noise.vel_process_cov =
        common::usesLidarOnly(sensor_type_) ? yaml_helper.get<double>("lo_vel_process_cov") : vel_process_cov;
    eskf_noise.imu_acc_process_cov = imu_acc_process_cov;
    eskf_noise.imu_gyr_process_cov =
        common::usesLidarOnly(sensor_type_) ? yaml_helper.get<double>("lo_imu_gyr_process_cov") : imu_gyr_process_cov;
    eskf_noise.acc_bias_process_cov = yaml_helper.get<double>("acc_bias_process_cov", 0.001);
    eskf_noise.gyr_bias_process_cov = yaml_helper.get<double>("gyr_bias_process_cov", 0.001);
    imu_acc_meas_noise_ = yaml_helper.get<double>("imu_acc_meas_noise");
    imu_gyr_meas_noise_ = yaml_helper.get<double>("imu_gyr_meas_noise");
    if (common::usesKinematics(sensor_type_)) kin_meas_noise_ = yaml_helper.get<double>("kin_meas_noise");
    p2plane_kernel_ = CauchyKernel(yaml_helper.get<double>("p2plane_kernel_threshold", 5.0));
    ndt_kernel_ = CauchyKernel(yaml_helper.get<double>("ndt_kernel_threshold", 5.0));

    two_step_lidar_eskf_ = yaml_helper.get<bool>("two_step_lidar_eskf", true);
    p2p_enable_ = yaml_helper.get<bool>("p2plane_enable", true);
    ndt_enable_ = yaml_helper.get<bool>("ndt_enable", false);
    if (!p2p_enable_ && !ndt_enable_) { p2p_enable_ = true; }
    use_state_covariance_for_lidar_points_ = yaml_helper.get<bool>("use_state_covariance_for_lidar_points", true);
    ieskf_max_iterations_ = yaml_helper.get<int>("ieskf_max_iterations", 3);
    eskf_ = std::make_unique<ESKF>(eskf_noise);
    eskf_->setPredictModel(common::usesLidarOnly(sensor_type_) ? PredictModelType::ConstantVelocity
                                                               : PredictModelType::Inertial);

    gravity_ = yaml_helper.get<double>("gravity", 9.81);
    StateInitial::Config init_cfg;
    init_cfg.gravity = gravity_;
    int init_type_val = yaml_helper.get<int>("init_type", 1);
    init_cfg.init_type = (init_type_val == 2) ? InitType::GravityAlignment : InitType::Identity;
    if (common::usesLidarOnly(sensor_type_) && init_cfg.init_type == InitType::GravityAlignment) {
        LOG(WARNING) << "GravityAlignment is ignored in LO mode, fallback to Identity initialization";
        init_cfg.init_type = InitType::Identity;
    }
    switch (sensor_type_) {
        case common::SensorType::LO: state_initial_ = std::make_unique<StateInitialByLidar>(init_cfg); break;
        case common::SensorType::LIO: state_initial_ = std::make_unique<StateInitialByImu>(init_cfg); break;
        case common::SensorType::KILO: state_initial_ = std::make_unique<StateInitialByKinImu>(init_cfg); break;
    }

    // Voxel map
    GaussianVoxelMap::Config voxel_map_config;
    VoxelMapUtils::range_noise = yaml_helper.get<float>("dept_err");
    VoxelMapUtils::degree_noise = yaml_helper.get<float>("beam_err");
    voxel_map_config.voxel_size = yaml_helper.get<double>("voxel_size");
    voxel_map_config.capacity = yaml_helper.get<size_t>("capacity", 1000000u);
    voxel_map_config.nearby_type = static_cast<NearByType>(yaml_helper.get<int>("nearby_type", 0));
    voxel_map_config.voxel_max_num = yaml_helper.get<size_t>("voxel_max_num", 50);
    voxel_map_config.planar_ratio = yaml_helper.get<float>("planar_ratio", 0.1f);
    voxel_map_config.planar_thickness = yaml_helper.get<float>("planar_thickness", 0.05f);
    voxel_map_config.ndt_enable = ndt_enable_;
    voxel_map_config.p2p_enable = p2p_enable_;
    voxel_map_config.ndt_min_points = yaml_helper.get<size_t>("ndt_min_points", 5u);
    voxel_map_config.ndt_jitter = yaml_helper.get<double>("ndt_jitter", 1e-6);
    voxel_map_config.ndt_eigenvalue_regularization = yaml_helper.get<bool>("ndt_eigenvalue_regularization", true);
    voxel_map_config.ndt_min_eigenvalue = yaml_helper.get<double>("ndt_min_eigenvalue", 1e-6);
    voxel_map_config.ndt_max_condition = yaml_helper.get<double>("ndt_max_condition", 1000.0);
    gaussian_voxel_map_ = std::make_unique<GaussianVoxelMap>(voxel_map_config);

    // Extrinsic
    std::vector<double> ext_t = yaml_helper.get<std::vector<double>>("extrinsic_T");
    std::vector<double> ext_R = yaml_helper.get<std::vector<double>>("extrinsic_R");
    ext_rot_ << MAT_FROM_ARRAY(ext_R);
    ext_t_ << VEC_FROM_ARRAY(ext_t);

    // Downsample
    float voxel_grid_resolution = yaml_helper.get<float>("voxel_grid_resolution");
    int vg_mode = yaml_helper.get<int>("voxel_grid_mode", 1);
    size_t vg_target = yaml_helper.get<size_t>("voxel_grid_target_num", 2000u);
    float vg_overshoot = yaml_helper.get<float>("voxel_grid_overshoot", 1.5f);
    voxel_grid_ = std::make_unique<VoxelGrid>(voxel_grid_resolution, static_cast<SamplingMode>(vg_mode), vg_target,
                                              vg_overshoot, 42);
}

GaussCloudPtr KILO::initializeMap(const CloudPtr& cloud_lidar) {
    const size_t N = cloud_lidar->size();
    GaussCloudPtr gauss_cloud_ptr = std::make_shared<GaussCloud>();
    gauss_cloud_ptr->resize(N);
    const Mat3D rot = eskf_->getRot();
    const Vec3D pos = eskf_->getPos();
    const Mat3D rot_cov = eskf_->getRotCov();
    const Mat3D pos_cov = eskf_->getPosCov();
    for (size_t i = 0; i < N; ++i) {
        const Vec3D pt_lidar = cloud_lidar->points[i].getVector3fMap().cast<double>();
        const Vec3D pt_body = ext_rot_ * pt_lidar + ext_t_;
        const Vec3D pt_world = rot * pt_body + pos;
        Mat3D cov_lidar;
        VoxelMapUtils::calculatePointMeasureCov(pt_lidar, cov_lidar);

        gauss_cloud_ptr->at(i).pt = pt_world;
        gauss_cloud_ptr->at(i).cov =
            computeWorldPointCov(pt_body, ext_rot_ * cov_lidar * ext_rot_.transpose(), rot, rot_cov, pos_cov);
    }
    gaussian_voxel_map_->insertPoints(*gauss_cloud_ptr);
    return gauss_cloud_ptr;
}

Mat3D KILO::computeWorldPointCov(const Vec3D& point_body, const Mat3D& cov_body, const Mat3D& rot, const Mat3D& rot_cov,
                                 const Mat3D& pos_cov) const {
    Mat3D cov_world = rot * cov_body * rot.transpose();
    if (use_state_covariance_for_lidar_points_) {
        const Mat3D rot_crossmat = rot * SKEW_SYM_MATRIX(point_body);
        cov_world.noalias() += rot_crossmat * rot_cov * rot_crossmat.transpose() + pos_cov;
    }
    return cov_world;
}

void KILO::backpropagate(const GaussCloud& cloud_world, GaussCloud& cloud_body) const {
    CHECK_EQ(cloud_world.size(), cloud_body.size());

    const Mat3D rot = eskf_->getRot();
    const Mat3D rotT = rot.transpose();
    const Vec3D pos = eskf_->getPos();

    for (size_t i = 0; i < cloud_body.size(); ++i) {
        const GaussPoint& gs_world = cloud_world[i];
        GaussPoint& gs_body = cloud_body[i];

        gs_body.pt.noalias() = rotT * (gs_world.pt - pos);
        VoxelMapUtils::calculatePointMeasureCov(gs_body.pt, gs_body.cov);
    }
}

void KILO::resetLoPredictInput(double current_time) {
    lo_predict_world_vel_.setZero();
    lo_predict_gyr_.setZero();
    lo_last_corrected_rot_ = eskf_->getRot();
    lo_last_corrected_pos_ = eskf_->getPos();
    lo_last_corrected_time_ = current_time;
    lo_has_corrected_state_ = true;
}

void KILO::updateLoPredictInput(double current_time) {
    if (!common::usesLidarOnly(sensor_type_)) return;

    const Mat3D rot_cur = eskf_->getRot();
    const Vec3D pos_cur = eskf_->getPos();
    if (lo_has_corrected_state_) {
        const double dt = current_time - lo_last_corrected_time_;
        if (dt > 1e-6) {
            const Eigen::Matrix3d diff_rot = lo_last_corrected_rot_.transpose() * rot_cur;
            lo_predict_gyr_ = Log(diff_rot) / dt;
            lo_predict_world_vel_ = (pos_cur - lo_last_corrected_pos_) / dt;
        }
    }

    lo_last_corrected_rot_ = rot_cur;
    lo_last_corrected_pos_ = pos_cur;
    lo_last_corrected_time_ = current_time;
    lo_has_corrected_state_ = true;
}

bool KILO::eskfPredict(double current_time) {
    ESKF::PredictInput predict_input;
    if (common::usesLidarOnly(sensor_type_)) {
        predict_input.imu_gyr = lo_predict_gyr_;
        predict_input.world_vel = lo_predict_world_vel_;
    } else {
        predict_input.imu_acc = eskf_->state().imu_a_;
        predict_input.imu_gyr = eskf_->state().imu_w_;
    }

    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, predict_input, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, predict_input, true, false);
    last_state_predict_time_ = current_time;
    return true;
}

std::pair<size_t, size_t> KILO::predictUpdatePoint(const double current_time, const size_t idx_i, const size_t idx_j,
                                                   const CloudPtr& cloud_lidar_pcl, GaussCloud& cloud_body,
                                                   GaussCloud& cloud_world, LidarMatchTypes* match_types) {
    // 1) Predict state
    this->eskfPredict(current_time);
    const Mat3D rot_predict = eskf_->getRot();
    const Vec3D pos_predict = eskf_->getPos();
    const Mat3D rot_cov_predict = eskf_->getRotCov();
    const Mat3D pos_cov_predict = eskf_->getPosCov();

    // 2) Residuals
    size_t points_size = idx_j - idx_i;
    std::vector<KNearestRes<1>> p2p_results;
    std::vector<KNearestRes<3>> ndt_results;
    p2p_results.reserve(points_size);
    if (ndt_enable_) ndt_results.reserve(points_size);

    for (size_t i = 0; i < points_size; ++i) {
        const size_t cur_idx = i + idx_i;
        const Vec3D pt_lidar = cloud_lidar_pcl->points[cur_idx].getArray3fMap().cast<double>();

        // 2.1 point var(body and world) compute
        GaussPoint& gs_body = cloud_body[cur_idx];
        GaussPoint& gs_world = cloud_world[cur_idx];

        Mat3D pt_lidar_cov;
        VoxelMapUtils::calculatePointMeasureCov(pt_lidar, pt_lidar_cov);
        gs_body.pt = ext_rot_ * pt_lidar + ext_t_;
        gs_body.cov = ext_rot_ * pt_lidar_cov * ext_rot_.transpose();

        gs_world.pt = rot_predict * gs_body.pt + pos_predict;
        gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, rot_predict, rot_cov_predict, pos_cov_predict);

        // 2.2 residual
        KNearestInput knn_input(&gs_body.pt, &gs_body.cov, &gs_world.pt, &gs_world.cov, &rot_predict);
        KNearestRes<1> p2p_result;
        if (p2p_enable_) { gaussian_voxel_map_->buildPoint2PlaneResidual(knn_input, p2p_result, kFirstStepLidarNoise); }

        // 2.3 push if pvalid (or try NDT if P2P disabled/invalid)
        if (p2p_result.valid) {
            p2p_results.push_back(p2p_result);
            if (match_types && cur_idx < match_types->size()) { (*match_types)[cur_idx] = LidarMatchType::Point2Plane; }
        } else if (ndt_enable_) {
            KNearestRes<3> ndt_result;
            gaussian_voxel_map_->buildNdtResidual(knn_input, ndt_result, kFirstStepLidarNoise);
            if (ndt_result.valid) {
                ndt_results.push_back(ndt_result);
                if (match_types && cur_idx < match_types->size()) { (*match_types)[cur_idx] = LidarMatchType::Ndt; }
            }
        }
    }

    const size_t p2p_count = p2p_results.size();
    const size_t ndt_count = ndt_results.size();
    bool eskf_update_enable = (p2p_count + ndt_count) > 0;

    if (eskf_update_enable) {
        ObsShared obs_shared;
        size_t h_total_rows = p2p_results.size() + 3 * ndt_results.size();
        obs_shared.pt_h.resize(h_total_rows, 6);
        obs_shared.pt_R.resize(h_total_rows);
        obs_shared.pt_z.resize(h_total_rows);

        for (size_t k = 0; k < p2p_results.size(); ++k) {
            obs_shared.pt_h.row(k) = p2p_results[k].J;
            obs_shared.pt_z(k) = p2p_results[k].r(0, 0);
            const double p2plane_mahalanobis2 = p2p_results[k].r.squaredNorm();
            const double p2plane_weight = p2plane_kernel_.weight(p2plane_mahalanobis2);
            obs_shared.pt_R(k) = robustVarianceScale(p2plane_weight);
        }

        const size_t p2p_end_row = p2p_results.size();
        for (size_t m = 0; m < ndt_results.size(); ++m) {
            obs_shared.pt_h.block<3, 6>(p2p_end_row + 3 * m, 0) = ndt_results[m].J;
            obs_shared.pt_z.segment<3>(p2p_end_row + 3 * m) = ndt_results[m].r;
            const double ndt_weight = ndt_kernel_.weight(ndt_results[m].r.squaredNorm());
            obs_shared.pt_R.segment<3>(p2p_end_row + 3 * m) = robustVarianceScale(ndt_weight) * Eigen::Vector3d::Ones();
        }

        eskf_->updateByPoints(obs_shared);
        last_state_update_time_ = current_time;

        // recompute world with updated state and update var
        const Mat3D rot_update = eskf_->getRot();
        const Vec3D pos_update = eskf_->getPos();
        const Mat3D rot_cov_update = eskf_->getRotCov();
        const Mat3D pos_cov_update = eskf_->getPosCov();
        for (size_t i = 0; i < points_size; ++i) {
            const size_t cur_idx = i + idx_i;
            const GaussPoint& gs_body = cloud_body[cur_idx];
            GaussPoint& gs_world = cloud_world[cur_idx];

            gs_world.pt = rot_update * gs_body.pt + pos_update;
            gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, rot_update, rot_cov_update, pos_cov_update);
        }
    }
    return {p2p_count, ndt_count};
}

std::pair<size_t, size_t> KILO::predictUpdateCloud(const GaussCloud& cloud_body, GaussCloud& cloud_world,
                                                   LidarMatchTypes* match_types) {
    CHECK_EQ(cloud_body.size(), cloud_world.size());

    const State state_before_iter = eskf_->getState();
    size_t iteration = 0;
    size_t p2p_valid_num = 0;
    size_t ndt_valid_num = 0;
    bool converge = false;

    for (; iteration < ieskf_max_iterations_; ++iteration) {
        p2p_valid_num = 0;
        ndt_valid_num = 0;
        const Mat3D R_cur = eskf_->getRot();
        const Vec3D pos_cur = eskf_->getPos();
        const Mat3D rot_cov_cur = eskf_->getRotCov();
        const Mat3D pos_cov_cur = eskf_->getPosCov();
        const size_t N = cloud_body.size();

        for (size_t j = 0; j < N; ++j) {
            const GaussPoint& gs_body = cloud_body[j];
            GaussPoint& gs_world = cloud_world[j];
            gs_world.pt = R_cur * gs_body.pt + pos_cur;
            gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, R_cur, rot_cov_cur, pos_cov_cur);
        }

        std::vector<KNearestRes<1>> p2p_results(N);
        std::vector<KNearestRes<3>> ndt_results;
        if (ndt_enable_) ndt_results.resize(N);

        const auto process_func = [&](size_t i) {
            KNearestInput knn_input(&cloud_body[i].pt, &cloud_body[i].cov, &cloud_world[i].pt, &cloud_world[i].cov,
                                    &R_cur);
            if (p2p_enable_) {
                gaussian_voxel_map_->buildPoint2PlaneResidual(knn_input, p2p_results[i], kSecondStepLidarNoise);
            }
            if ((!p2p_enable_ || p2p_results[i].valid == false) && ndt_enable_) {
                gaussian_voxel_map_->buildNdtResidual(knn_input, ndt_results[i], kSecondStepLidarNoise);
            }
        };

        static constexpr size_t kGain = 128;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, N, kGain), [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) process_func(i);
        });

        p2p_valid_num =
            std::count_if(p2p_results.begin(), p2p_results.end(), [](const KNearestRes<1>& res) { return res.valid; });
        ndt_valid_num = ndt_enable_ ? std::count_if(ndt_results.begin(), ndt_results.end(),
                                                    [](const KNearestRes<3>& res) { return res.valid; })
                                    : 0;
        if (match_types && match_types->size() == N) {
            std::fill(match_types->begin(), match_types->end(), LidarMatchType::Unused);
            for (size_t i = 0; i < N; ++i) {
                if (p2p_results[i].valid) {
                    (*match_types)[i] = LidarMatchType::Point2Plane;
                } else if (ndt_enable_ && ndt_results[i].valid) {
                    (*match_types)[i] = LidarMatchType::Ndt;
                }
            }
        }
        bool eskf_update_enable = (p2p_valid_num + ndt_valid_num) > 0;
        if (eskf_update_enable) {
            ObsShared obs_shared;
            size_t h_total_rows = p2p_valid_num + 3 * ndt_valid_num;
            obs_shared.pt_h.resize(h_total_rows, 6);
            obs_shared.pt_R.resize(h_total_rows);
            obs_shared.pt_z.resize(h_total_rows);

            size_t row = 0;
            for (size_t k = 0; k < p2p_results.size(); ++k) {
                if (!p2p_results[k].valid) continue;
                obs_shared.pt_h.row(row) = p2p_results[k].J;
                obs_shared.pt_z(row) = p2p_results[k].r(0, 0);
                const double p2plane_mahalanobis2 = p2p_results[k].r.squaredNorm();
                const double p2plane_weight = p2plane_kernel_.weight(p2plane_mahalanobis2);
                obs_shared.pt_R(row) = robustVarianceScale(p2plane_weight);
                ++row;
            }
            for (size_t m = 0; m < ndt_results.size(); ++m) {
                if (!ndt_results[m].valid) continue;
                obs_shared.pt_h.block<3, 6>(row, 0) = ndt_results[m].J;
                obs_shared.pt_z.segment<3>(row) = ndt_results[m].r;
                const double ndt_weight = ndt_kernel_.weight(ndt_results[m].r.squaredNorm());
                obs_shared.pt_R.segment<3>(row) = robustVarianceScale(ndt_weight) * Eigen::Vector3d::Ones();
                row += 3;
            }

            converge = eskf_->updateByCloud(obs_shared, state_before_iter, ieskf_max_iterations_, iteration);
            if (converge) break;
        }
    }
    // LOG(INFO) << "IESKF iteration times: " << iteration + 1 << ", converge: " << converge;

    // After iteration, ensure cloud_world reflects final state for downstream map update
    const Mat3D R_final = eskf_->getRot();
    const Vec3D pos_final = eskf_->getPos();
    const Mat3D rot_cov_final = eskf_->getRotCov();
    const Mat3D pos_cov_final = eskf_->getPosCov();
    for (size_t j = 0; j < cloud_body.size(); ++j) {
        const GaussPoint& gs_body = cloud_body[j];
        GaussPoint& gs_world = cloud_world[j];
        gs_world.pt = R_final * gs_body.pt + pos_final;
        gs_world.cov = computeWorldPointCov(gs_body.pt, gs_body.cov, R_final, rot_cov_final, pos_cov_final);
    }
    return {p2p_valid_num, ndt_valid_num};
}

bool KILO::predictUpdateImu(const ros_compat::ImuMsgPtr& imu) {
    // 1) Predict state
    double current_time = ros_compat::toSec(imu->header.stamp);
    this->eskfPredict(current_time);

    // 2) IMU update
    ObsShared obs_shared;
    obs_shared.ki_R.resize(6);
    obs_shared.ki_z.resize(6);
    Vec3D imu_acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    Vec3D imu_gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R << imu_acc_meas_noise_, imu_acc_meas_noise_, imu_acc_meas_noise_, imu_gyr_meas_noise_,
        imu_gyr_meas_noise_, imu_gyr_meas_noise_;

    eskf_->updateByImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

bool KILO::predictUpdateKinImu(const common::KinImuMeas& kin_imu) {
    double current_time = kin_imu.time_stamp_;
    // 1) Predict state
    this->eskfPredict(current_time);

    // 2) Kin-IMU update
    int contact_nums = 0;
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) { contact_nums++; }
    }

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6 + 3 * contact_nums);
    obs_shared.ki_z.resize(6 + 3 * contact_nums);
    obs_shared.ki_h.resize(6 + 3 * contact_nums, DIM_STATE);
    obs_shared.ki_h.setZero();

    obs_shared.ki_h.block<6, 6>(0, 9) = Eigen::Matrix<double, 6, 6>::Identity();
    obs_shared.ki_h.block<6, 6>(0, 18) = Eigen::Matrix<double, 6, 6>::Identity();
    Vec3D imu_acc(kin_imu.acc_[0], kin_imu.acc_[1], kin_imu.acc_[2]);
    Vec3D imu_gyr(kin_imu.gyr_[0], kin_imu.gyr_[1], kin_imu.gyr_[2]);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R.block<6, 1>(0, 0) << imu_acc_meas_noise_, imu_acc_meas_noise_, imu_acc_meas_noise_,
        imu_gyr_meas_noise_, imu_gyr_meas_noise_, imu_gyr_meas_noise_;

    int idx = 0;
    Mat3D w_skew = SKEW_SYM_MATRIX(eskf_->state().imu_w_);
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) {
            Vec3D foot_pos(kin_imu.foot_pos_[i][0], kin_imu.foot_pos_[i][1], kin_imu.foot_pos_[i][2]);
            Vec3D foot_vel(kin_imu.foot_vel_[i][0], kin_imu.foot_vel_[i][1], kin_imu.foot_vel_[i][2]);

            Vec3D w_skew_pos_vel = w_skew * foot_pos + foot_vel;

            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 0) = -eskf_->getRot() * SKEW_SYM_MATRIX(w_skew_pos_vel);
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 6) = Mat3D::Identity();
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 21) = -eskf_->getRot() * SKEW_SYM_MATRIX(foot_pos);

            obs_shared.ki_z.block<3, 1>(6 + 3 * idx, 0) = -eskf_->getVel() - eskf_->getRot() * w_skew_pos_vel;

            obs_shared.ki_R.block<3, 1>(6 + 3 * idx, 0) << kin_meas_noise_, kin_meas_noise_, kin_meas_noise_;
            idx++;
        }
    }

    eskf_->updateByKinImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

ProcessResult KILO::process(common::MeasGroup& measure) {
    ProcessResult result;

    CloudPtr cloud_raw = measure.lidar_scan_.cloud_;
    CloudPtr cloud_down_lidar(new PointCloudType());
    CloudPtr cloud_down_body(new PointCloudType());
    CloudPtr cloud_down_world(new PointCloudType());
    GaussCloudPtr gauss_cloud_body(new GaussCloud());
    GaussCloudPtr gauss_cloud_world(new GaussCloud());

    auto& imus = measure.imus_;
    auto& kin_imus = measure.kin_imus_;
    double begin_time = measure.lidar_scan_.lidar_begin_time_;
    double end_time = measure.lidar_scan_.lidar_end_time_;

    if (cloud_raw->points.empty() || (common::usesImu(sensor_type_) && imus.empty()) ||
        (common::usesKinematics(sensor_type_) && kin_imus.empty())) {
        LOG(WARNING) << "Data packet is not ready";
        return result;
    }

    // Downsampling
    Timer::measure("Downsampling", [&, this]() { voxel_grid_->filter(cloud_raw, cloud_down_lidar); });

    // First-frame initialization
    if (init_flag_) {
        state_initial_->ingest(measure);

        eskf_->state().grav_ = state_initial_->gravityVec();
        eskf_->state().bw_ = state_initial_->gyroBias();
        eskf_->state().rot_ = state_initial_->initRotation();
        eskf_->cov() = 0.000001 * StateCov::Identity();

        gauss_cloud_world = this->initializeMap(cloud_down_lidar);
        pcl_utils::GaussCloudToPclCloud(*gauss_cloud_world, cloud_down_world);
        pcl::transformPointCloud(*cloud_down_lidar, *cloud_down_body,
                                 makeIsometry3d(ext_rot_, ext_t_).matrix().cast<float>());

        auto gravity_vec = eskf_->state().grav_;
        auto bw = eskf_->state().bw_;
        LOG(INFO) << "Gravity is initialized to " << std::fixed << std::setprecision(3) << gravity_vec.transpose();
        LOG(INFO) << "IMU bw is initialized to " << bw.transpose();

        acc_norm_ = state_initial_->accNorm();
        last_state_predict_time_ = end_time;
        last_state_update_time_ = end_time;
        if (common::usesLidarOnly(sensor_type_)) { resetLoPredictInput(end_time); }
        init_flag_ = false;

        result.valid = true;
        result.total_pts_size = cloud_raw->size();
        result.down_pts_size = cloud_down_lidar->size();
        result.success_pts_size = cloud_down_lidar->size();
        result.cloud_world = cloud_down_world;
        result.cloud_lidar = cloud_down_lidar;
        result.cloud_body = cloud_down_body;
        result.match_types = std::make_shared<LidarMatchTypes>(cloud_down_lidar->size(), LidarMatchType::Unused);
        return result;
    }

    // Prepare per-point var storage
    size_t p2p_total = 0;
    size_t ndt_total = 0;
    const size_t N = cloud_down_lidar->size();
    gauss_cloud_body->resize(N);
    gauss_cloud_world->resize(N);
    auto match_types = std::make_shared<LidarMatchTypes>(N, LidarMatchType::Unused);
    bool frame_had_lidar_update = false;

    // First-step ESKF (per point/imu/kin.)
    Timer::measure("State predict/update - 1st", [&, this]() {
        // Sort by per-point time offset (curvature field)
        auto& pts = cloud_down_lidar->points;
        std::sort(pts.begin(), pts.end(), time_list);

        // Predict/update cycle across time-buckets of equal curvature
        const size_t pts_size = pts.size();
        size_t idx_i = 0;
        while (idx_i < pts_size) {
            double cur_point_time = begin_time + pts[idx_i].curvature;
            size_t idx_j = idx_i + 1;
            while (idx_j < pts_size && pts[idx_i].curvature == pts[idx_j].curvature) { idx_j++; }

            if (common::usesImu(sensor_type_)) {
                while (!imus.empty() && ros_compat::toSec(imus.front()->header.stamp) < cur_point_time) {
                    this->predictUpdateImu(imus.front());
                    imus.pop_front();
                }
            } else if (common::usesKinematics(sensor_type_)) {
                while (!kin_imus.empty() && kin_imus.front().time_stamp_ < cur_point_time) {
                    this->predictUpdateKinImu(kin_imus.front());
                    kin_imus.pop_front();
                }
            }

            auto counts = this->predictUpdatePoint(cur_point_time, idx_i, idx_j, cloud_down_lidar, *gauss_cloud_body,
                                                   *gauss_cloud_world, match_types.get());
            p2p_total += counts.first;
            ndt_total += counts.second;
            frame_had_lidar_update = frame_had_lidar_update || ((counts.first + counts.second) > 0);
            idx_i = idx_j;
        }
    });

    // Second-step ESKF (full pointcloud)
    if (two_step_lidar_eskf_) {
        p2p_total = 0;
        ndt_total = 0;
        std::fill(match_types->begin(), match_types->end(), LidarMatchType::Unused);

        Timer::measure("Backpropagate", [&, this]() { this->backpropagate(*gauss_cloud_world, *gauss_cloud_body); });

        Timer::measure("State predict/update - 2nd", [&, this]() {
            auto counts = this->predictUpdateCloud(*gauss_cloud_body, *gauss_cloud_world, match_types.get());
            p2p_total = counts.first;
            ndt_total = counts.second;
            frame_had_lidar_update = frame_had_lidar_update || ((counts.first + counts.second) > 0);
        });
    }

    Timer::measure("Voxel map update", [&, this]() { gaussian_voxel_map_->insertPoints(*gauss_cloud_world); });

    if (common::usesLidarOnly(sensor_type_) && frame_had_lidar_update) { updateLoPredictInput(end_time); }

    // Fill in result
    pcl_utils::GaussCloudToPclCloud(*gauss_cloud_world, cloud_down_world);
    pcl::transformPointCloud(*cloud_down_world, *cloud_down_body,
                             makeIsometry3d(eskf_->getRot(), eskf_->getPos()).inverse().matrix().cast<float>());
    pcl::transformPointCloud(*cloud_down_body, *cloud_down_lidar,
                             makeIsometry3d(ext_rot_, ext_t_).inverse().matrix().cast<float>());
    result.valid = true;
    result.total_pts_size = cloud_raw->size();
    result.down_pts_size = cloud_down_lidar->size();
    result.p2p_count = p2p_total;
    result.ndt_count = ndt_total;
    result.success_pts_size = p2p_total + ndt_total;
    result.cloud_world = cloud_down_world;
    result.cloud_lidar = cloud_down_lidar;
    result.cloud_body = cloud_down_body;
    result.match_types = match_types;
    return result;
}

}  // namespace legkilo
