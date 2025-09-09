#include "core/slam/KILO.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <utility>
#include "common/math_utils.hpp"

#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/eskf.h"
#include "core/slam/voxel_map.h"
#include "preprocess/state_initial.hpp"

namespace legkilo {

namespace {
inline bool time_list(PointType& x, PointType& y) { return (x.curvature < y.curvature); }
}  // namespace

KILO::KILO(const std::string& config_file) { initializeFromYaml(config_file); }
KILO::~KILO() = default;

void KILO::initializeFromYaml(const std::string& config_file) {
    YamlHelper yaml_helper(config_file);

    // Mode
    imu_mode_only_ = yaml_helper.get<bool>("only_imu_use", true);

    // ESKF
    ESKF::Config eskf_config;
    eskf_config.vel_process_cov = yaml_helper.get<double>("vel_process_cov");
    eskf_config.imu_acc_process_cov = yaml_helper.get<double>("imu_acc_process_cov");
    eskf_config.imu_gyr_process_cov = yaml_helper.get<double>("imu_gyr_process_cov");
    eskf_config.acc_bias_process_cov = yaml_helper.get<double>("acc_bias_process_cov");
    eskf_config.gyr_bias_process_cov = yaml_helper.get<double>("gyr_bias_process_cov");
    eskf_config.kin_bias_process_cov = yaml_helper.get<double>("kin_bias_process_cov");
    eskf_config.contact_process_cov = yaml_helper.get<double>("contact_process_cov");
    eskf_config.imu_acc_meas_noise = yaml_helper.get<double>("imu_acc_meas_noise");
    eskf_config.imu_acc_z_meas_noise = yaml_helper.get<double>("imu_acc_z_meas_noise");
    eskf_config.imu_gyr_meas_noise = yaml_helper.get<double>("imu_gyr_meas_noise");
    eskf_config.kin_meas_noise = yaml_helper.get<double>("kin_meas_noise");
    eskf_config.chd_meas_noise = yaml_helper.get<double>("chd_meas_noise");
    eskf_config.contact_meas_noise = yaml_helper.get<double>("contact_meas_noise");
    eskf_config.lidar_point_meas_ratio = yaml_helper.get<double>("lidar_point_meas_ratio");
    eskf_ = std::make_unique<ESKF>(eskf_config);

    // Init method
    gravity_ = yaml_helper.get<double>("gravity", 9.81);
    if (imu_mode_only_) {
        state_initial_ = std::make_unique<StateInitialByImu>(gravity_);
    } else {
        state_initial_ = std::make_unique<StateInitialByKinImu>(gravity_);
    }

    // Voxel map
    VoxelMapConfig voxel_map_config;
    voxel_map_config.is_pub_plane_map_ = yaml_helper.get<bool>("pub_plane_en");
    voxel_map_config.max_layer_ = yaml_helper.get<int>("max_layer");
    voxel_map_config.max_voxel_size_ = yaml_helper.get<double>("voxel_size");
    voxel_map_config.planner_threshold_ = yaml_helper.get<double>("min_eigen_value");
    voxel_map_config.sigma_num_ = yaml_helper.get<double>("sigma_num");
    voxel_map_config.beam_err_ = yaml_helper.get<double>("beam_err");
    voxel_map_config.dept_err_ = yaml_helper.get<double>("dept_err");
    voxel_map_config.layer_init_num_ = yaml_helper.get<std::vector<int>>("layer_init_num");
    voxel_map_config.max_points_num_ = yaml_helper.get<int>("max_points_num");
    voxel_map_config.map_sliding_en = yaml_helper.get<bool>("map_sliding_en");
    voxel_map_config.half_map_size = yaml_helper.get<int>("half_map_size");
    voxel_map_config.sliding_thresh = yaml_helper.get<double>("sliding_thresh");
    map_manager_ = std::make_unique<VoxelMapManager>(voxel_map_config);

    // Extrinsic
    std::vector<double> ext_t = yaml_helper.get<std::vector<double>>("extrinsic_T");
    std::vector<double> ext_R = yaml_helper.get<std::vector<double>>("extrinsic_R");
    ext_rot_ << MAT_FROM_ARRAY(ext_R);
    ext_t_ << VEC_FROM_ARRAY(ext_t);
    map_manager_->extT_ = ext_t_;
    map_manager_->extR_ = ext_rot_;

    // Downsample
    float voxel_grid_resolution = yaml_helper.get<float>("voxel_grid_resolution");
    voxel_grid_.setLeafSize(voxel_grid_resolution, voxel_grid_resolution, voxel_grid_resolution);
}

const Vec3D& KILO::position() const { return eskf_->state().pos_; }
const Mat3D& KILO::rotation() const { return eskf_->state().rot_; }
Mat3D KILO::getRotMatrix() const { return eskf_->getRot(); }

void KILO::cloudLidarToWorld(const CloudPtr& cloud_lidar, CloudPtr& cloud_world) {
    cloud_world->clear();
    cloud_world->points.resize(cloud_lidar->points.size());
    for (size_t i = 0; i < cloud_lidar->points.size(); ++i) {
        pointLidarToWorld(cloud_lidar->points[i], cloud_world->points[i]);
    }
}

inline void KILO::pointLidarToWorld(const PointType& point_lidar, PointType& point_world) {
    Eigen::Vector3d pt_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
    Eigen::Vector3d pt_imu = ext_rot_ * pt_lidar + ext_t_;
    Eigen::Vector3d pt_world = eskf_->state().rot_ * pt_imu + eskf_->state().pos_;

    point_world.x = static_cast<float>(pt_world(0));
    point_world.y = static_cast<float>(pt_world(1));
    point_world.z = static_cast<float>(pt_world(2));
    point_world.intensity = point_lidar.intensity;
}

bool KILO::predictUpdatePoint(double current_time, size_t idx_i, size_t idx_j, const PointCloudType& cloud_down_body,
                              PointCloudType& cloud_down_world, size_t& success_pts_size_out) {
    // 1) Predict state
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    // 2) Residuals
    size_t points_size = idx_j - idx_i;
    std::vector<PointToPlane> ptpl_list;
    std::vector<pointWithVar> pv_list(points_size);
    ptpl_list.reserve(points_size);
    for (size_t i = 0; i < points_size; ++i) {
        PointType const& cur_pt = cloud_down_body.points[i + idx_i];

        // 2.1 point var(body and world) compute
        pointWithVar& cur_pt_var = pv_list[i];
        cur_pt_var.point_b << cur_pt.x, cur_pt.y, cur_pt.z;
        cur_pt_var.point_i = ext_rot_ * cur_pt_var.point_b + ext_t_;
        cur_pt_var.point_w = eskf_->state().rot_ * cur_pt_var.point_i + eskf_->state().pos_;
        cloud_down_world.points[idx_i + i].x = cur_pt_var.point_w(0);
        cloud_down_world.points[idx_i + i].y = cur_pt_var.point_w(1);
        cloud_down_world.points[idx_i + i].z = cur_pt_var.point_w(2);
        cloud_down_world.points[idx_i + i].intensity = 0;
        calcBodyCov(cur_pt_var.point_b, map_manager_->config_setting_.dept_err_,
                    map_manager_->config_setting_.beam_err_, cur_pt_var.body_var);
        cur_pt_var.point_crossmat << SKEW_SYM_MATRIX(cur_pt_var.point_i);
        Mat3D rot_extR = eskf_->state().rot_ * ext_rot_;
        Mat3D rot_crossmat = eskf_->state().rot_ * cur_pt_var.point_crossmat;
        cur_pt_var.var = rot_extR * cur_pt_var.body_var * rot_extR.transpose() +
                         rot_crossmat * eskf_->cov().block<3, 3>(0, 0) * rot_crossmat.transpose() +
                         eskf_->cov().block<3, 3>(3, 3);

        // 2.2 residual
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = cur_pt_var.point_w[j] / map_manager_->config_setting_.max_voxel_size_;
            if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
        }
        Eigen::Vector3i position((int)loc_xyz[0], (int)loc_xyz[1], (int)loc_xyz[2]);
        auto iter = map_manager_->voxel_map_.find(position);
        if (iter != map_manager_->voxel_map_.end()) {
            VoxelOctoTree* current_octo = iter->second;
            PointToPlane single_ptpl;
            bool is_success = false;
            double prob = 0;
            map_manager_->build_single_residual(cur_pt_var, current_octo, 0, is_success, prob, single_ptpl);
            if (!is_success) {
                Eigen::Vector3i near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x() = near_position.x() + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) {
                    near_position.x() = near_position.x() - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y() = near_position.y() + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) {
                    near_position.y() = near_position.y() - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z() = near_position.z() + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) {
                    near_position.z() = near_position.z() - 1;
                }
                auto iter_near = map_manager_->voxel_map_.find(near_position);
                if (iter_near != map_manager_->voxel_map_.end()) {
                    map_manager_->build_single_residual(cur_pt_var, iter_near->second, 0, is_success, prob,
                                                        single_ptpl);
                }
            }
            if (is_success) {
                ++success_pts_size_out;
                ptpl_list.push_back(single_ptpl);
            }
        }
    }

    // 3) KF update with points
    size_t effect_num = ptpl_list.size();
    bool eskf_update = effect_num > 0;
    if (eskf_update) {
        ObsShared obs_shared;
        obs_shared.pt_h.resize(effect_num, 6);
        obs_shared.pt_R.resize(effect_num);
        obs_shared.pt_z.resize(effect_num);
        for (size_t k = 0; k < effect_num; ++k) {
            Vec3D crossmat_rotT_u =
                ptpl_list[k].point_crossmat_ * eskf_->state().rot_.transpose() * ptpl_list[k].normal_;
            obs_shared.pt_h.row(k) << crossmat_rotT_u(0), crossmat_rotT_u(1), crossmat_rotT_u(2),
                ptpl_list[k].normal_(0), ptpl_list[k].normal_(1), ptpl_list[k].normal_(2);

            obs_shared.pt_z(k) = -ptpl_list[k].dis_to_plane_;

            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = ptpl_list[k].point_w_ - ptpl_list[k].center_;
            J_nq.block<1, 3>(0, 3) = -ptpl_list[k].normal_;
            Mat3D var;
            var = eskf_->state().rot_ * ext_rot_ * ptpl_list[k].body_cov_ * ext_rot_.transpose() *
                  eskf_->state().rot_.transpose();
            double single_l = J_nq * ptpl_list[k].plane_var_ * J_nq.transpose();
            obs_shared.pt_R(k) = eskf_->config().lidar_point_meas_ratio *
                                 (single_l + ptpl_list[k].normal_.transpose() * var * ptpl_list[k].normal_);
        }
        eskf_->updateByPoints(obs_shared);
        last_state_update_time_ = current_time;
    }

    // 4) voxel map update
    if (eskf_update) {
        for (size_t i = 0; i < points_size; ++i) {
            // recompute world with updated state and update var
            pv_list[i].point_w = eskf_->state().rot_ * pv_list[i].point_i + eskf_->state().pos_;
            cloud_down_world.points[idx_i + i].x = pv_list[i].point_w(0);
            cloud_down_world.points[idx_i + i].y = pv_list[i].point_w(1);
            cloud_down_world.points[idx_i + i].z = pv_list[i].point_w(2);
            cloud_down_world.points[idx_i + i].intensity = 255;

            Mat3D rot_extR = eskf_->state().rot_ * ext_rot_;
            Mat3D rot_crossmat = eskf_->state().rot_ * pv_list[i].point_crossmat;
            pv_list[i].var = rot_extR * pv_list[i].body_var * rot_extR.transpose() +
                             rot_crossmat * eskf_->cov().block<3, 3>(0, 0) * rot_crossmat.transpose() +
                             eskf_->cov().block<3, 3>(3, 3);
        }
    }
    map_manager_->UpdateVoxelMap(pv_list);
    return effect_num > 0;
}

bool KILO::predictUpdateImu(const sensor_msgs::ImuPtr& imu) {
    double current_time = imu->header.stamp.toSec();
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6);
    obs_shared.ki_z.resize(6);
    Vec3D imu_acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    Vec3D imu_gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    eskf_->updateByImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

bool KILO::predictUpdateKinImu(const common::KinImuMeas& kin_imu) {
    double current_time = kin_imu.time_stamp_;
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

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

    obs_shared.ki_R.block<6, 1>(0, 0) << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    int idx = 0;
    Mat3D w_skew = SKEW_SYM_MATRIX(eskf_->state().imu_w_);
    for (int i = 0; i < 4; ++i) {
        if (kin_imu.contact_[i]) {
            Vec3D foot_pos(kin_imu.foot_pos_[i][0], kin_imu.foot_pos_[i][1], kin_imu.foot_pos_[i][2]);
            Vec3D foot_vel(kin_imu.foot_vel_[i][0], kin_imu.foot_vel_[i][1], kin_imu.foot_vel_[i][2]);

            Vec3D w_skew_pos_vel = w_skew * foot_pos + foot_vel;

            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 0) = -eskf_->state().rot_ * SKEW_SYM_MATRIX(w_skew_pos_vel);
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 6) = Mat3D::Identity();
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 21) = -eskf_->state().rot_ * SKEW_SYM_MATRIX(foot_pos);

            obs_shared.ki_z.block<3, 1>(6 + 3 * idx, 0) = -eskf_->state().vel_ - eskf_->state().rot_ * w_skew_pos_vel;

            obs_shared.ki_R.block<3, 1>(6 + 3 * idx, 0) << eskf_->config().kin_meas_noise,
                eskf_->config().kin_meas_noise, eskf_->config().kin_meas_noise;
            idx++;
        }
    }

    eskf_->updateByKinImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}

bool KILO::process(common::MeasGroup measure, CloudPtr& cloud_down_body_out, CloudPtr& cloud_down_world_out,
                   size_t& success_pts_size_out) {
    success_pts_size_out = 0;

    CloudPtr cloud_raw = measure.lidar_scan_.cloud_;
    auto& imus = measure.imus_;
    auto& kin_imus = measure.kin_imus_;
    double begin_time = measure.lidar_scan_.lidar_begin_time_;
    double end_time = measure.lidar_scan_.lidar_end_time_;

    if (cloud_raw->points.empty() || (imu_mode_only_ && imus.empty()) || (!imu_mode_only_ && kin_imus.empty())) {
        LOG(WARNING) << "Data packet is not ready";
        return false;
    }

    // First-frame initialization
    if (init_flag_) {
        state_initial_->processing(measure, *eskf_);

        cloud_down_world_out.reset(new PointCloudType());
        this->cloudLidarToWorld(cloud_raw, cloud_down_world_out);
        map_manager_->feats_down_body_ = cloud_raw;
        map_manager_->feats_down_world_ = cloud_down_world_out;
        map_manager_->BuildVoxelMap(eskf_->state().rot_, eskf_->cov().block<3, 3>(0, 0),
                                    eskf_->cov().block<3, 3>(3, 3));

        auto gravity_vec = eskf_->state().grav_;
        auto bw = eskf_->state().bw_;
        LOG(INFO) << "Initialization is finished";
        LOG(INFO) << "Gravity is initialized to " << std::fixed << std::setprecision(3) << gravity_vec(0) << " "
                  << gravity_vec(1) << " " << gravity_vec(2);
        LOG(INFO) << "IMU bw is initialized to " << bw(0) << " " << bw(1) << " " << bw(2);

        init_flag_ = false;
        acc_norm_ = state_initial_->getAccNorm();
        last_state_predict_time_ = end_time;
        last_state_update_time_ = end_time;
        return true;
    }

    // Downsampling
    Timer::measure("Downsampling", [&, this]() {
        cloud_down_body_out.reset(new PointCloudType());
        voxel_grid_.setInputCloud(cloud_raw);
        voxel_grid_.filter(*cloud_down_body_out);
    });

    // Prepare output container size
    cloud_down_world_out.reset(new PointCloudType());
    cloud_down_world_out->points.resize(cloud_down_body_out->points.size());

    // State predict/update & Map update
    Timer::measure("State predict/update & Map update", [&, this]() {
        // Sort by per-point time offset (curvature field)
        auto& pts = cloud_down_body_out->points;
        std::sort(pts.begin(), pts.end(), time_list);

        // Predict/update cycle across time-buckets of equal curvature
        const size_t pts_size = pts.size();
        size_t idx_i = 0;
        while (idx_i < pts_size) {
            double cur_point_time = begin_time + pts[idx_i].curvature;
            size_t idx_j = idx_i + 1;
            while (idx_j < pts_size && pts[idx_i].curvature == pts[idx_j].curvature) { idx_j++; }

            if (imu_mode_only_) {
                while (!imus.empty() && imus.front()->header.stamp.toSec() < cur_point_time) {
                    this->predictUpdateImu(imus.front());
                    imus.pop_front();
                }
            } else {
                while (!kin_imus.empty() && kin_imus.front().time_stamp_ < cur_point_time) {
                    this->predictUpdateKinImu(kin_imus.front());
                    kin_imus.pop_front();
                }
            }

            this->predictUpdatePoint(cur_point_time, idx_i, idx_j, *cloud_down_body_out, *cloud_down_world_out,
                                     success_pts_size_out);
            idx_i = idx_j;
        }
    });

    return true;
}

}  // namespace legkilo
