/*
    Original project: FAST-LIVO2 https://github.com/hku-mars/FAST-LIVO2
    This file was modified by Ou. in 2025/02/02.
*/

/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include <math.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <fstream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include "common/eigen_types.hpp"
#include "common/math_utils.hpp"
#include "common/pcl_types.h"
#include "common/sensor_types.hpp"

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig {
    double max_voxel_size_;
    int max_layer_;
    int max_iterations_;
    std::vector<int> layer_init_num_;
    int max_points_num_;
    double planner_threshold_;
    double beam_err_;
    double dept_err_;
    double sigma_num_;
    bool is_pub_plane_map_;

    // config of local map sliding
    double sliding_thresh;
    bool map_sliding_en;
    int half_map_size;
} VoxelMapConfig;

typedef struct pointWithVar {
    Eigen::Vector3d point_b;      // point in the lidar body frame
    Eigen::Vector3d point_i;      // point in the imu body frame
    Eigen::Vector3d point_w;      // point in the world frame
    Eigen::Matrix3d var_nostate;  // the var removed the state covarience
    Eigen::Matrix3d body_var;
    Eigen::Matrix3d var;
    Eigen::Matrix3d point_crossmat;
    Eigen::Vector3d normal;
    pointWithVar() {
        var_nostate = Eigen::Matrix3d::Zero();
        var = Eigen::Matrix3d::Zero();
        body_var = Eigen::Matrix3d::Zero();
        point_crossmat = Eigen::Matrix3d::Zero();
        point_b = Eigen::Vector3d::Zero();
        point_i = Eigen::Vector3d::Zero();
        point_w = Eigen::Vector3d::Zero();
        normal = Eigen::Vector3d::Zero();
    };
} pointWithVar;

typedef struct PointToPlane {
    Eigen::Vector3d point_b_;
    Eigen::Vector3d point_w_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d center_;
    Eigen::Matrix<double, 3, 3> point_crossmat_;
    Eigen::Matrix<double, 6, 6> plane_var_;
    Eigen::Matrix3d body_cov_;
    int layer_;
    double d_;
    double eigen_value_;
    bool is_valid_;
    float dis_to_plane_;
    double dis_r;
} PointToPlane;

typedef struct VoxelPlane {
    Eigen::Vector3d center_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d y_normal_;
    Eigen::Vector3d x_normal_;
    Eigen::Matrix3d covariance_;
    Eigen::Matrix<double, 6, 6> plane_var_;
    float radius_ = 0;
    float min_eigen_value_ = 1;
    float mid_eigen_value_ = 1;
    float max_eigen_value_ = 1;
    float d_ = 0;
    int points_size_ = 0;
    bool is_plane_ = false;
    bool is_init_ = false;
    int id_ = 0;
    bool is_update_ = false;
    VoxelPlane() {
        plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
        covariance_ = Eigen::Matrix3d::Zero();
        center_ = Eigen::Vector3d::Zero();
        normal_ = Eigen::Vector3d::Zero();
    }
} VoxelPlane;


struct DS_POINT {
    float xyz[3];
    float intensity;
    int count = 0;
};

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree {
   public:
    VoxelOctoTree() = default;
    std::vector<pointWithVar> temp_points_;
    VoxelPlane *plane_ptr_;
    int layer_;
    int octo_state_;  // 0 is end of tree, 1 is not
    VoxelOctoTree *leaves_[8];
    double voxel_center_[3];  // x, y, z
    std::vector<int> layer_init_num_;
    float quater_length_;
    float planer_threshold_;
    int points_size_threshold_;
    int update_size_threshold_;
    int max_points_num_;
    int max_layer_;
    int new_points_;
    bool init_octo_;
    bool update_enable_;

    VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
        : max_layer_(max_layer),
          layer_(layer),
          points_size_threshold_(points_size_threshold),
          max_points_num_(max_points_num),
          planer_threshold_(planer_threshold) {
        temp_points_.clear();
        octo_state_ = 0;
        new_points_ = 0;
        update_size_threshold_ = 5;
        init_octo_ = false;
        update_enable_ = true;
        for (int i = 0; i < 8; i++) { leaves_[i] = nullptr; }
        plane_ptr_ = new VoxelPlane;
    }

    ~VoxelOctoTree() {
        for (int i = 0; i < 8; i++) { delete leaves_[i]; }
        delete plane_ptr_;
    }
    void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
    void init_octo_tree();
    void cut_octo_tree();
    void UpdateOctoTree(const pointWithVar &pv);

    VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
    VoxelOctoTree *Insert(const pointWithVar &pv);
};

// void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

class VoxelMapManager {
   public:
    VoxelMapManager() = default;
    VoxelMapConfig config_setting_;
    int current_frame_id_ = 0;
    ros::Publisher voxel_map_pub_;
    std::unordered_map<Eigen::Vector3i, VoxelOctoTree *, legkilo::hash_vec<3>, legkilo::equal_vec<3>> voxel_map_;

    legkilo::CloudPtr feats_undistort_;
    legkilo::CloudPtr feats_down_body_;
    legkilo::CloudPtr feats_down_world_;

    Eigen::Matrix3d extR_;
    Eigen::Vector3d extT_;
    float build_residual_time, ekf_time;
    float ave_build_residual_time = 0.0;
    float ave_ekf_time = 0.0;
    int scan_count = 0;
    // StatesGroup state_;
    Eigen::Vector3d position_last_;

    Eigen::Vector3d last_slide_position = {0, 0, 0};

    geometry_msgs::Quaternion geoQuat_;

    int feats_down_size_;
    int effct_feat_num_;
    std::vector<Eigen::Matrix3d> cross_mat_list_;
    std::vector<Eigen::Matrix3d> body_cov_list_;
    std::vector<pointWithVar> pv_list_;
    std::vector<PointToPlane> ptpl_list_;

    VoxelMapManager(VoxelMapConfig &config_setting) : config_setting_(config_setting) {
        current_frame_id_ = 0;
        feats_undistort_.reset(new legkilo::PointCloudType());
        feats_down_body_.reset(new legkilo::PointCloudType());
        feats_down_world_.reset(new legkilo::PointCloudType());
    };

    void BuildVoxelMap(const Eigen::Matrix3d rot, const Eigen::Matrix3d rot_cov, const Eigen::Matrix3d pos_cov);
    // V3F RGBFromVoxel(const V3D &input_point);

    void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

    // void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

    void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer,
                               bool &is_success, double &prob, PointToPlane &single_ptpl);

    void pubVoxelMap();

    bool mapSliding();
    void clearMemOutOfMap(const int &x_max, const int &x_min, const int &y_max, const int &y_min, const int &z_max,
                          const int &z_min);

    void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer,
                        std::vector<VoxelPlane> &plane_list);

    void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns,
                        const VoxelPlane &single_plane, const float alpha, const Eigen::Vector3d rgb);
    void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                         geometry_msgs::Quaternion &q);

    void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif  // VOXEL_MAP_H_