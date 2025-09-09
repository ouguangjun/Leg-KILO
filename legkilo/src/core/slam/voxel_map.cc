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

#include "core/slam/voxel_map.h"

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov) {
    if (pb[2] == 0) pb[2] = 0.0001;
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane) {
    plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance_ = Eigen::Matrix3d::Zero();
    plane->center_ = Eigen::Vector3d::Zero();
    plane->normal_ = Eigen::Vector3d::Zero();
    plane->points_size_ = points.size();
    plane->radius_ = 0;
    for (auto pv : points) {
        plane->covariance_ += pv.point_w * pv.point_w.transpose();
        plane->center_ += pv.point_w;
    }
    plane->center_ = plane->center_ / plane->points_size_;
    plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    // if(evalsMid >= 3){
    //   std::cout << static_cast<int>(evalsMin) << " " << static_cast<int>(evalsMax) << std::endl;
    //   throw std::runtime_error("wrong evals mid index");
    // }
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;
    // && evalsReal(evalsMid) > 0.05
    //&& evalsReal(evalsMid) > 0.01
    if (evalsReal(evalsMin) < planer_threshold_) {
        for (int i = 0; i < points.size(); i++) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m = (points[i].point_w - plane->center_).transpose() /
                                                      ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
                                                      (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                                                       evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                    F.row(m) = F_m;
                } else {
                    Eigen::Matrix<double, 1, 3> F_m;
                    F_m << 0, 0, 0;
                    F.row(m) = F_m;
                }
            }
            J.block<3, 3>(0, 0) = evecs.real() * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_var_ += J * points[i].var * J.transpose();
        }

        plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->min_eigen_value_ = evalsReal(evalsMin);
        plane->mid_eigen_value_ = evalsReal(evalsMid);
        plane->max_eigen_value_ = evalsReal(evalsMax);
        plane->radius_ = sqrt(evalsReal(evalsMax));
        plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) +
                      plane->normal_(2) * plane->center_(2));
        plane->is_plane_ = true;
        plane->is_update_ = true;
        if (!plane->is_init_) {
            plane->id_ = voxel_plane_id;
            voxel_plane_id++;
            plane->is_init_ = true;
        }
    } else {
        plane->is_update_ = true;
        plane->is_plane_ = false;
    }
}

void VoxelOctoTree::init_octo_tree() {
    if (temp_points_.size() > points_size_threshold_) {
        init_plane(temp_points_, plane_ptr_);
        if (plane_ptr_->is_plane_ == true) {
            octo_state_ = 0;
            // new added
            if (temp_points_.size() > max_points_num_) {
                update_enable_ = false;
                std::vector<pointWithVar>().swap(temp_points_);
                new_points_ = 0;
            }
        } else {
            octo_state_ = 1;
            cut_octo_tree();
        }
        init_octo_ = true;
        new_points_ = 0;
    }
}

void VoxelOctoTree::cut_octo_tree() {
    if (layer_ >= max_layer_) {
        octo_state_ = 0;
        return;
    }
    for (size_t i = 0; i < temp_points_.size(); i++) {
        int xyz[3] = {0, 0, 0};
        if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_,
                                                 planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_++;
    }
    for (uint i = 0; i < 8; i++) {
        if (leaves_[i] != nullptr) {
            if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_) {
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                if (leaves_[i]->plane_ptr_->is_plane_) {
                    leaves_[i]->octo_state_ = 0;
                    // new added
                    if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_) {
                        leaves_[i]->update_enable_ = false;
                        std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
                        new_points_ = 0;
                    }
                } else {
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octo_tree();
                }
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_ = 0;
            }
        }
    }
}

void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv) {
    if (!init_octo_) {
        new_points_++;
        temp_points_.push_back(pv);
        if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
    } else {
        if (plane_ptr_->is_plane_) {
            if (update_enable_) {
                new_points_++;
                temp_points_.push_back(pv);
                if (new_points_ > update_size_threshold_) {
                    init_plane(temp_points_, plane_ptr_);
                    new_points_ = 0;
                }
                if (temp_points_.size() >= max_points_num_) {
                    update_enable_ = false;
                    std::vector<pointWithVar>().swap(temp_points_);
                    new_points_ = 0;
                }
            }
        } else {
            if (layer_ < max_layer_) {
                int xyz[3] = {0, 0, 0};
                if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
                if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
                if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves_[leafnum] != nullptr) {
                    leaves_[leafnum]->UpdateOctoTree(pv);
                } else {
                    leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1],
                                                         max_points_num_, planer_threshold_);
                    leaves_[leafnum]->layer_init_num_ = layer_init_num_;
                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->UpdateOctoTree(pv);
                }
            } else {
                if (update_enable_) {
                    new_points_++;
                    temp_points_.push_back(pv);
                    if (new_points_ > update_size_threshold_) {
                        init_plane(temp_points_, plane_ptr_);
                        new_points_ = 0;
                    }
                    if (temp_points_.size() > max_points_num_) {
                        update_enable_ = false;
                        std::vector<pointWithVar>().swap(temp_points_);
                        new_points_ = 0;
                    }
                }
            }
        }
    }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw) {
    if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

    int xyz[3] = {0, 0, 0};
    xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    // printf("leafnum: %d. \n", leafnum);

    return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv) {
    if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) ||
        (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_))) {
        new_points_++;
        temp_points_.push_back(pv);
        return this;
    }

    if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_)) {
        int xyz[3] = {0, 0, 0};
        xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
        xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
        xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) {
            return leaves_[leafnum]->Insert(pv);
        } else {
            leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_,
                                                 planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            return leaves_[leafnum]->Insert(pv);
        }
    }
    return nullptr;
}

void VoxelMapManager::BuildVoxelMap(const Eigen::Matrix3d rot, const Eigen::Matrix3d rot_cov,
                                    const Eigen::Matrix3d pos_cov) {
    float voxel_size = config_setting_.max_voxel_size_;
    float planer_threshold = config_setting_.planner_threshold_;
    int max_layer = config_setting_.max_layer_;
    int max_points_num = config_setting_.max_points_num_;
    std::vector<int> layer_init_num = config_setting_.layer_init_num_;

    std::vector<pointWithVar> input_points;

    for (size_t i = 0; i < feats_down_world_->size(); i++) {
        pointWithVar pv;
        pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
        Eigen::Vector3d point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y,
                                   feats_down_body_->points[i].z);
        Eigen::Matrix3d var;
        calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
        Eigen::Matrix3d point_crossmat;
        point_crossmat = legkilo::SKEW_SYM_MATRIX(point_this);
        var = (rot * extR_) * var * (rot * extR_).transpose() +
              (-point_crossmat) * rot_cov * (-point_crossmat).transpose() + pos_cov;
        pv.var = var;
        input_points.push_back(pv);
    }

    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const pointWithVar p_v = input_points[i];
        Eigen::Vector3i position = legkilo::voxelKeyFloor(p_v.point_w, voxel_size);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_++;
        } else {
            VoxelOctoTree *octo_tree =
                new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->quater_length_ = voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] = (0.5 + position[0]) * voxel_size;
            voxel_map_[position]->voxel_center_[1] = (0.5 + position[1]) * voxel_size;
            voxel_map_[position]->voxel_center_[2] = (0.5 + position[2]) * voxel_size;
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_++;
            voxel_map_[position]->layer_init_num_ = layer_init_num;
        }
    }
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) { iter->second->init_octo_tree(); }
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points) {
    float voxel_size = config_setting_.max_voxel_size_;
    float planer_threshold = config_setting_.planner_threshold_;
    int max_layer = config_setting_.max_layer_;
    int max_points_num = config_setting_.max_points_num_;
    std::vector<int> layer_init_num = config_setting_.layer_init_num_;
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const pointWithVar p_v = input_points[i];
        Eigen::Vector3i position = legkilo::voxelKeyFloor(p_v.point_w, voxel_size);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->UpdateOctoTree(p_v);
        } else {
            VoxelOctoTree *octo_tree =
                new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->layer_init_num_ = layer_init_num;
            voxel_map_[position]->quater_length_ = voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] = (0.5 + position[0]) * voxel_size;
            voxel_map_[position]->voxel_center_[1] = (0.5 + position[1]) * voxel_size;
            voxel_map_[position]->voxel_center_[2] = (0.5 + position[2]) * voxel_size;
            voxel_map_[position]->UpdateOctoTree(p_v);
        }
    }
}

void VoxelMapManager::build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo,
                                            const int current_layer, bool &is_success, double &prob,
                                            PointToPlane &single_ptpl) {
    int max_layer = config_setting_.max_layer_;
    double sigma_num = config_setting_.sigma_num_;

    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_w;
    if (current_octo->plane_ptr_->is_plane_) {
        VoxelPlane &plane = *current_octo->plane_ptr_;
        Eigen::Vector3d p_world_to_center = p_w - plane.center_;
        float dis_to_plane =
            fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);
        float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
                              (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                              (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius_) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
            J_nq.block<1, 3>(0, 3) = -plane.normal_;
            double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
            sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
            if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
                is_success = true;
                double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    pv.normal = plane.normal_;
                    single_ptpl.body_cov_ = pv.body_var;
                    single_ptpl.point_b_ = pv.point_b;
                    single_ptpl.point_w_ = pv.point_w;
                    single_ptpl.plane_var_ = plane.plane_var_;
                    single_ptpl.normal_ = plane.normal_;
                    single_ptpl.center_ = plane.center_;
                    single_ptpl.d_ = plane.d_;
                    single_ptpl.layer_ = current_layer;
                    single_ptpl.dis_to_plane_ =
                        plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
                    single_ptpl.point_crossmat_ = pv.point_crossmat;
                }
                return;
            } else {
                // is_success = false;
                return;
            }
        } else {
            // is_success = false;
            return;
        }
    } else {
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->leaves_[leafnum] != nullptr) {
                    VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
                    build_single_residual(pv, leaf_octo, current_layer + 1, is_success, prob, single_ptpl);
                }
            }
            return;
        } else {
            return;
        }
    }
}

void VoxelMapManager::pubVoxelMap() {
    double max_trace = 0.25;
    double pow_num = 0.2;
    ros::Rate loop(500);
    float use_alpha = 0.8;
    visualization_msgs::MarkerArray voxel_plane;
    voxel_plane.markers.reserve(1000000);
    std::vector<VoxelPlane> pub_plane_list;
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++) {
        GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
    }
    for (size_t i = 0; i < pub_plane_list.size(); i++) {
        Eigen::Vector3d plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
        double trace = plane_cov.sum();
        if (trace >= max_trace) { trace = max_trace; }
        trace = trace * (1.0 / max_trace);
        trace = pow(trace, pow_num);
        uint8_t r, g, b;
        mapJet(trace, 0, 1, r, g, b);
        Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
        double alpha;
        if (pub_plane_list[i].is_plane_) {
            alpha = use_alpha;
        } else {
            alpha = 0;
        }
        pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
    }
    voxel_map_pub_.publish(voxel_plane);
    loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer,
                                     std::vector<VoxelPlane> &plane_list) {
    if (current_octo->layer_ > pub_max_voxel_layer) { return; }
    if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane_) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list);
                }
            }
        }
    }
    return;
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns,
                                     const VoxelPlane &single_plane, const float alpha, const Eigen::Vector3d rgb) {
    visualization_msgs::Marker plane;
    plane.header.frame_id = "camera_init";
    plane.header.stamp = ros::Time();
    plane.ns = plane_ns;
    plane.id = single_plane.id_;
    plane.type = visualization_msgs::Marker::CYLINDER;
    plane.action = visualization_msgs::Marker::ADD;
    plane.pose.position.x = single_plane.center_[0];
    plane.pose.position.y = single_plane.center_[1];
    plane.pose.position.z = single_plane.center_[2];
    geometry_msgs::Quaternion q;
    CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
    plane.pose.orientation = q;
    plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
    plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
    plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
    plane.color.a = alpha;
    plane.color.r = rgb(0);
    plane.color.g = rgb(1);
    plane.color.b = rgb(2);
    plane.lifetime = ros::Duration();
    plane_pub.markers.push_back(plane);
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec,
                                      const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q) {
    Eigen::Matrix3d rot;
    rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
    Eigen::Matrix3d rotation = rot.transpose();
    Eigen::Quaterniond eq(rotation);
    q.w = eq.w();
    q.x = eq.x();
    q.y = eq.y();
    q.z = eq.z();
}

void VoxelMapManager::mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b) {
    r = 255;
    g = 255;
    b = 255;

    if (v < vmin) { v = vmin; }

    if (v > vmax) { v = vmax; }

    double dr, dg, db;

    if (v < 0.1242) {
        db = 0.504 + ((1. - 0.504) / 0.1242) * v;
        dg = dr = 0.;
    } else if (v < 0.3747) {
        db = 1.;
        dr = 0.;
        dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
    } else if (v < 0.6253) {
        db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
        dg = 1.;
        dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
    } else if (v < 0.8758) {
        db = 0.;
        dr = 1.;
        dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
    } else {
        db = 0.;
        dg = 0.;
        dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
    }

    r = (uint8_t)(255 * dr);
    g = (uint8_t)(255 * dg);
    b = (uint8_t)(255 * db);
}

bool VoxelMapManager::mapSliding() {
    if ((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh) {
        // std::cout<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
        return false;
    }

    // get global id now
    last_slide_position = position_last_;
    // double t_sliding_start = omp_get_wtime();
    Eigen::Vector3i k = legkilo::voxelKeyFloor(position_last_, config_setting_.max_voxel_size_);
    int ix = k[0];
    int iy = k[1];
    int iz = k[2];
    clearMemOutOfMap(ix + config_setting_.half_map_size, ix - config_setting_.half_map_size,
                     iy + config_setting_.half_map_size, iy - config_setting_.half_map_size,
                     iz + config_setting_.half_map_size, iz - config_setting_.half_map_size);
    // double t_sliding_end = omp_get_wtime();
    // std::cout<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
    return true;
}

void VoxelMapManager::clearMemOutOfMap(const int &x_max, const int &x_min, const int &y_max, const int &y_min,
                                       const int &z_max, const int &z_min) {
    int delete_voxel_cout = 0;
    // double delete_time = 0;
    // double last_delete_time = 0;
    for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
        const Eigen::Vector3i &loc = it->first;
        bool should_remove =
            loc[0] > x_max || loc[0] < x_min || loc[1] > y_max || loc[1] < y_min || loc[2] > z_max || loc[2] < z_min;
        if (should_remove) {
            // last_delete_time = omp_get_wtime();
            delete it->second;
            it = voxel_map_.erase(it);
            // delete_time += omp_get_wtime() - last_delete_time;
            delete_voxel_cout++;
        } else {
            ++it;
        }
    }
    // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
    // std::cout<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}
