#include "core/slam/frontend/gaussian_voxel_map.h"

#include <algorithm>

#include <glog/logging.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "common/math_utils.hpp"

namespace legkilo {
void SubGrid::addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise) {
    const Eigen::Vector3d rel = p - origin;
    Eigen::Vector3i sub_idx = voxelKeyFastFloor(rel, inv_sub_size);
    int ix = clamp(sub_idx.x());
    int iy = clamp(sub_idx.y());
    int iz = clamp(sub_idx.z());
    const int idx = flatten(ix, iy, iz);
    SubVoxel& cell = cells[idx];
    const bool was_empty = (cell.count == 0);
    cell.addPoint(p, noise);
    if (was_empty) {
        occupancy_mask |= (1u << idx);
        ++active_count;
    }
}

const SubVoxel* SubGrid::nearest(const Eigen::Vector3d& p) const {
    if (occupancy_mask == 0) return nullptr;
    const Eigen::Vector3d rel = p - origin;
    Eigen::Vector3i sub_idx = voxelKeyFastFloor(rel, inv_sub_size);
    int ix = clamp(sub_idx.x());
    int iy = clamp(sub_idx.y());
    int iz = clamp(sub_idx.z());
    const int idx = flatten(ix, iy, iz);
    if (((occupancy_mask >> idx) & 1u) == 0u) return nullptr;
    return &cells[idx];
}

Voxel::Voxel() {}

void Plane::addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& p_cov) {
    if (finalized) return;
    updated = false;
    ++count;
    ++count_fit;
    sum_p += p;
    sum_ppT.noalias() += p * p.transpose();
    if (!plane_cov_accumulator) { plane_cov_accumulator = std::make_unique<VoxelMapUtils::PlaneCovAccumulator>(); }
    plane_cov_accumulator->addPoint(p, p_cov);
}

void Plane::update() {
    if (updated || finalized) return;

    const double n = static_cast<double>(count);
    if (n <= 0.0) return;
    center = sum_p / n;
    covariance = sum_ppT / n - center * center.transpose();

    bool if_update_plane =
        (inited && count_fit >= kMinPointsNumForPlaneUpdateInterval) || (!inited && count >= kMinPointsNumForPlaneInit);
    if (if_update_plane) { this->fit(); }

    if (count > kMaxPointsNum) {
        finalized = true;
        plane_cov_accumulator.reset();
    }

    updated = true;
    return;
}

void Plane::fit() {
    Eigen::Matrix3d cov_sym = 0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov_sym);
    if (eig.info() != Eigen::Success) return;
    inited = true;
    Eigen::Vector3d evals = eig.eigenvalues();
    Eigen::Matrix3d evecs = eig.eigenvectors();

    const double planar_ratio = evals[0] / (evals.sum() + 1e-6);
    const double planar_thickness2 = evals[0];

    planarity = planar_thickness2;

    if (planar_ratio > kPlanarRatio || planar_thickness2 > kPlanarThickness2) {
        valid = false;
    } else {
        valid = true;
        normal = evecs.col(0).normalized();
        plane_cov_accumulator->computePlaneCov(evecs, evals[0], evals[1], evals[2], cov_nq);
    }
    count_fit = 0;
    return;
}

GaussianVoxelMap::GaussianVoxelMap(const Config& config) : config_(config) {
    voxel_size_ = config_.voxel_size;
    inv_voxel_size_ = 1.0 / voxel_size_;
    Plane::kMaxPointsNum = config_.voxel_max_num;
    Plane::kPlanarRatio = config_.planar_ratio;
    Plane::kPlanarThickness2 = config_.planar_thickness * config_.planar_thickness;
    generateNearbyGrids();
}

Eigen::Matrix3d GaussianVoxelMap::regularizeNdtCovariance(const Eigen::Matrix3d& covariance) const {
    Eigen::Matrix3d covariance_sym = 0.5 * (covariance + covariance.transpose());
    const double jitter = std::max(config_.ndt_jitter, 0.0);
    covariance_sym.diagonal().array() += jitter;

    if (!config_.ndt_eigenvalue_regularization) return covariance_sym;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance_sym);
    if (eig.info() != Eigen::Success) return covariance_sym;

    Eigen::Vector3d evals = eig.eigenvalues().cwiseMax(jitter);
    const double lambda_max = std::max(evals.maxCoeff(), jitter);
    double lambda_floor = std::max(config_.ndt_min_eigenvalue, jitter);
    if (config_.ndt_max_condition > 1.0) {
        lambda_floor = std::max(lambda_floor, lambda_max / config_.ndt_max_condition);
    }
    evals = evals.cwiseMax(lambda_floor);

    return eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();
}

void GaussianVoxelMap::insertPoints(const GaussCloud& cloud) {
    const size_t N = cloud.size();
    if (N == 0) return;
    std::vector<std::pair<int64_t, size_t>> voxel_entries;
    voxel_entries.resize(N);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, N, 1024), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            const auto& point = cloud[i];
            const auto key = voxelKeyFastFloor(point.pt, inv_voxel_size_);
            voxel_entries[i].first = encodeKey(key);
            voxel_entries[i].second = i;
        }
    });

    tbb::parallel_sort(
        voxel_entries.begin(), voxel_entries.end(),
        [](const std::pair<int64_t, size_t>& a, const std::pair<int64_t, size_t>& b) { return a.first < b.first; });

    size_t start = 0;
    while (start < N) {
        size_t begin = start;
        size_t end = start + 1;
        while (end < N && voxel_entries[end].first == voxel_entries[begin].first) { ++end; }

        const VoxelKey key_code = voxel_entries[begin].first;
        auto it = voxel_map_.find(key_code);
        std::list<std::pair<VoxelKey, VoxelPtr>>::iterator list_it;
        if (it == voxel_map_.end()) {
            VoxelPtr voxel_ptr = std::make_shared<Voxel>();
            voxel_list_.push_front(std::make_pair(key_code, voxel_ptr));
            voxel_map_.emplace(key_code, voxel_list_.begin());
            list_it = voxel_list_.begin();
            if (voxel_list_.size() > config_.capacity) {
                voxel_map_.erase(voxel_list_.back().first);
                voxel_list_.pop_back();
            }
        } else {
            voxel_list_.splice(voxel_list_.begin(), voxel_list_, it->second);
            list_it = it->second;
        }

        VoxelPtr& voxel_ptr = list_it->second;
        if (config_.p2p_enable) {
            for (size_t i = begin; i < end; ++i) {
                const auto& point = cloud[voxel_entries[i].second];
                voxel_ptr->plane.addPoint(point.pt, point.cov);
            }
            voxel_ptr->plane.update();
        }

        if (config_.ndt_enable) {
            const Eigen::Vector3i grid_idx = decodeKey(key_code);
            const Eigen::Vector3d origin_min = grid_idx.cast<double>() * voxel_size_;
            for (size_t i = begin; i < end; ++i) {
                const auto& point = cloud[voxel_entries[i].second];
                voxel_ptr->subgridAddPoint(point.pt, point.cov, origin_min, voxel_size_);
            }
        }

        start = end;
    }
}

bool GaussianVoxelMap::buildPoint2PlaneResidual(const KNearestInput& knn_input, KNearestRes<1>& knn_res,
                                                const ResidualNoise& residual_noise) const {
    static constexpr double kSigmaTimes2 = 9.0;
    static constexpr double kMinVar = 1e-12;

    const Eigen::Vector3d& p_world = *knn_input.point_world;
    const Eigen::Vector3d& p_body = *knn_input.point_body;
    const Eigen::Matrix3d& cov_world = *knn_input.point_cov_world;
    const Eigen::Matrix3d& cov_body = *knn_input.point_cov_body;
    const Eigen::Matrix3d& rot_predict = *knn_input.rot_predict;

    const Eigen::Vector3i p_floor = voxelKeyFastFloor(p_world, inv_voxel_size_);

    for (size_t i = 0; i < nearby_grids_.size(); ++i) {
        Eigen::Vector3i cur_floor = p_floor + nearby_grids_[i];
        VoxelKey key = encodeKey(cur_floor);

        auto iter = voxel_map_.find(key);
        if (iter == voxel_map_.end()) continue;

        const VoxelPtr cur_voxel = iter->second->second;
        if (!cur_voxel->plane.valid) continue;

        Eigen::RowVector3d normalT = cur_voxel->plane.normal.transpose();
        Eigen::Vector3d diff = p_world - cur_voxel->plane.center;
        double d = normalT * diff;

        Eigen::Matrix<double, 1, 6> Jw_nq;
        Eigen::Matrix<double, 1, 3> Jw_p;
        Jw_nq.block<1, 3>(0, 0) = diff.transpose();
        Jw_nq.block<1, 3>(0, 3) = -normalT;
        Jw_p = normalT;

        double sigma2_plane = (Jw_nq * cur_voxel->plane.cov_nq * Jw_nq.transpose())(0, 0);
        double sigma2_point = (Jw_p * cov_world * Jw_p.transpose())(0, 0);
        double sigma2 = sigma2_plane + sigma2_point;

        if (d * d > kSigmaTimes2 * sigma2) continue;

        double probability = gaussianPdf1D(d, std::sqrt(std::max(sigma2, kMinVar)));
        if (probability > knn_res.score) {
            knn_res.valid = true;
            knn_res.score = probability;

            Eigen::Matrix<double, 1, 3> Jv_p = normalT * rot_predict;
            double var_plane_R = sigma2_plane;
            double var_point_R = (Jv_p * cov_body * Jv_p.transpose())(0, 0);
            double var_R =
                residual_noise.p2plane_meas_ratio * (var_plane_R + var_point_R) + residual_noise.p2plane_min_noise;
            const double inv_std = 1.0 / std::sqrt(var_R);

            knn_res.r(0, 0) = -d * inv_std;
            knn_res.J.block<1, 3>(0, 0) = -normalT * rot_predict * SKEW_SYM_MATRIX(p_body) * inv_std;
            knn_res.J.block<1, 3>(0, 3) = normalT * inv_std;
            knn_res.R.setIdentity();
        }
    }

    return knn_res.valid;
}

bool GaussianVoxelMap::buildNdtResidual(const KNearestInput& knn_input, KNearestRes<3>& knn_res,
                                        const ResidualNoise& residual_noise) const {
    knn_res.valid = false;
    const Eigen::Vector3d& p_world = *knn_input.point_world;
    const Eigen::Vector3d& p_body = *knn_input.point_body;
    const Eigen::Matrix3d& cov_body = *knn_input.point_cov_body;
    const Eigen::Matrix3d& rot_predict = *knn_input.rot_predict;

    double best_dist2 = std::numeric_limits<double>::infinity();
    const double kMaxDistance2 = 0.5 * 0.5 * voxel_size_ * voxel_size_;

    Eigen::Matrix<double, 3, 6> H3;
    H3.block<3, 3>(0, 0) = -rot_predict * SKEW_SYM_MATRIX(p_body);
    H3.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Vector3i p_floor = voxelKeyFastFloor(p_world, inv_voxel_size_);
    for (size_t i = 0; i < nearby_grids_.size(); ++i) {
        const Eigen::Vector3i cur_floor = p_floor + nearby_grids_[i];
        const VoxelKey key = encodeKey(cur_floor);

        auto iter = voxel_map_.find(key);
        if (iter == voxel_map_.end()) continue;

        const VoxelPtr vox = iter->second->second;
        if (!vox || !vox->subgrid) continue;

        const SubGrid& sg = *vox->subgrid;
        const uint32_t mask = sg.occupancy_mask;

        for (int idx = 0; idx < SubGrid::kCellNum; ++idx) {
            if (((mask >> idx) & 1u) == 0u) continue;
            const SubVoxel& cell = sg.cells[idx];
            if (cell.count < config_.ndt_min_points) continue;

            const Eigen::Vector3d r3 = p_world - cell.mean;
            const double dist2 = r3.squaredNorm();

            if (dist2 > kMaxDistance2) continue;

            Eigen::Matrix3d S = cell.cov + cell.noise_sum / static_cast<double>(cell.count) +
                                rot_predict * cov_body * rot_predict.transpose();
            S = residual_noise.ndt_meas_ratio * regularizeNdtCovariance(S);
            S.diagonal().array() += residual_noise.ndt_min_noise;
            S = 0.5 * (S + S.transpose());

            Eigen::LLT<Eigen::Matrix3d> llt(S);
            if (llt.info() != Eigen::Success) continue;
            const auto Ltri = llt.matrixL();

            const Eigen::Vector3d r_w = Ltri.solve(r3);

            if (dist2 < best_dist2) {
                const Eigen::Matrix<double, 3, 6> H_w = Ltri.solve(H3);
                best_dist2 = dist2;
                knn_res.J = H_w;
                knn_res.r = -r_w;
                knn_res.R.setIdentity();
                knn_res.score = -dist2;
                knn_res.valid = true;
            }
        }
    }

    return knn_res.valid;
}

void GaussianVoxelMap::generateNearbyGrids() {
    nearby_grids_.clear();
    if (config_.nearby_type == NearByType::CENTER) {
        nearby_grids_ = {Eigen::Vector3i(0, 0, 0)};
    } else if (config_.nearby_type == NearByType::NEARBY6) {
        nearby_grids_ = {Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(1, 0, 0),
                         Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, -1),
                         Eigen::Vector3i(0, 0, 1)};
    } else if (config_.nearby_type == NearByType::NEARBY26) {
        nearby_grids_.reserve(27);
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                for (int z = -1; z <= 1; z++) { nearby_grids_.emplace_back(Eigen::Vector3i(x, y, z)); }
            }
        }
    } else {
        LOG(ERROR) << "Undefined NearbyType !";
        nearby_grids_ = {Eigen::Vector3i(0, 0, 0)};
    }
}
}  // namespace legkilo
