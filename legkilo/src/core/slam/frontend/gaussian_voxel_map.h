// SPDX-License-Identifier: MIT
// @file gaussian_voxel_map.h
// @brief Gaussian voxel map
// @author Ou Guangjun
// @created 2025-11-29
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_GAUSSIAN_VOXEL_MAP_H
#define LEG_KILO_GAUSSIAN_VOXEL_MAP_H

#include <array>
#include <limits>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"
#include "core/slam/frontend/voxel_map_utils.hpp"

namespace legkilo {
using VoxelKey = int64_t;
enum class NearByType { CENTER = 0, NEARBY6, NEARBY26 };

struct SubVoxel {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    size_t count = 0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d noise_sum = Eigen::Matrix3d::Zero();

    inline static double kJitter = 1e-6;  // numeric robustness

    inline void addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise) {
        const double n_old = static_cast<double>(count);
        const double n_new = n_old + 1.0;
        if (count == 0) {
            mean = p;
            cov = Eigen::Matrix3d::Identity() * kJitter;
            noise_sum = noise;
            count = 1;
            return;
        }
        const Eigen::Vector3d delta = p - mean;                 // x - mu_n
        const Eigen::Vector3d mean_new = mean + delta / n_new;  // mu_{n+1}
        const Eigen::Vector3d delta2 = p - mean_new;            // x - mu_{n+1}
        // Σ_{n+1} = (n/(n+1)) Σ_n + (1/(n+1)) δ δ'^T  (population covariance)
        cov *= (n_old / n_new);
        cov.noalias() += (delta * delta2.transpose()) / n_new;
        noise_sum += noise;
        mean = mean_new;
        count += 1;
    }
};

struct SubGrid {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int kDim = 2;                       // 3x3x3
    static constexpr int kCellNum = kDim * kDim * kDim;  // 27

    std::array<SubVoxel, kCellNum> cells;
    uint32_t occupancy_mask = 0;  // bit i -> cell i has at least one point
    uint8_t active_count = 0;

    Eigen::Vector3d origin = Eigen::Vector3d::Zero();  // min corner of parent voxel
    double sub_size = 0.0;                             // parent_voxel_size / 3
    double inv_sub_size = 0.0;

    SubGrid(const Eigen::Vector3d& origin_min, double parent_voxel_size) {
        origin = origin_min;
        sub_size = parent_voxel_size / static_cast<double>(kDim);
        inv_sub_size = 1.0 / sub_size;
    }

    inline static int flatten(int ix, int iy, int iz) { return iz * (kDim * kDim) + iy * kDim + ix; }
    inline static int clamp(int v) { return (v < 0) ? 0 : (v >= kDim ? (kDim - 1) : v); }

    // Add a point to the corresponding sub-voxel.
    void addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise);
    // Return pointer to the sub-voxel where p falls; nullptr if that cell is empty.
    const SubVoxel* nearest(const Eigen::Vector3d& p) const;
};

struct Plane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Public plane outputs
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();                              // normal of fitted plane
    Eigen::Vector3d center = Eigen::Vector3d::Zero();                              // mean/center of plane points
    Eigen::Matrix<double, 6, 6> cov_nq = Eigen::Matrix<double, 6, 6>::Identity();  // cov of normal and center
    double planarity = 0.0;                                                        // planarity of fitted plane
    bool valid = false;                                                            // whether the fitted plane is valid
    bool inited = false;  // whether the plane has been initialized

    void addPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& p_cov);
    void update();

   private:
    void fit();

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();                       // covariance of points
    bool updated = false;                                                       // whether updated in this round
    bool finalized = false;                                                     // whether frozen
    size_t count = 0;                                                           // point num
    size_t count_fit = 0;                                                       // temp point num for plane fit
    Eigen::Vector3d sum_p = Eigen::Vector3d::Zero();                            // sum of points
    Eigen::Matrix3d sum_ppT = Eigen::Matrix3d::Zero();                          // sum of point outer product
    std::unique_ptr<VoxelMapUtils::PlaneCovAccumulator> plane_cov_accumulator;  // plane covariance accumulator

   public:
    inline static size_t kMinPointsNumForPlaneInit = 5;
    inline static size_t kMinPointsNumForPlaneUpdateInterval = 2;
    inline static size_t kMaxPointsNum = 100;
    inline static float kPlanarRatio = 0.1f;
    inline static float kPlanarThickness2 = 0.0025f;
};

struct Voxel {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Voxel();
    ~Voxel() = default;

    Plane plane;
    std::unique_ptr<SubGrid> subgrid;

    inline void subgridAddPoint(const Eigen::Vector3d& p, const Eigen::Matrix3d& noise,
                                const Eigen::Vector3d& origin_min, double parent_voxel_size) {
        if (!subgrid) subgrid = std::make_unique<SubGrid>(origin_min, parent_voxel_size);
        subgrid->addPoint(p, noise);
    }
};
using VoxelPtr = std::shared_ptr<Voxel>;

struct KNearestInput {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KNearestInput(const Eigen::Vector3d* p_body, const Eigen::Matrix3d* p_cov_body, const Eigen::Vector3d* p_world,
                  const Eigen::Matrix3d* p_cov_world, const Eigen::Matrix3d* rot)
        : point_body(p_body),
          point_cov_body(p_cov_body),
          point_world(p_world),
          point_cov_world(p_cov_world),
          rot_predict(rot) {}
    const Eigen::Vector3d* point_body = nullptr;
    const Eigen::Matrix3d* point_cov_body = nullptr;
    const Eigen::Vector3d* point_world = nullptr;
    const Eigen::Matrix3d* point_cov_world = nullptr;
    const Eigen::Matrix3d* rot_predict = nullptr;
};

template <int DIM>
struct KNearestRes {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, DIM, 6> J;
    Eigen::Matrix<double, DIM, 1> r;
    Eigen::Matrix<double, DIM, DIM> R;
    double score = std::numeric_limits<double>::lowest();
    bool valid = false;
};

class GaussianVoxelMap {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Config {
        double voxel_size = 0.5;
        size_t capacity = 100000;
        NearByType nearby_type = NearByType::CENTER;
        float planar_ratio = 0.1f;
        float planar_thickness = 0.05f;
        size_t voxel_max_num = 50;
        bool p2p_enable = true;
        bool ndt_enable = false;
        size_t ndt_min_points = 5;
        double ndt_jitter = 1e-6;
        bool ndt_eigenvalue_regularization = true;
        double ndt_min_eigenvalue = 1e-6;
        double ndt_max_condition = 100.0;
    };

    struct ResidualNoise {
        double p2plane_meas_ratio = 10.0;
        double p2plane_min_noise = 1e-3;
        double ndt_meas_ratio = 10.0;
        double ndt_min_noise = 1e-3;
    };

    explicit GaussianVoxelMap(const Config& config);

    void insertPoints(const GaussCloud& cloud);

    bool buildPoint2PlaneResidual(const KNearestInput& knn_input, KNearestRes<1>& knn_res,
                                  const ResidualNoise& residual_noise) const;
    bool buildNdtResidual(const KNearestInput& knn_input, KNearestRes<3>& knn_res,
                          const ResidualNoise& residual_noise) const;

   private:
    void generateNearbyGrids();

    Eigen::Matrix3d regularizeNdtCovariance(const Eigen::Matrix3d& covariance) const;

    Config config_;

    double voxel_size_ = 0.5;
    double inv_voxel_size_ = 2.0;

    std::list<std::pair<VoxelKey, VoxelPtr>> voxel_list_;
    std::unordered_map<VoxelKey, std::list<std::pair<VoxelKey, VoxelPtr>>::iterator> voxel_map_;
    std::vector<Eigen::Vector3i> nearby_grids_;
};
}  // namespace legkilo
#endif  // LEG_KILO_GAUSSIAN_VOXEL_MAP_H
