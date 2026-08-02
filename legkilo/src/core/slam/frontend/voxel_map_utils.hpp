// SPDX-License-Identifier: MIT
// @file voxel_map_utils.hpp
// @brief Voxel map utilities and plane covariance accumulator.
// @author Ou Guangjun
// @created 2025-11-29
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_VOXEL_MAP_UTILS_H
#define LEG_KILO_VOXEL_MAP_UTILS_H

#include <Eigen/Core>
#include <cmath>

namespace legkilo {
namespace VoxelMapUtils {

/// @brief Incremental accumulator for plane covariance as described in:
///        Yang X, Li W, Ge Q, et al.
///        "C3P-VoxelMap: Compact, Cumulative and Coalescible Probabilistic Voxel Mapping"
///        Proc. 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 7908–7915.
///        This struct implements the incremental statistics for the joint covariance of [n; q],
///        where n is the plane normal and q is the plane center.
struct PlaneCovAccumulator {
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    int N = 0;              ///< Number of points.
    Vec3 q = Vec3::Zero();  ///< Mean of all points used for the plane.

    // Store only 6 independent X_{j,k} blocks:
    //   X_diag[i] = X_{i,i}
    //   X_off[0]  = X_{0,1} = X_{1,0}
    //   X_off[1]  = X_{0,2} = X_{2,0}
    //   X_off[2]  = X_{1,2} = X_{2,1}
    Mat3 X_diag[3];  ///< X_{0,0}, X_{1,1}, X_{2,2}
    Mat3 X_off[3];   ///< X_{0,1}, X_{0,2}, X_{1,2}

    Mat3 Y[3];              ///< Y_j = Σ Σ_p * e_j * p^T
    Mat3 Z = Mat3::Zero();  ///< Z   = Σ Σ_p

    PlaneCovAccumulator() { reset(); }

    /// @brief Reset all statistics to zero.
    void reset() {
        N = 0;
        q.setZero();
        Z.setZero();
        for (int i = 0; i < 3; ++i) {
            X_diag[i].setZero();
            Y[i].setZero();
            X_off[i].setZero();
        }
    }

    /// @brief Incrementally add a point and its covariance in the world frame.
    /// @param p        Point position in the world frame.
    /// @param Sigma_p  3x3 point covariance in the world frame (symmetric).
    inline void addPoint(const Vec3& p, const Mat3& Sigma_p) {
        // Incremental update of the mean q.
        if (N == 0) {
            q = p;
        } else {
            const double n = static_cast<double>(N);
            const double n1 = n + 1.0;
            q = (n / n1) * q + (1.0 / n1) * p;
        }

        Mat3 ppT = p * p.transpose();

        // Update X blocks using the 6 independent elements of Sigma_p.
        X_diag[0].noalias() += Sigma_p(0, 0) * ppT;  // X00
        X_diag[1].noalias() += Sigma_p(1, 1) * ppT;  // X11
        X_diag[2].noalias() += Sigma_p(2, 2) * ppT;  // X22

        X_off[0].noalias() += Sigma_p(0, 1) * ppT;  // X01 = X10
        X_off[1].noalias() += Sigma_p(0, 2) * ppT;  // X02 = X20
        X_off[2].noalias() += Sigma_p(1, 2) * ppT;  // X12 = X21

        // Update Y_j: Y_j += Σ_p * e_j * p^T  (Sigma_p.col(j) = Σ_p * e_j).
        for (int j = 0; j < 3; ++j) { Y[j].noalias() += Sigma_p.col(j) * p.transpose(); }

        // Update Z: Z += Σ_p.
        Z.noalias() += Sigma_p;

        ++N;
    }

    /// @brief Compute the joint covariance of [n; q] as a 6x6 matrix.
    ///
    /// This uses the incremental statistics together with the eigen-decomposition
    /// of the point covariance A = E[(p - q)(p - q)^T].
    ///
    /// @param U           3x3 eigenvector matrix of A.
    ///                    U.col(0) must correspond to lambda_min (plane normal).
    /// @param lambda_min  Smallest eigenvalue of A (normal direction).
    /// @param lambda1     Second eigenvalue of A.
    /// @param lambda2     Largest eigenvalue of A.
    /// @param plane_cov   Output 6x6 covariance matrix ordered as [n_x n_y n_z q_x q_y q_z]^T.
    /// @return            False if N < 3 or degenerate, true on success.
    bool computePlaneCov(const Mat3& U, double lambda_min, double lambda1, double lambda2,
                         Eigen::Matrix<double, 6, 6>& plane_cov) const {
        plane_cov.setZero();
        if (N < 3) { return false; }

        const double n_pts = static_cast<double>(N);
        const Vec3 n = U.col(0);  // Normal direction.

        // Standard basis.
        const Vec3 ex(1.0, 0.0, 0.0);
        const Vec3 ey(0.0, 1.0, 0.0);
        const Vec3 ez(0.0, 0.0, 1.0);
        const Vec3 e[3] = {ex, ey, ez};

        // 1) Construct F_m (m = 0, 1, 2) as in the paper.
        Mat3 F[3];
        F[0].setZero();  // Direction of the smallest eigenvalue (normal) has F = 0.

        auto makeF = [&](int idx, double lambda_m) -> Mat3 {
            Mat3 Fm;
            double denom = n_pts * (lambda_min - lambda_m);
            if (std::abs(denom) < 1e-12) {
                Fm.setZero();
            } else {
                Vec3 um = U.col(idx);
                // F_m = 1 / [N (λ_min - λ_m)] * (u_m n^T + n u_m^T)
                Fm.noalias() = (um * n.transpose() + n * um.transpose()) / denom;
            }
            return Fm;
        };

        F[1] = makeF(1, lambda1);
        F[2] = makeF(2, lambda2);

        const Vec3& qc = q;
        const Mat3& ZZ = Z;

        // Helper: Term2(Fm, Fn) = Σ_i p_i^T Fm Σ_i Fn^T q.
        auto Term2 = [&](const Mat3& Fm, const Mat3& Fn) -> double {
            Vec3 r = Fn.transpose() * qc;  // r_j = e_j^T Fn^T q
            Mat3 S = Mat3::Zero();
            for (int j = 0; j < 3; ++j) {
                S.noalias() += r(j) * Y[j];  // S = Σ_j r_j * Y_j
            }
            return (S * Fm).trace();
        };

        // 2) Compute B (3x3) s.t. Σ_nn = U B U^T.
        Mat3 B = Mat3::Zero();

        for (int m = 0; m < 3; ++m) {
            for (int n_ = 0; n_ < 3; ++n_) {
                const Mat3& Fm = F[m];
                const Mat3& Fn = F[n_];

                // Precompute a_j = Fm * e_j, b_k = Fn * e_k.
                Vec3 a[3], b[3];
                for (int j = 0; j < 3; ++j) {
                    a[j] = Fm * e[j];
                    b[j] = Fn * e[j];
                }

                // Rank-1 matrices M_jk = a_j * b_k^T.
                Mat3 M00 = a[0] * b[0].transpose();
                Mat3 M11 = a[1] * b[1].transpose();
                Mat3 M22 = a[2] * b[2].transpose();

                Mat3 M01p = a[0] * b[1].transpose() + a[1] * b[0].transpose();  // M01 + M10
                Mat3 M02p = a[0] * b[2].transpose() + a[2] * b[0].transpose();  // M02 + M20
                Mat3 M12p = a[1] * b[2].transpose() + a[2] * b[1].transpose();  // M12 + M21

                // Term1 = Σ_{j,k} tr(X_{j,k} M_{j,k})
                double t1 = 0.0;
                t1 += (X_diag[0] * M00).trace();
                t1 += (X_diag[1] * M11).trace();
                t1 += (X_diag[2] * M22).trace();
                t1 += (X_off[0] * M01p).trace();  // X01
                t1 += (X_off[1] * M02p).trace();  // X02
                t1 += (X_off[2] * M12p).trace();  // X12

                double t2 = Term2(Fm, Fn);
                double t3 = Term2(Fn, Fm);
                double t4 = qc.transpose() * Fm * ZZ * Fn.transpose() * qc;

                B(m, n_) = t1 - t2 - t3 + t4;
            }
        }

        // Σ_nn = U B U^T.
        Mat3 Sigma_nn = U * B * U.transpose();

        // 3) Compute Σ_nq = (1/N) * U * C.
        Mat3 C = Mat3::Zero();

        for (int m = 0; m < 3; ++m) {
            const Mat3& Fm = F[m];

            for (int l = 0; l < 3; ++l) {
                // C_{m,l} = Σ_i (p_i - q)^T Fm Σ_i e_l
                //         = Σ_i p_i^T Fm Σ_i e_l - q^T Fm Σ_i Σ_i e_l.
                // First term: Σ_i p_i^T Fm Σ_i e_l = tr(Y_l^T Fm).
                double t1 = (Y[l].transpose() * Fm).trace();

                // Second term: q^T Fm Z e_l.
                Vec3 v = ZZ * e[l];
                double t2 = qc.transpose() * Fm * v;

                C(m, l) = t1 - t2;
            }
        }

        Mat3 Sigma_nq = (1.0 / n_pts) * (U * C);

        // 4) Σ_qq = (1/N^2) Σ Σ_p_i = Z / N^2.
        Mat3 Sigma_qq = ZZ / (n_pts * n_pts);

        // 5) Assemble plane_cov = [ Σ_nn, Σ_nq; Σ_nq^T, Σ_qq ].
        plane_cov.setZero();
        plane_cov.block<3, 3>(0, 0) = Sigma_nn;
        plane_cov.block<3, 3>(0, 3) = Sigma_nq;
        plane_cov.block<3, 3>(3, 0) = Sigma_nq.transpose();
        plane_cov.block<3, 3>(3, 3) = Sigma_qq;

        return true;
    }
};

// -----------------------------------------------------------------------------
// @brief  Compute LiDAR point measurement covariance in the body frame.
//
//         This implements the mathematical model from Eq.(1) of:
//
//         "Yuan C, Xu W, Liu X, et al. Efficient and probabilistic adaptive voxel mapping for accurate online lidar
//         odometry[J]. IEEE Robotics and Automation Letters, 2022, 7(3): 8518-8525."
// -----------------------------------------------------------------------------
inline float range_noise = 0.1f;
inline float degree_noise = 0.04f;
inline void calculatePointMeasureCov(const Eigen::Vector3d& point_lidar, Eigen::Matrix3d& cov_lidar) {
    double x = point_lidar.x();
    double y = point_lidar.y();
    double z = point_lidar.z();
    double r2 = x * x + y * y + z * z;
    double d = std::sqrt(r2);
    double inv_d = 1.0 / d;

    double ox = x * inv_d;
    double oy = y * inv_d;
    double oz = z * inv_d;

    double rx, ry, rz;
    if (std::abs(oz) < 0.9) {
        rx = 0.0;
        ry = 0.0;
        rz = 1.0;
    } else {
        rx = 1.0;
        ry = 0.0;
        rz = 0.0;
    }

    double dot_or = ox * rx + oy * ry + oz * rz;
    double b1x = rx - dot_or * ox;
    double b1y = ry - dot_or * oy;
    double b1z = rz - dot_or * oz;

    double n1 = std::sqrt(b1x * b1x + b1y * b1y + b1z * b1z);
    double inv_n1 = 1.0 / n1;
    b1x *= inv_n1;
    b1y *= inv_n1;
    b1z *= inv_n1;

    double b2x = oy * b1z - oz * b1y;
    double b2y = oz * b1x - ox * b1z;
    double b2z = ox * b1y - oy * b1x;

    double n2 = std::sqrt(b2x * b2x + b2y * b2y + b2z * b2z);
    double inv_n2 = 1.0 / n2;
    b2x *= inv_n2;
    b2y *= inv_n2;
    b2z *= inv_n2;

    double a0x = ox;
    double a0y = oy;
    double a0z = oz;

    double c1x = oy * b1z - oz * b1y;
    double c1y = oz * b1x - ox * b1z;
    double c1z = ox * b1y - oy * b1x;

    double c2x = oy * b2z - oz * b2y;
    double c2y = oz * b2x - ox * b2z;
    double c2z = ox * b2y - oy * b2x;

    double md = -d;

    double a1x = md * c1x;
    double a1y = md * c1y;
    double a1z = md * c1z;

    double a2x = md * c2x;
    double a2y = md * c2y;
    double a2z = md * c2z;

    double sigma_d2 = range_noise * range_noise;

    double pi = static_cast<double>(M_PI);
    double ang_std_rad = degree_noise * pi / double(180.0);
    double sigma_omega2 = ang_std_rad * ang_std_rad;

    double c00 = sigma_d2 * (a0x * a0x) + sigma_omega2 * (a1x * a1x + a2x * a2x);
    double c01 = sigma_d2 * (a0x * a0y) + sigma_omega2 * (a1x * a1y + a2x * a2y);
    double c02 = sigma_d2 * (a0x * a0z) + sigma_omega2 * (a1x * a1z + a2x * a2z);

    double c11 = sigma_d2 * (a0y * a0y) + sigma_omega2 * (a1y * a1y + a2y * a2y);
    double c12 = sigma_d2 * (a0y * a0z) + sigma_omega2 * (a1y * a1z + a2y * a2z);

    double c22 = sigma_d2 * (a0z * a0z) + sigma_omega2 * (a1z * a1z + a2z * a2z);

    cov_lidar(0, 0) = c00;
    cov_lidar(0, 1) = c01;
    cov_lidar(0, 2) = c02;

    cov_lidar(1, 0) = c01;
    cov_lidar(1, 1) = c11;
    cov_lidar(1, 2) = c12;

    cov_lidar(2, 0) = c02;
    cov_lidar(2, 1) = c12;
    cov_lidar(2, 2) = c22;
}
}  // namespace VoxelMapUtils
}  // namespace legkilo
#endif  // LEG_KILO_VOXEL_MAP_UTILS_H