// SPDX-License-Identifier: MIT
// @file math_utils.hpp
// @brief SO(3) utilities, skew/exponential maps and helpers.
// @author Ou Guangjun
// @created 2025-01-18
// @maintainer ouguangjun98@gmail.com
#ifndef LEG_KILO_MATH_UTILS_H
#define LEG_KILO_MATH_UTILS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace legkilo {

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

using Vec2D = Eigen::Vector2d;
using Vec3D = Eigen::Vector3d;
using Vec4D = Eigen::Vector4d;
using Vec6D = Eigen::Matrix<double, 6, 1>;

using Vec2F = Eigen::Vector2f;
using Vec3F = Eigen::Vector3f;
using Vec4F = Eigen::Vector4f;
using Vec6F = Eigen::Matrix<float, 6, 1>;

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec4i = Eigen::Vector4i;
using Vec6i = Eigen::Matrix<int, 6, 1>;

using Mat2D = Eigen::Matrix2d;
using Mat3D = Eigen::Matrix3d;
using Mat4D = Eigen::Matrix4d;
using Mat6D = Eigen::Matrix<double, 6, 6>;

using Mat2F = Eigen::Matrix2f;
using Mat3F = Eigen::Matrix3f;
using Mat4F = Eigen::Matrix4f;
using Mat6F = Eigen::Matrix<float, 6, 6>;

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SKEW_SYM_MATRIX(const Eigen::MatrixBase<Derived> &v) {
    using T = typename Derived::Scalar;
    Eigen::Matrix<T, 3, 3> m;
    m << T(0), -v(2), v(1), v(2), T(0), -v(0), -v(1), v(0), T(0);
    return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang) {
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T, typename Ts>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K = SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3) {
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001) {
        Eigen::Matrix<T, 3, 1> r_ang(v1 / norm, v2 / norm, v3 / norm);
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    } else {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
inline Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R) {
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

inline double gaussianPdf1D(double x, double sigma) {
    constexpr double kMinSigma = 1e-4;
    constexpr double kInvSqrt2Pi = 0.3989422804014327;
    if (sigma <= kMinSigma) { sigma = kMinSigma; }
    double inv_std = 1.0 / sigma;
    double z = x * inv_std;
    return kInvSqrt2Pi * inv_std * std::exp(-0.5 * z * z);
}

// Discretize a 3D point into voxel key by floor
inline Eigen::Vector3i voxelKeyFloor(const Eigen::Vector3d &pt, double voxel_size) {
    Eigen::Vector3i key;
    key[0] = static_cast<int>(std::floor(pt[0] / voxel_size));
    key[1] = static_cast<int>(std::floor(pt[1] / voxel_size));
    key[2] = static_cast<int>(std::floor(pt[2] / voxel_size));
    return key;
}

template <typename T>
inline int fast_floor(T x) {
    int i = static_cast<int>(x);
    return (x < i) ? (i - 1) : i;
}

inline Eigen::Vector3i voxelKeyFastFloor(const Eigen::Vector3d &pt, double voxel_inv_size) {
    return Eigen::Vector3i(fast_floor(pt[0] * voxel_inv_size), fast_floor(pt[1] * voxel_inv_size),
                           fast_floor(pt[2] * voxel_inv_size));
}

static constexpr int64_t VOXEL_BIAS = 1ll << 20;  // 2^20 = 1048576

inline int64_t encodeKey(const Eigen::Vector3i &key) {
    assert(key[0] >= -VOXEL_BIAS && key[0] < VOXEL_BIAS);
    assert(key[1] >= -VOXEL_BIAS && key[1] < VOXEL_BIAS);
    assert(key[2] >= -VOXEL_BIAS && key[2] < VOXEL_BIAS);
    return (static_cast<int64_t>(key[0] + VOXEL_BIAS) & 0x1FFFFFll) |
           ((static_cast<int64_t>(key[1] + VOXEL_BIAS) & 0x1FFFFFll) << 21) |
           ((static_cast<int64_t>(key[2] + VOXEL_BIAS) & 0x1FFFFFll) << 42);
}

inline Eigen::Vector3i decodeKey(int64_t code) {
    Eigen::Vector3i key;
    key[0] = static_cast<int>((code & 0x1FFFFFll) - static_cast<int>(VOXEL_BIAS));
    key[1] = static_cast<int>(((code >> 21) & 0x1FFFFFll) - static_cast<int>(VOXEL_BIAS));
    key[2] = static_cast<int>(((code >> 42) & 0x1FFFFFll) - static_cast<int>(VOXEL_BIAS));
    return key;
}

inline Eigen::Isometry3d makeIsometry3d(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation) {
    Eigen::Isometry3d iso_transform = Eigen::Isometry3d::Identity();
    iso_transform.translation() = translation;
    iso_transform.linear() = rotation;
    return iso_transform;
}

class UnorderedIntPairKey {
   public:
    using value_type = std::int64_t;

    constexpr UnorderedIntPairKey() noexcept : a_(0), b_(0) {}

    constexpr UnorderedIntPairKey(value_type x, value_type y) noexcept : a_(x), b_(y) { normalize_(); }

    constexpr value_type first() const noexcept { return a_; }
    constexpr value_type second() const noexcept { return b_; }

    constexpr std::pair<value_type, value_type> asPair() const noexcept { return {a_, b_}; }

    friend constexpr bool operator<(const UnorderedIntPairKey &lhs, const UnorderedIntPairKey &rhs) noexcept {
        return (lhs.a_ < rhs.a_) || (lhs.a_ == rhs.a_ && lhs.b_ < rhs.b_);
    }

    friend constexpr bool operator==(const UnorderedIntPairKey &lhs, const UnorderedIntPairKey &rhs) noexcept {
        return lhs.a_ == rhs.a_ && lhs.b_ == rhs.b_;
    }

    friend constexpr bool operator!=(const UnorderedIntPairKey &lhs, const UnorderedIntPairKey &rhs) noexcept {
        return !(lhs == rhs);
    }
    struct Hasher {
        std::size_t operator()(const UnorderedIntPairKey &k) const noexcept {
            std::size_t h1 = std::hash<value_type>{}(k.first());
            std::size_t h2 = std::hash<value_type>{}(k.second());
            return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
        }
    };

    struct Equal {
        constexpr bool operator()(const UnorderedIntPairKey &lhs, const UnorderedIntPairKey &rhs) const noexcept {
            return lhs == rhs;
        }
    };

   private:
    value_type a_, b_;

    constexpr void normalize_() noexcept {
        if (b_ < a_) std::swap(a_, b_);
    }
};

}  // namespace legkilo
#endif  // LEG_KILO_MATH_UTILS_H