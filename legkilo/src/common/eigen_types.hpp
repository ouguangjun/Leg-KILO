#ifndef LEG_KILO_EIGEN_TYPES_H
#define LEG_KILO_EIGEN_TYPES_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace legkilo {

// Vector types
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

// Matrix types
using Mat2D = Eigen::Matrix2d;
using Mat3D = Eigen::Matrix3d;
using Mat4D = Eigen::Matrix4d;
using Mat6D = Eigen::Matrix<double, 6, 6>;

using Mat2F = Eigen::Matrix2f;
using Mat3F = Eigen::Matrix3f;
using Mat4F = Eigen::Matrix4f;
using Mat6F = Eigen::Matrix<float, 6, 6>;

/// GaoXiang: Slam In Autonomous Driving
/// https://github.com/gaoxiang12/slam_in_autonomous_driving/blob/master/src/common/eigen_types.h

/// 矢量比较
template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

/// 矢量哈希
template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

// 矢量相等（用于 unordered_map 的 Key）
template <int N>
struct equal_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const {
        for (int i = 0; i < N; ++i) {
            if (v1[i] != v2[i]) return false;
        }
        return true;
    }
};

// 实现2D和3D的比较
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

/// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
}

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}

constexpr auto less_vec2i = [](const Vec2i& v1, const Vec2i& v2) {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
};

// Discretize a 3D point into voxel key by floor
inline Eigen::Vector3i voxelKeyFloor(const Eigen::Vector3d &p_world, double voxel_size) {
    Eigen::Vector3i key;
    key[0] = static_cast<int>(std::floor(p_world[0] / voxel_size));
    key[1] = static_cast<int>(std::floor(p_world[1] / voxel_size));
    key[2] = static_cast<int>(std::floor(p_world[2] / voxel_size));
    return key;
}
}  // namespace legkilo
#endif  // LEG_KILO_EIGEN_TYPES_H
