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

// Matrix types
using Mat2D = Eigen::Matrix2d;
using Mat3D = Eigen::Matrix3d;
using Mat4D = Eigen::Matrix4d;
using Mat6D = Eigen::Matrix<double, 6, 6>;

using Mat2F = Eigen::Matrix2f;
using Mat3F = Eigen::Matrix3f;
using Mat4F = Eigen::Matrix4f;

}  // namespace legkilo

#endif  // LEG_KILO_EIGEN_TYPES_H