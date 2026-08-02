// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#define _USE_MATH_DEFINES

#include <cmath>

#include <robin/core.hpp>
#include <robin/graph.hpp>
#include <robin/math.hpp>
#include <robin/utils.hpp>
#include <utility>

/// This file contains the definitions of a collection of functions that can be used with robin to
/// reject outliers

namespace robin {

//
// Single Vector Averaging (SVA)
//

/**
 * @brief Set of R^n points
 */
class VectorY : public robin::SetBase<Eigen::VectorXd> {
public:
  VectorY() = delete;
  explicit VectorY(Eigen::MatrixXd y_set) : y_set_(std::move(y_set)) {}
  Eigen::VectorXd operator[](size_t i) const override { return y_set_.col(i); }
  size_t size() const override { return y_set_.cols(); }

private:
  Eigen::MatrixXd y_set_;
};

/**
 * @brief Euclidean distance function
 */
struct EuclideanDist {
  double operator()(const Eigen::VectorXd& a, const Eigen::VectorXd& b) { return (a - b).norm(); }
  double operator()(const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return (a - b).norm(); }
};

/**
 * @brief Comparison functions for scalar distances
 */
struct ScalarDistComp {
  explicit ScalarDistComp(double thres) : noise_threshold(thres) {}
  bool operator()(const double& a, const double& b) { return std::abs(a - b) <= noise_threshold; }
  double noise_threshold = 0;
};

/**
 * Compatibility check function for Single Vector Averaging (SVA)
 */
struct SvaCompCheck {
  explicit SvaCompCheck(double thres) : noise_threshold(thres) {}
  bool operator()(VectorY* measurements, const size_t* subset_indices) const {
    return ((*measurements)[subset_indices[0]] - (*measurements)[subset_indices[1]]).norm() <=
           noise_threshold;
  }
  double noise_threshold = 0;
};

//
// Single Rotation Averaging (SRA)
//

/**
 * @brief Set of SO(3) points
 */
class So3Y : public robin::SetBase<Eigen::Matrix3d> {
public:
  So3Y() = delete;
  explicit So3Y(std::vector<Eigen::Matrix3d> y_set) : y_set_(std::move(y_set)) {}
  Eigen::Matrix3d operator[](size_t i) const override { return y_set_[i]; }
  size_t size() const override { return y_set_.size(); }

private:
  std::vector<Eigen::Matrix3d> y_set_;
};

/**
 * @brief Functor for SO(3) geodesic distance
 */
struct So3GeodesicDist {
  double operator()(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    assert(IsSo3(a));
    assert(IsSo3(b));
    Eigen::Matrix3d A = a.transpose() * b;
    return std::abs(std::acos((A.trace() - 1) / 2));
  }
};

/**
 * @brief Functor for SO(3) chordal distance
 */
struct So3ChordalDist {
  double operator()(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    assert(IsSo3(a));
    assert(IsSo3(b));
    Eigen::Matrix3d a_m_b = a - b;
    double F_norm = std::sqrt((a_m_b * a_m_b.transpose()).trace());
    return F_norm;
  }
};

/**
 * Compatibility check function for Single Rotation Averaging (SRA)
 * @tparam DistFn a distance function type for SO(3)
 */
template <typename DistFn> struct SraCompCheck {
  /**
   * Constructor that takes in a distance function pointer and a noise threshold
   * @param fn
   * @param noise_threshold
   */
  explicit SraCompCheck(DistFn* fn, double noise_threshold)
      : dist_fn(fn), noise_threshold(noise_threshold) {}

  /**
   * operator() for calculating compatibility checks
   * @param measurements
   * @param subset_indices
   * @return
   */
  bool operator()(So3Y* measurements, const size_t* subset_indices) {
    double dist =
        (*dist_fn)((*measurements)[subset_indices[0]], (*measurements)[subset_indices[1]]);
    return dist <= noise_threshold;
  }

  DistFn* dist_fn = nullptr;
  double noise_threshold = 0;
};

//
// 3D Registration (3DREG)
//

/**
 * @brief 3D point cloud set
 */
class Points3d : public robin::SetBase<Eigen::Vector3d> {
public:
  Points3d() = delete;
  explicit Points3d(Eigen::Matrix3Xd y_set) : y_set_(std::move(y_set)) {}
  Eigen::Vector3d operator[](size_t i) const override { return y_set_.col(i); }
  size_t size() const override { return y_set_.cols(); }

private:
  Eigen::Matrix3Xd y_set_;
};

/**
 * Compatibility function for 3D points registration with correspondences
 *
 * Uses the Euclidean distance as the invariant function
 */
struct Points3dRegCompCheck {
  Points3dRegCompCheck(Points3d* model, double thres) : model(model), noise_threshold(thres) {}

  bool operator()(Points3d* measurements, const size_t* subset_indices) const {
    double dist_1 =
        ((*measurements)[subset_indices[0]] - (*measurements)[subset_indices[1]]).norm();
    double dist_2 = ((*model)[subset_indices[0]] - (*model)[subset_indices[1]]).norm();
    return std::abs(dist_1 - dist_2) <= noise_threshold;
  }

  Points3d* model = nullptr;
  double noise_threshold = 0;
};

//
// 3D Points + Normals Registration
//
/// Type alias for a 6-by-1 vector
using Point3dWithNormal = Eigen::Matrix<double, 6, 1>;

/**
 * @brief Set of points with normals.
 *
 * Each point with normal is a 6-by-1 Eigen vector, with the top 3 entries representing
 * the 3D point and bottom 3 entries representing the normals.
 */
class Points3dWithNormals : public robin::SetBase<Point3dWithNormal> {

public:
  Points3dWithNormals() = delete;
  explicit Points3dWithNormals(Eigen::Matrix<double, 6, Eigen::Dynamic> y_set)
      : y_set_(std::move(y_set)) {}
  Point3dWithNormal operator[](size_t i) const override { return y_set_.col(i); }
  size_t size() const override { return y_set_.cols(); }

private:
  Eigen::Matrix<double, 6, Eigen::Dynamic> y_set_;
};

/**
 * @brief Compatibility function for 3D points+normals registration with correspondences
 */
struct PointsNormals3dRegCompCheck {
  /**
   * Constructor for this compatibility check functor
   * @param model
   * @param point_noise_threshold
   * @param normal_noise_threshold threshold for normal noise, equals to cos(2*beta)
   */
  PointsNormals3dRegCompCheck(Points3dWithNormals* model, double point_noise_threshold,
                              double normal_noise_threshold)
      : model(model), point_noise_threshold(point_noise_threshold),
        normal_noise_threshold(normal_noise_threshold) {}

  /**
   * For use in the CompGraphConstructor
   * @param measurements
   * @param subset_indices a subset of indices to test on (in this case, it should be of length 2)
   * @return
   */
  bool operator()(Points3dWithNormals* measurements, const size_t* subset_indices) const {
    // distances on the measurement side
    double measurement_point_dist =
        robin::EuclideanDistance((*measurements)[subset_indices[0]].topRows<3>(),
                                 (*measurements)[subset_indices[1]].topRows<3>());
    double measurement_normal_dist =
        robin::CosineSimilarity((*measurements)[subset_indices[0]].bottomRows<3>(),
                                (*measurements)[subset_indices[1]].bottomRows<3>());

    // distances on the model side
    double model_point_dist = robin::EuclideanDistance((*model)[subset_indices[0]].topRows<3>(),
                                                       (*model)[subset_indices[1]].topRows<3>());
    double model_normal_dist = robin::CosineSimilarity((*model)[subset_indices[0]].bottomRows<3>(),
                                                       (*model)[subset_indices[1]].bottomRows<3>());

    // check compatibility
    double point_dist_diff = std::abs(measurement_point_dist - model_point_dist);
    if (point_dist_diff > point_noise_threshold) {
      return false;
    }
    // using the distance comparison equation that does not involve arccos
    // double lhs = 2 * noise_threshold(1) * a(1) * b(1) + 1 - a(1) * a(1) - b(1) * b(1);
    const double& ca = measurement_normal_dist;
    const double& cb = model_normal_dist;
    double lhs = ca * cb + std::sqrt((1 - ca * ca) * (1 - cb * cb));
    bool normal_consistency = lhs >= normal_noise_threshold;
    return normal_consistency;
  }

  Points3dWithNormals* model = nullptr;
  double point_noise_threshold = 0;
  double normal_noise_threshold = 0;
};

/**
 * @brief Compare the distance between two vectors with a vector threshold
 * @deprecated
 */
struct VectorDistComp {
  explicit VectorDistComp(Eigen::VectorXd thres) : noise_threshold(std::move(thres)) {}
  bool operator()(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    assert(a.size() == b.size());
    assert(noise_threshold.size() == a.size());
    Eigen::VectorXd diff = (a - b).cwiseAbs();
    for (size_t i = 0; i < noise_threshold.size(); ++i) {
      if (diff(i) > noise_threshold(i)) {
        return false;
      }
    }
    return true;
  }
  Eigen::VectorXd noise_threshold;
};

/**
 * @brief Compare the distances between two 2D vectors returned by Points3dWithNormalsDist
 * @deprecated
 */
struct Vector2dDistComp {
  explicit Vector2dDistComp(Eigen::Vector2d thres) : noise_threshold(std::move(thres)) {}
  bool operator()(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    Eigen::Vector2d diff = (a - b).cwiseAbs();
    if (diff(0) > noise_threshold(0) || diff(1) > noise_threshold(1)) {
      return false;
    }
    return true;
  }
  Eigen::Vector2d noise_threshold;
};

/**
 * @brief Distance function for 3D points with a normal
 * @deprecated
 */
struct Points3dWithNormalsAngularDist {
  Eigen::Vector2d operator()(const Point3dWithNormal& a, const Point3dWithNormal& b) {
    // 3d point dists
    double point_dist = (a.topRows<3>() - b.topRows<3>()).norm();

    // directional dists
    double normal_diff_angle = a.bottomRows<3>().dot(b.bottomRows<3>()) /
                               (a.bottomRows<3>().norm() * b.bottomRows<3>().norm());
    if (normal_diff_angle > 1.0) {
      normal_diff_angle = 0;
    } else if (normal_diff_angle < -1.0) {
      normal_diff_angle = M_PI;
    } else {
      normal_diff_angle = std::acos(normal_diff_angle);
    }

    Eigen::Vector2d vec_dists;
    vec_dists << point_dist, normal_diff_angle;
    return vec_dists;
  }
};

/**
 * @brief Distance function for 3D points with a normal
 * @deprecated
 */
struct Points3dWithNondirectionalNormalsDist {
  Eigen::Vector2d operator()(const Point3dWithNormal& a, const Point3dWithNormal& b) {
    // 3d point dists
    double point_dist = (a.topRows<3>() - b.topRows<3>()).norm();

    // directional dists
    double normal_diff_angle = a.bottomRows<3>().dot(b.bottomRows<3>()) /
                               (a.bottomRows<3>().norm() * b.bottomRows<3>().norm());
    if (normal_diff_angle > 1.0) {
      normal_diff_angle = 0;
    } else if (normal_diff_angle < -1.0) {
      normal_diff_angle = M_PI;
    } else {
      normal_diff_angle = std::acos(normal_diff_angle);
    }
    normal_diff_angle = std::min(normal_diff_angle, M_PI - normal_diff_angle);

    Eigen::Vector2d vec_dists;
    vec_dists << point_dist, normal_diff_angle;
    return vec_dists;
  }
};

} // namespace robin
