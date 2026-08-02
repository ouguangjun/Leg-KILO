// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/ann/traits.hpp>
#include <small_gicp/points/traits.hpp>

namespace small_gicp {

/// @brief Point-to-point per-point error factor.
struct ICPFactor {
  struct Setting {};

  ICPFactor(const Setting& setting = Setting())
  : target_index(std::numeric_limits<size_t>::max()),
    source_index(std::numeric_limits<size_t>::max()),
    has_correspondence(false),
    corr_sq_dist(std::numeric_limits<double>::infinity()) {}

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree, typename CorrespondenceRejector>
  bool linearize(
    const TargetPointCloud& target,
    const SourcePointCloud& source,
    const TargetTree& target_tree,
    const Eigen::Isometry3d& T,
    size_t source_index,
    const CorrespondenceRejector& rejector,
    Eigen::Matrix<double, 6, 6>* H,
    Eigen::Matrix<double, 6, 1>* b,
    double* e) {
    //
    this->source_index = source_index;
    this->target_index = std::numeric_limits<size_t>::max();
    this->has_correspondence = false;
    this->corr_sq_dist = std::numeric_limits<double>::infinity();

    const Eigen::Vector4d transed_source_pt = T * traits::point(source, source_index);

    size_t k_index;
    double k_sq_dist;
    if (!traits::nearest_neighbor_search(target_tree, transed_source_pt, &k_index, &k_sq_dist)) {
      return false;
    }

    has_correspondence = true;
    corr_sq_dist = k_sq_dist;

    if (rejector(target, source, T, k_index, source_index, k_sq_dist)) {
      return false;
    }

    target_index = k_index;
    const Eigen::Vector4d residual = traits::point(target, target_index) - transed_source_pt;

    Eigen::Matrix<double, 4, 6> J = Eigen::Matrix<double, 4, 6>::Zero();
    J.block<3, 3>(0, 0) = T.linear() * skew(traits::point(source, source_index).template head<3>());
    J.block<3, 3>(0, 3) = -T.linear();

    *H = J.transpose() * J;
    *b = J.transpose() * residual;
    *e = 0.5 * residual.squaredNorm();

    return true;
  }

  template <typename TargetPointCloud, typename SourcePointCloud>
  double error(const TargetPointCloud& target, const SourcePointCloud& source, const Eigen::Isometry3d& T) const {
    if (target_index == std::numeric_limits<size_t>::max()) {
      return 0.0;
    }

    const Eigen::Vector4d residual = traits::point(target, target_index) - T * traits::point(source, source_index);
    return 0.5 * residual.squaredNorm();
  }

  bool inlier() const { return target_index != std::numeric_limits<size_t>::max(); }
  bool stat_inlier(double max_dist_sq) const { return has_correspondence && corr_sq_dist <= max_dist_sq; }

  size_t target_index;
  size_t source_index;
  bool has_correspondence;
  double corr_sq_dist;
};
}  // namespace small_gicp
