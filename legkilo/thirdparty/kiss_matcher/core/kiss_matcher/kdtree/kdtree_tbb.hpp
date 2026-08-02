// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <Eigen/Core>
#include <kiss_matcher/kdtree/kdtree.hpp>
#include <kiss_matcher/kdtree/nanoflann_tbb.hpp>
#include <kiss_matcher/points/traits.hpp>

namespace kiss_matcher {

/// @brief Unsafe KdTree with multi-thread tree construction with TBB backend.
/// @note  This class only parallelizes the tree construction. The search is still single-threaded
/// as in the normal KdTree.
template <class PointCloud>
using UnsafeKdTreeTBB = UnsafeKdTreeGeneric<PointCloud, kiss_matcher::KDTreeSingleIndexAdaptorTBB>;

/// @brief KdTree with multi-thread tree construction with TBB backend.
/// @note  This class only parallelizes the tree construction. The search is still single-threaded
/// as in the normal KdTree.
template <class PointCloud>
using KdTreeTBB = KdTreeGeneric<PointCloud, kiss_matcher::KDTreeSingleIndexAdaptorTBB>;

}  // namespace kiss_matcher
