// SPDX-License-Identifier: MIT
// @file euclidean_method.h
// @brief Simple loop candidates by Euclidean distance and id separation
// @author Ou

#ifndef LEGKILO_LOOP_CLOSURE_LOOP_DETECTOR_EUCLIDEAN_METHOD_H
#define LEGKILO_LOOP_CLOSURE_LOOP_DETECTOR_EUCLIDEAN_METHOD_H

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace legkilo {

using NodeType = int64_t;

class EuclideanLoopDetector {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EuclideanLoopDetector() = default;

    void setSearchRadius(double r);

    void setMinIdSeparation(int s);

    std::vector<std::pair<NodeType, NodeType>> detectLatest(
        const std::vector<std::pair<NodeType, Eigen::Isometry3d>>& poses) const;

   private:
    double search_radius_ = 5.0;
    int min_id_separation_ = 5;
};

}  // namespace legkilo

#endif  // LEGKILO_LOOP_CLOSURE_LOOP_DETECTOR_EUCLIDEAN_METHOD_H
