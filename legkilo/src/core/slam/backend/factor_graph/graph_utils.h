// SPDX-License-Identifier: MIT
// @file graph_utils.h
// @brief Graph utility functions and structures
// @author Ou Guangjun
// @created 2026-03-04
// @maintainer ouguangjun98@gmail.com

#ifndef LEGKILO_GRAPH_UTILS_H
#define LEGKILO_GRAPH_UTILS_H

#include "common/math_utils.hpp"

#include <ceres/ceres.h>
#if (CERES_VERSION_MAJOR >= 2)
#include <ceres/manifold.h>
#else
#include <ceres/local_parameterization.h>
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace legkilo {
namespace graph {

struct PoseParam {
    double t[3]{0.0, 0.0, 0.0};
    double q[4]{0.0, 0.0, 0.0, 1.0};  // xyzw

    PoseParam() = default;

    PoseParam(const Eigen::Isometry3d& iso) { this->set(iso.translation(), Eigen::Quaterniond(iso.rotation())); }

    Eigen::Vector3d tEigen() const { return {t[0], t[1], t[2]}; }

    Eigen::Quaterniond qEigen() const { return Eigen::Quaterniond(q[3], q[0], q[1], q[2]); }  // wxyz

    void set(const Eigen::Vector3d& tt, const Eigen::Quaterniond& qq) {
        t[0] = tt.x();
        t[1] = tt.y();
        t[2] = tt.z();
        q[0] = qq.x();
        q[1] = qq.y();
        q[2] = qq.z();
        q[3] = qq.w();
    }

    Eigen::Isometry3d toIsometry() const {
        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        iso.translate(this->tEigen());
        iso.rotate(this->qEigen());
        return iso;
    }
};

using EdgeType = UnorderedIntPairKey;
using NodeType = int64_t;

struct LoopEdge {
    NodeType i, j;
    Eigen::Isometry3d meas;  // relative pose measurement from i to j

    LoopEdge(NodeType ii, NodeType jj, const Eigen::Isometry3d& m) : i(ii), j(jj), meas(m) {}
};

class QuaternionUpdateRule {
   public:
    static void Attach(ceres::Problem& problem, double* q_xyzw) {
#if (CERES_VERSION_MAJOR >= 2)
        problem.SetManifold(q_xyzw, new ceres::EigenQuaternionManifold());
#else
        problem.SetParameterization(q_xyzw, new ceres::EigenQuaternionParameterization());
#endif
    }
};

}  // namespace graph
}  // namespace legkilo
#endif  // LEGKILO_GRAPH_UTILS_H
