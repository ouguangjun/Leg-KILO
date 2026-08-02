// SPDX-License-Identifier: MIT
// @file factor_graph.h
// @brief Factor graph representation and manipulation
// @author Ou Guangjun
// @created 2026-03-04
// @maintainer ouguangjun98@gmail.com

#ifndef LEGKILO_FACTOR_GRAPH_H
#define LEGKILO_FACTOR_GRAPH_H

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/math_utils.hpp"
#include "core/slam/backend/factor_graph/ceres_factor.h"
#include "core/slam/backend/factor_graph/graph_utils.h"

namespace legkilo {

class FactorGraph {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using NodeType = int64_t;

    FactorGraph(const std::string& yaml_file);

    ~FactorGraph();

    void addNode(NodeType id, const Eigen::Isometry3d& initial_pose);

    void addLoopClosureEdge(NodeType i, NodeType j, const Eigen::Isometry3d& meas);

    void optimize();

    bool getPose(NodeType node_id, Eigen::Isometry3d& pose) const;

   private:
    void addRelativeEdge(NodeType i, NodeType j, const Eigen::Isometry3d& meas, double trans_sigma,
                         double rot_sigma_deg, ceres::LossFunction* loss_function = nullptr);

    void addOdometryEdgesForNewestNode();

    std::map<NodeType, graph::PoseParam> poses_;  // optimized poses

    std::map<NodeType, Eigen::Isometry3d> initial_poses_;  // fixed

    std::unordered_map<graph::EdgeType, graph::LoopEdge, graph::EdgeType::Hasher> loop_edges_;
    std::vector<NodeType> node_ids_;
    std::unique_ptr<ceres::Problem> problem_;

    // parameters for optimization
    size_t max_iterations_ = 100;
    double odom_near_trans_sigma_ = 0.1;
    double odom_near_rot_sigma_deg_ = 5.0;
    double odom_next_trans_sigma_multiplier_ = 2.0;
    double odom_next_rot_sigma_multiplier_ = 2.0;
    double loopclosure_trans_sigma_multiplier_ = 10.0;
    double loopclosure_rot_sigma_multiplier_ = 4.0;
    double loopclosure_loss_scale_ = 3.0;
    ceres::LinearSolverType linear_solver_type_ = ceres::SPARSE_NORMAL_CHOLESKY;
};

}  // namespace legkilo
#endif  // LEGKILO_FACTOR_GRAPH_H
