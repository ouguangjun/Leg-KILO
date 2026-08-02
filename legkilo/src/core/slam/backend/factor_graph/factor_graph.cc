#include "core/slam/backend/factor_graph/factor_graph.h"

#include <algorithm>
#include <cmath>

#include <glog/logging.h>

#include "common/yaml_helper.hpp"

namespace legkilo {

namespace {

Eigen::Matrix<double, 6, 6> MakeSqrtInformation(double trans_sigma, double rot_sigma_deg) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

    const double safe_trans_sigma = trans_sigma;
    const double safe_rot_sigma = rot_sigma_deg * kDegToRad;

    Eigen::Matrix<double, 6, 6> sqrt_information = Eigen::Matrix<double, 6, 6>::Zero();
    sqrt_information.diagonal().head<3>().setConstant(1.0 / safe_trans_sigma);
    sqrt_information.diagonal().tail<3>().setConstant(1.0 / safe_rot_sigma);
    return sqrt_information;
}

}  // namespace

FactorGraph::FactorGraph(const std::string &yaml_file) : problem_(std::make_unique<ceres::Problem>()) {
    YamlHelper yaml(yaml_file);
    max_iterations_ = yaml.get<size_t>("factor_graph_max_iterations", 100);
    odom_near_trans_sigma_ = yaml.get<double>("odom_near_trans_sigma", 0.1);
    odom_near_rot_sigma_deg_ = yaml.get<double>("odom_near_rot_sigma_deg", 5.0);
    odom_next_trans_sigma_multiplier_ = yaml.get<double>("odom_next_trans_sigma_multiplier", 2.0);
    odom_next_rot_sigma_multiplier_ = yaml.get<double>("odom_next_rot_sigma_multiplier", 2.0);
    loopclosure_trans_sigma_multiplier_ = yaml.get<double>("loopclosure_trans_sigma_multiplier", 10.0);
    loopclosure_rot_sigma_multiplier_ = yaml.get<double>("loopclosure_rot_sigma_multiplier", 4.0);
    loopclosure_loss_scale_ = yaml.get<double>("loopclosure_loss_scale", 3.0);
    LOG(INFO) << "FactorGraph is constructed";
}

FactorGraph::~FactorGraph() { LOG(INFO) << "FactorGraph is destructed"; }

void FactorGraph::addNode(NodeType id, const Eigen::Isometry3d &initial_pose) {
    if (initial_poses_.find(id) != initial_poses_.end()) return;
    initial_poses_.insert({id, initial_pose});
    node_ids_.push_back(id);

    auto it = poses_.emplace(id, graph::PoseParam(initial_pose)).first;
    graph::PoseParam &node = it->second;
    problem_->AddParameterBlock(node.t, 3);
    problem_->AddParameterBlock(node.q, 4);
    graph::QuaternionUpdateRule::Attach(*problem_, node.q);

    if (node_ids_.size() == 1) {
        problem_->SetParameterBlockConstant(node.t);
        problem_->SetParameterBlockConstant(node.q);
    }

    addOdometryEdgesForNewestNode();
}

void FactorGraph::addLoopClosureEdge(NodeType i, NodeType j, const Eigen::Isometry3d &meas) {
    if (loop_edges_.find({i, j}) != loop_edges_.end()) return;
    if (initial_poses_.find(i) == initial_poses_.end() || initial_poses_.find(j) == initial_poses_.end()) return;
    LOG(INFO) << "Adding loop closure edge between " << i << " and " << j;
    loop_edges_.insert({{i, j}, graph::LoopEdge(i, j, meas)});
    addRelativeEdge(i, j, meas, odom_near_trans_sigma_ * loopclosure_trans_sigma_multiplier_,
                    odom_near_rot_sigma_deg_ * loopclosure_rot_sigma_multiplier_,
                    new ceres::CauchyLoss(loopclosure_loss_scale_));
}

bool FactorGraph::getPose(NodeType node_id, Eigen::Isometry3d &pose) const {
    if (poses_.find(node_id) == poses_.end()) return false;
    pose = poses_.at(node_id).toIsometry();
    return true;
}

void FactorGraph::addRelativeEdge(NodeType i, NodeType j, const Eigen::Isometry3d &meas, double trans_sigma,
                                  double rot_sigma_deg, ceres::LossFunction *loss_function) {
    auto it_i = poses_.find(i);
    auto it_j = poses_.find(j);
    if (it_i == poses_.end() || it_j == poses_.end()) return;

    const Eigen::Matrix<double, 6, 6> sqrt_information = MakeSqrtInformation(trans_sigma, rot_sigma_deg);
    ceres::CostFunction *cost = graph::RelativePoseFactor::Create(meas, sqrt_information);
    problem_->AddResidualBlock(cost, loss_function, it_i->second.t, it_i->second.q, it_j->second.t, it_j->second.q);
}

void FactorGraph::addOdometryEdgesForNewestNode() {
    const size_t num_nodes = node_ids_.size();
    if (num_nodes < 2) return;

    const NodeType newest_id = node_ids_.back();
    const NodeType prev_id = node_ids_[num_nodes - 2];
    addRelativeEdge(prev_id, newest_id, initial_poses_.at(prev_id).inverse() * initial_poses_.at(newest_id),
                    odom_near_trans_sigma_, odom_near_rot_sigma_deg_);

    if (num_nodes >= 3) {
        const NodeType prev2_id = node_ids_[num_nodes - 3];
        addRelativeEdge(prev2_id, newest_id, initial_poses_.at(prev2_id).inverse() * initial_poses_.at(newest_id),
                        odom_near_trans_sigma_ * odom_next_trans_sigma_multiplier_,
                        odom_near_rot_sigma_deg_ * odom_next_rot_sigma_multiplier_);
    }
}

void FactorGraph::optimize() {
    if (poses_.empty()) return;

    ceres::Solver::Options options;
    options.max_num_iterations = static_cast<int>(max_iterations_);
    options.linear_solver_type = linear_solver_type_;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem_.get(), &summary);
    LOG(INFO) << "FactorGraph optimization done. Iterations: " << summary.iterations.size()
              << ", final cost: " << summary.final_cost;
}

}  // namespace legkilo
