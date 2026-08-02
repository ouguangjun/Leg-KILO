#include "core/slam/backend/loop_closure/loop_closure.h"

#include <cstdlib>

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

#include "common/timer_utils.hpp"
#include "common/yaml_helper.hpp"
#include "core/slam/backend/loop_closure/loop_detector/euclidean_method.h"
#include "kiss_matcher/KISSMatcher.hpp"
#include "small_gicp/ann/kdtree_tbb.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/points/point_cloud.hpp"
#include "small_gicp/registration/reduction_tbb.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/util/normal_estimation_tbb.hpp"

namespace legkilo {

LoopClosure::LoopClosure(const std::string& config_file) {
    stopping_.store(false, std::memory_order_release);

    YamlHelper yaml(config_file);

    search_radius_ = yaml.get<double>("loop_search_radius", 5.0);
    min_id_separation_ = yaml.get<int>("loop_min_id_separation", 5);
    icp_max_correspondence_dist_ = yaml.get<double>("loop_icp_max_corr_dist", 1.0);
    icp_inlier_dist_threshold_ = yaml.get<double>("loop_icp_inlier_dist_threshold", 0.5);
    icp_max_iterations_ = yaml.get<int>("loop_icp_max_iterations", 20);
    icp_source_inlier_ratio_threshold_ = yaml.get<double>("loop_icp_fitness_threshold", 0.7);
    icp_num_threads_ = yaml.get<int>("loop_icp_num_threads", 2);
    verify_max_attempts_ = yaml.get<int>("loop_verify_max_attempts", 2);
    loop_kiss_matcher_enabled_ = yaml.get<bool>("loop_kiss_matcher_enabled", false);
    kiss_inliers_threshold_ = yaml.get<size_t>("loop_kiss_inliers_threshold", 40);
    kiss_refine_min_id_gap_ = yaml.get<int>("loop_kiss_refine_min_id_gap", 50);
    kiss_use_quatro_ = yaml.get<bool>("loop_kiss_use_quatro", false);

    detector_ = std::make_unique<EuclideanLoopDetector>();
    detector_->setSearchRadius(search_radius_);
    detector_->setMinIdSeparation(min_id_separation_);

    const std::string res_folder = yaml.get<std::string>("temp_result_save_folder", "temp");
    submap_pcd_folder_ = std::string(ROOT_DIR) + "result/" + res_folder + "/";
}

LoopClosure::~LoopClosure() { this->stop(); }

void LoopClosure::start() { worker_ = std::thread(&LoopClosure::workerLoop, this); }

void LoopClosure::stop() {
    stopping_.store(true, std::memory_order_release);
    cv_.notify_one();
    if (worker_.joinable()) worker_.join();
}

void LoopClosure::insert(IDPosesPtr optimized_poses) {
    if (!optimized_poses || optimized_poses->empty()) return;

    {
        std::lock_guard<std::mutex> lk(mutex_queue_);
        trigger_queue_.push_back(optimized_poses);
    }

    cv_.notify_one();
}

std::vector<LoopClosureOutput> LoopClosure::fetchVerified() {
    std::vector<LoopClosureOutput> outs;
    std::lock_guard<std::mutex> lk(mutex_cand_);
    for (auto& kv : history_candidates_) {
        auto& c = kv.second;
        if (c.valid && !c.fetched) {
            outs.push_back({c.i, c.j, c.meas});
            c.fetched = true;
        }
    }
    return outs;
}

void LoopClosure::workerLoop() {
    while (true) {
        IDPosesPtr cur_input;

        {
            std::unique_lock<std::mutex> lk(mutex_queue_);
            cv_.wait(lk, [&] { return stopping_.load(std::memory_order_acquire) || !trigger_queue_.empty(); });
            if (!trigger_queue_.empty()) {
                if (trigger_queue_.size() > 1) {
                    LOG(WARNING) << "LoopClosure worker is falling behind! Queue size: " << trigger_queue_.size();
                }
                cur_input = trigger_queue_.back();
                trigger_queue_.clear();
            }
        }

        if (stopping_.load(std::memory_order_acquire)) break;
        if (!cur_input) continue;

        const auto detect_result = detector_->detectLatest(*cur_input);
        std::vector<Candidate> verified_candidates;

        {
            std::lock_guard<std::mutex> lk(mutex_cand_);
            for (const auto& ij : detect_result) {
                UnorderedIntPairKey key(ij.first, ij.second);
                if (history_candidates_.find(key) == history_candidates_.end()) {
                    Candidate c;
                    c.i = ij.first;
                    c.j = ij.second;
                    c.meas = cur_input->at(ij.first).second.inverse() *
                             cur_input->at(ij.second).second;  // initial guess from optimized poses
                    history_candidates_.insert({key, c});
                }
            }

            for (const auto& kv : history_candidates_) {
                const auto& c = kv.second;
                if (c.valid || c.attempts >= verify_max_attempts_) continue;
                verified_candidates.push_back(c);
            }
        }

        if (verified_candidates.empty()) continue;

        LOG(INFO) << "Verifying candidate number: " << verified_candidates.size();

        for (auto& c : verified_candidates) {
            Timer::measure("LoopClosure::verify", [&]() { c.valid = this->verify(c.i, c.j, c.meas); });
        }

        {
            std::lock_guard<std::mutex> lk(mutex_cand_);
            for (const auto& c : verified_candidates) {
                UnorderedIntPairKey key(c.i, c.j);
                if (history_candidates_.find(key) != history_candidates_.end()) {
                    history_candidates_[key].attempts += 1;
                    history_candidates_[key].valid = c.valid;
                    history_candidates_[key].meas = c.meas;
                }
            }
        }
    }
}

bool LoopClosure::verify(NodeType i, NodeType j, Eigen::Isometry3d& T_ij_out) {
    // Load submap clouds
    // todo: lru cache
    const std::string fi = submap_pcd_folder_ + "submap_" + std::to_string(i) + ".pcd";
    const std::string fj = submap_pcd_folder_ + "submap_" + std::to_string(j) + ".pcd";

    CloudPtr cloud_i = pcl_utils::makeCloud<PointType>();
    CloudPtr cloud_j = pcl_utils::makeCloud<PointType>();
    if (pcl::io::loadPCDFile(fi, *cloud_i) != 0) return false;
    if (pcl::io::loadPCDFile(fj, *cloud_j) != 0) return false;
    if (cloud_i->empty() || cloud_j->empty()) return false;

    auto target_vec = pcl_utils::PclCloudToVecCloud(cloud_i);
    auto source_vec = pcl_utils::PclCloudToVecCloud(cloud_j);
    const Eigen::Isometry3d initial_guess = T_ij_out;

    auto target = std::make_shared<small_gicp::PointCloud>(*target_vec);
    auto source = std::make_shared<small_gicp::PointCloud>(*source_vec);

    auto target_tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(target, small_gicp::KdTreeBuilderTBB());
    auto source_tree =
        std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(source, small_gicp::KdTreeBuilderTBB());

    small_gicp::estimate_covariances_tbb(*target, *target_tree, 10);
    small_gicp::estimate_covariances_tbb(*source, *source_tree, 10);

    auto run_small_gicp = [&](const Eigen::Isometry3d& init_T) {
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionTBB> reg;
        reg.rejector.max_dist_sq = icp_max_correspondence_dist_ * icp_max_correspondence_dist_;
        reg.inlier_max_dist_sq = icp_inlier_dist_threshold_ * icp_inlier_dist_threshold_;
        reg.optimizer.max_iterations = icp_max_iterations_;
        return reg.align(*target, *source, *target_tree, init_T);
    };

    auto result1 = run_small_gicp(initial_guess);
    auto selected_result = result1;
    bool used_result2 = false;

    const auto id_gap = static_cast<int64_t>(std::llabs(static_cast<long long>(i - j)));
    bool kiss_checked = false;
    bool kiss_passed = false;
    size_t kiss_final_inliers = 0;
    small_gicp::RegistrationResult result2;

    if (loop_kiss_matcher_enabled_ && id_gap > kiss_refine_min_id_gap_) {
        kiss_checked = true;

        kiss_matcher::KISSMatcherConfig kiss_config;
        kiss_config.use_voxel_sampling_ = true;
        kiss_config.voxel_size_ = 0.3;
        kiss_config.use_quatro_ = kiss_use_quatro_;
        kiss_matcher::KISSMatcher matcher(kiss_config);
        const auto kiss_solution = matcher.estimate(*source_vec, *target_vec);
        kiss_final_inliers = matcher.getNumFinalInliers();
        kiss_passed = kiss_solution.valid && kiss_final_inliers >= kiss_inliers_threshold_;

        if (kiss_passed) {
            Eigen::Isometry3d T_kiss = Eigen::Isometry3d::Identity();
            T_kiss.linear() = kiss_solution.rotation;
            T_kiss.translation() = kiss_solution.translation;

            result2 = run_small_gicp(T_kiss);

            const bool result2_is_better =
                result2.converged &&
                (!result1.converged ||
                 (result2.error < result1.error && result2.source_inlier_ratio > result1.source_inlier_ratio));

            if (result2_is_better) {
                selected_result = result2;
                used_result2 = true;
            }
        }
    }

    T_ij_out = selected_result.T_target_source;
    const bool verified =
        selected_result.converged && selected_result.source_inlier_ratio >= icp_source_inlier_ratio_threshold_;

    LOG(INFO) << "LoopClosure verify [" << i << ", " << j << "]"
              << " id_gap=" << id_gap << " result1_converged=" << result1.converged
              << " result1_error=" << result1.error << " result1_num_inliers=" << result1.num_inliers
              << " result1_source_inlier_ratio=" << result1.source_inlier_ratio << " kiss_checked=" << kiss_checked
              << " kiss_passed=" << kiss_passed << " kiss_final_inliers=" << kiss_final_inliers
              << " result2_converged=" << result2.converged << " result2_error=" << result2.error
              << " result2_num_inliers=" << result2.num_inliers
              << " result2_source_inlier_ratio=" << result2.source_inlier_ratio
              << " selected=" << (used_result2 ? "result2" : "result1")
              << " min_source_inlier_ratio=" << icp_source_inlier_ratio_threshold_ << " verified=" << verified;

    return verified;
}

}  // namespace legkilo
