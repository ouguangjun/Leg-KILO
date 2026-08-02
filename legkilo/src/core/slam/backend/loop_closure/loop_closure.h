#ifndef LEGKILO_LOOP_CLOSURE_H
#define LEGKILO_LOOP_CLOSURE_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"

namespace legkilo {
class EuclideanLoopDetector;
}

namespace legkilo {

using NodeType = int64_t;
using IDPoses = std::vector<std::pair<NodeType, Eigen::Isometry3d>>;
using IDPosesPtr = std::shared_ptr<IDPoses>;

struct LoopClosureOutput {
    NodeType i;
    NodeType j;
    Eigen::Isometry3d meas;  // relative pose from i to j
};

class LoopClosure {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LoopClosure(const std::string& config_file);

    ~LoopClosure();

    void start();

    void stop();

    // Input: optimized submap poses (global). Typically pass all finished submaps in order.
    void insert(IDPosesPtr optimized_poses);

    // Output: fetch verified loop closures (not yet fetched).
    std::vector<LoopClosureOutput> fetchVerified();

   private:
    struct Candidate {
        NodeType i;
        NodeType j;
        int attempts = 0;
        bool valid = false;
        bool fetched = false;
        double fitness_score = 0.0;
        size_t kiss_inliers = 0;
        Eigen::Isometry3d meas = Eigen::Isometry3d::Identity();  // i -> j
    };

    void workerLoop();

    bool verify(NodeType i, NodeType j, Eigen::Isometry3d& T_ij_out);

   private:
    // config
    double search_radius_ = 5.0;
    int min_id_separation_ = 5;
    double icp_max_correspondence_dist_ = 1.0;
    double icp_inlier_dist_threshold_ = 0.5;
    int icp_max_iterations_ = 40;
    int icp_num_threads_ = 2;
    double icp_source_inlier_ratio_threshold_ = 0.5;
    bool loop_kiss_matcher_enabled_ = false;
    size_t kiss_inliers_threshold_ = 40;
    bool kiss_use_quatro_ = false;
    int kiss_refine_min_id_gap_ = 100;
    int verify_max_attempts_ = 2;

    // PCD folder
    std::string submap_pcd_folder_;

    // history candidates
    std::mutex mutex_cand_;
    std::unordered_map<UnorderedIntPairKey, Candidate, UnorderedIntPairKey::Hasher, UnorderedIntPairKey::Equal>
        history_candidates_;

    // detector
    std::unique_ptr<EuclideanLoopDetector> detector_;

    // threading
    std::mutex mutex_queue_;
    std::condition_variable cv_;
    std::deque<IDPosesPtr> trigger_queue_;
    std::thread worker_;
    std::atomic<bool> stopping_{false};
};

}  // namespace legkilo
#endif  // LEGKILO_LOOP_CLOSURE_H
