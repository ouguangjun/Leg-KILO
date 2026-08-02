#include "core/slam/backend/loop_closure/loop_detector/euclidean_method.h"

#include <array>

#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.hpp"

namespace legkilo {

void EuclideanLoopDetector::setSearchRadius(double r) { search_radius_ = std::max(0.1, r); }

void EuclideanLoopDetector::setMinIdSeparation(int s) { min_id_separation_ = std::max(1, s); }

std::vector<std::pair<NodeType, NodeType>> EuclideanLoopDetector::detectLatest(
    const std::vector<std::pair<NodeType, Eigen::Isometry3d>>& poses) const {
    std::vector<std::pair<NodeType, NodeType>> out;
    const size_t n = poses.size();
    if (n < 2) return out;

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(n);
    std::vector<NodeType> ids;
    ids.reserve(n);
    for (const auto& pj : poses) {
        pts.emplace_back(pj.second.translation());
        ids.emplace_back(pj.first);
    }

    std::vector<std::array<double, 3>> vv;
    vv.reserve(n);
    for (const auto& p : pts) vv.push_back({p.x(), p.y(), p.z()});

    using VVT = std::vector<std::array<double, 3>>;
    using KDAdaptor = KDTreeVectorOfVectorsAdaptor<VVT, double, 3, nanoflann::metric_L2_Simple>;
    KDAdaptor kd(3, vv);
    kd.index->buildIndex();

    const double r2 = search_radius_ * search_radius_;
    using SearchParamsT = nanoflann::SearchParameters;
    SearchParamsT params{};

    for (size_t j = 0; j < n; ++j) {
        const double query_pt[3] = {pts[j].x(), pts[j].y(), pts[j].z()};
        std::vector<nanoflann::ResultItem<size_t, double>> matches;
        kd.index->radiusSearch(query_pt, r2, matches, params);

        const NodeType id_j = ids[j];
        for (const auto& kv : matches) {
            const size_t i = kv.first;
            if (i == j) continue;
            const NodeType id_i = ids[i];
            if (id_i < id_j && (id_j - id_i) >= static_cast<NodeType>(min_id_separation_)) {
                out.emplace_back(id_i, id_j);
            }
        }
    }

    return out;
}

}  // namespace legkilo
