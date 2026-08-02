// SPDX-License-Identifier: MIT
// @file voxel_grid.hpp
// @brief Parallel voxel grid downsampling utilities.
// @author Ou Guangjun
// @created 2025-01-18
// @maintainer ouguangjun98@gmail.com
//
// Downsampling strategies (brief):
// - MedianRepresentative: encode + sort by voxel key, then pick the input point closest to the voxel center.
// - RandomPerVoxelQuota: encode + sort; per voxel take up to K evenly-strided indices, then apply a global random
//   prune with std::sample to meet the overall target.
#ifndef LEG_KILO_VOXEL_GRID_H
#define LEG_KILO_VOXEL_GRID_H

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include "common/math_utils.hpp"
#include "common/pcl_types.h"

namespace legkilo {

enum class SamplingMode { MedianRepresentative, RandomPerVoxelQuota };

class VoxelGrid {
   public:
    using KeyType = int64_t;

    VoxelGrid(float resolution, SamplingMode mode, size_t target_num = 2000, float overshoot = 1.5f, size_t seed = 42)
        : sampling_mode_(mode), target_num_(target_num), overshoot_(overshoot), seed_(seed) {
        setResolution(resolution);
    }

    void setResolution(float resolution) {
        if (resolution < 0.001f) {
            resolution = 0.001f;
            std::cerr << "[VoxelGrid] Warning: resolution too small, set to minimum 0.001 m." << std::endl;
        }
        resolution_ = resolution;
        resolution_inv_ = 1.0f / resolution_;
    }

    void filter(const CloudPtr& cloud_in, CloudPtr& cloud_out) const {
        cloud_out->clear();
        if (!cloud_in || cloud_in->points.empty()) return;

        switch (sampling_mode_) {
            case SamplingMode::MedianRepresentative: filterMedian(cloud_in, cloud_out); break;
            case SamplingMode::RandomPerVoxelQuota: filterRandomPerVoxel(cloud_in, cloud_out); break;
        }
    }

   private:
    void buildEntries(const CloudPtr& cloud_in, std::vector<std::pair<KeyType, size_t>>& entries) const {
        const size_t N = cloud_in->points.size();
        entries.resize(N);

        const double inv_res = static_cast<double>(resolution_inv_);
        tbb::parallel_for(tbb::blocked_range<size_t>(0, N, 1024), [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                const auto& pt = cloud_in->points[i];
                Eigen::Vector3d p(static_cast<double>(pt.x), static_cast<double>(pt.y), static_cast<double>(pt.z));
                const Eigen::Vector3i key_i = voxelKeyFastFloor(p, inv_res);
                const KeyType code = encodeKey(key_i);
                entries[i].first = code;
                entries[i].second = i;
            }
        });

        tbb::parallel_sort(entries.begin(), entries.end(),
                           [](const auto& a, const auto& b) { return a.first < b.first; });
    }

    // --- 计算分组边界 ---
    void computeGroups(const std::vector<std::pair<KeyType, size_t>>& entries,
                       std::vector<std::pair<size_t, size_t>>& groups) const {
        groups.clear();
        const size_t N = entries.size();
        if (N == 0) return;
        size_t start = 0;
        while (start < N) {
            size_t begin = start;
            size_t end = start + 1;
            while (end < N && entries[end].first == entries[begin].first) { ++end; }
            groups.emplace_back(begin, end - begin);
            start = end;
        }
    }

    void filterMedian(const CloudPtr& cloud_in, CloudPtr& cloud_out) const {
        cloud_out.reset(new PointCloudType());
        if (!cloud_in || cloud_in->empty()) return;

        std::vector<std::pair<KeyType, size_t>> entries;
        buildEntries(cloud_in, entries);
        std::vector<std::pair<size_t, size_t>> groups;
        computeGroups(entries, groups);

        cloud_out->points.reserve(groups.size());
        for (const auto& g : groups) {
            const size_t begin = g.first;
            const size_t len = g.second;
            if (len == 0) continue;
            const size_t mid_off = len / 2;
            const size_t idx = entries[begin + mid_off].second;
            cloud_out->points.push_back(cloud_in->points[idx]);
        }
        cloud_out->points.shrink_to_fit();
    }

    void filterRandomPerVoxel(const CloudPtr& cloud_in, CloudPtr& cloud_out) const {
        cloud_out.reset(new PointCloudType());
        if (!cloud_in) return;
        const size_t N = cloud_in->points.size();
        if (target_num_ >= N || N == 0) {
            *cloud_out = *cloud_in;
            return;
        }

        std::vector<std::pair<KeyType, size_t>> entries;
        buildEntries(cloud_in, entries);
        std::vector<std::pair<size_t, size_t>> groups;
        computeGroups(entries, groups);

        const size_t num_voxels = groups.size();
        if (num_voxels == 0) return;

        size_t points_per_voxel = static_cast<size_t>(target_num_ * overshoot_ / num_voxels);
        if (points_per_voxel == 0) points_per_voxel = 1;

        const size_t max_total = static_cast<size_t>(std::ceil(overshoot_ * target_num_));

        tbb::enumerable_thread_specific<std::vector<size_t>> tls_indices;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, groups.size(), 64), [&](const tbb::blocked_range<size_t>& r) {
            auto& local = tls_indices.local();

            for (size_t gi = r.begin(); gi < r.end(); ++gi) {
                const size_t begin = groups[gi].first;
                const size_t m = groups[gi].second;
                if (m == 0) continue;
                if (m <= points_per_voxel) {
                    for (size_t off = 0; off < m; ++off) local.push_back(entries[begin + off].second);
                } else {
                    if (points_per_voxel == 1) {
                        local.push_back(entries[begin + m / 2].second);
                    } else {
                        const size_t sample_last = points_per_voxel - 1;
                        const size_t group_last = m - 1;
                        for (size_t off = 0; off < points_per_voxel; ++off) {
                            const size_t sample_off = (off * group_last + sample_last / 2) / sample_last;
                            local.push_back(entries[begin + sample_off].second);
                        }
                    }
                }
            }
        });

        std::vector<size_t> selected;
        {
            size_t total = 0;
            tls_indices.combine_each([&](const std::vector<size_t>& v) { total += v.size(); });
            selected.reserve(total);
            tls_indices.combine_each([&](std::vector<size_t>& v) {
                selected.insert(selected.end(), v.begin(), v.end());
                v.clear();
                v.shrink_to_fit();
            });
        }

        if (selected.size() > max_total) {
            std::vector<size_t> shrunk;
            shrunk.reserve(max_total);
            std::mt19937 cut_rng = std::mt19937(seed_);
            std::sample(selected.begin(), selected.end(), std::back_inserter(shrunk), max_total, cut_rng);
            selected.swap(shrunk);
        }

        cloud_out->points.reserve(selected.size());
        for (size_t i : selected) cloud_out->points.push_back(cloud_in->points[i]);
    }

   private:
    VoxelGrid() = delete;
    float resolution_ = 0.5f;
    float resolution_inv_ = 2.0f;
    SamplingMode sampling_mode_ = SamplingMode::RandomPerVoxelQuota;
    size_t target_num_ = 2000;
    float overshoot_ = 1.5f;
    size_t seed_ = 42;
};
}  // namespace legkilo
#endif  // LEG_KILO_VOXEL_GRID_H
