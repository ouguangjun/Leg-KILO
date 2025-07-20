#ifndef LEG_KILO_VOXEL_GRID_H
#define LEG_KILO_VOXEL_GRID_H


#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>
#include <tbb/concurrent_vector.h>

#include "common.hpp"

namespace legkilo {

struct HashVec{
    inline size_t operator()(const Eigen::Matrix<int, 3, 1>& key) const{
        return size_t(((key[0]) * 73856093) ^ ((key[1]) * 471943) ^ ((key[2]) * 83492791)) % 10000000;
    }
};

class VoxelGrid{
public:
    using KeyType = Eigen::Matrix<int, 3, 1>;
    using VecType = tbb::concurrent_vector<size_t>;
    using MapType = tbb::concurrent_unordered_map<KeyType, VecType, HashVec>;
    VoxelGrid(float resolution): resolution_(resolution){
        if(resolution < 0.001f){
            throw std::runtime_error("Error voxelgrid resolution, please set greater than 0.001");
        }
        resolution_inv_ = 1.0f / resolution;
    }

    void setResolution(float resolution){
        if(resolution < 0.001f){
            throw std::runtime_error("Error voxelgrid resolution, please set greater than 0.001");
        }
        resolution_ = resolution;
        resolution_inv_ = 1.0f / resolution;
    }

    void filter(CloudPtr& cloud_in, CloudPtr& cloud_out){
        cloud_out->clear();
        cloud_out->reserve(cloud_in->points.size());
        MapType hash_map;

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, cloud_in->points.size()),
            [&, this](const tbb::blocked_range<size_t>& range) {
                for (size_t i = range.begin(); i < range.end(); ++i) {
                    const auto& point = cloud_in->points[i];
                    KeyType key;
                    key[0] = static_cast<int>(std::floor(point.x * resolution_inv_));
                    key[1] = static_cast<int>(std::floor(point.y * resolution_inv_));
                    key[2] = static_cast<int>(std::floor(point.z * resolution_inv_));
                    hash_map[key].push_back(i);
                }
        });

        for(const auto& [key, idx_vec] : hash_map){
            size_t median_idx = idx_vec.size() / 2;
            cloud_out->points.push_back(cloud_in->points[idx_vec[median_idx]]);
        }
    }

private:
    VoxelGrid() = delete;
    float resolution_ = 0.5;
    float resolution_inv_ = 2.0;
};
} // namespace legkilo
#endif // LEG_KILO_VOXEL_GRID_H