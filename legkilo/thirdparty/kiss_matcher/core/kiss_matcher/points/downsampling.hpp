#pragma once

#include <atomic>
#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <kiss_matcher/points/fast_floor.hpp>
#include <kiss_matcher/points/point_cloud.hpp>
#include <kiss_matcher/points/traits.hpp>
#include <kiss_matcher/points/vector3i_hash.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace kiss_matcher {

/**
 * @brief Voxel grid downsampling with TBB backend.
 * @note  This function has minor run-by-run non-deterministic behavior due to parallel data
 * collection.
 * @param points     Input points
 * @param leaf_size  Downsampling resolution
 * @return           Downsampled points
 */
template <typename InputPointCloud, typename OutputPointCloud = InputPointCloud>
std::shared_ptr<OutputPointCloud> VoxelgridSampling(const InputPointCloud& points,
                                                    const double leaf_size) {
  if (traits::size(points) == 0) {
    return std::make_shared<OutputPointCloud>();
  }

  const double inv_leaf_size = 1.0 / leaf_size;

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size          = 21;
  constexpr size_t coord_bit_mask       = (1 << 21) - 1;
  constexpr int coord_offset            = 1 << (coord_bit_size - 1);

  std::vector<std::pair<std::uint64_t, size_t>> coord_pt(traits::size(points));
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, traits::size(points), 64),
      [&](const tbb::blocked_range<size_t>& range) {
        for (size_t i = range.begin(); i != range.end(); i++) {
          const Eigen::Array4i coord =
              fast_floor(traits::point(points, i) * inv_leaf_size) + coord_offset;
          if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
            std::cerr << "warning: voxel coord is out of range!!" << std::endl;
            coord_pt[i] = {invalid_coord, i};
            continue;
          }

          const std::uint64_t bits =
              (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |
              (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |
              (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
          coord_pt[i] = {bits, i};
        }
      });

  tbb::parallel_sort(coord_pt,
                     [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  auto downsampled = std::make_shared<OutputPointCloud>();
  traits::resize(*downsampled, traits::size(points));

  const int block_size            = 2048;
  std::atomic_uint64_t num_points = 0;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, traits::size(points), block_size),
                    [&](const tbb::blocked_range<size_t>& range) {
                      std::vector<Eigen::Vector4d> sub_points;
                      sub_points.reserve(block_size);

                      Eigen::Vector4d sum_pt =
                          traits::point(points, coord_pt[range.begin()].second);
                      for (size_t i = range.begin() + 1; i != range.end(); i++) {
                        if (coord_pt[i].first == invalid_coord) {
                          continue;
                        }

                        if (coord_pt[i - 1].first != coord_pt[i].first) {
                          sub_points.emplace_back(sum_pt / sum_pt.w());
                          sum_pt.setZero();
                        }
                        sum_pt += traits::point(points, coord_pt[i].second);
                      }
                      sub_points.emplace_back(sum_pt / sum_pt.w());

                      const size_t point_index_begin = num_points.fetch_add(sub_points.size());
                      for (size_t i = 0; i < sub_points.size(); i++) {
                        traits::set_point(*downsampled, point_index_begin + i, sub_points[i]);
                      }
                    });

  traits::resize(*downsampled, num_points);

  return downsampled;
}

inline std::vector<Eigen::Vector3f> VoxelgridSampling(const std::vector<Eigen::Vector3f>& points,
                                                      const double leaf_size) {
  if (points.empty()) {
    return {};
  }

  size_t num_raw_points      = points.size();
  const double inv_leaf_size = 1.0 / leaf_size;

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size          = 21;
  constexpr size_t coord_bit_mask       = (1 << 21) - 1;
  constexpr int coord_offset            = 1 << (coord_bit_size - 1);

  std::vector<std::pair<std::uint64_t, size_t>> coord_pt(num_raw_points);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, num_raw_points, 64),
      [&](const tbb::blocked_range<size_t>& range) {
        for (size_t i = range.begin(); i != range.end(); i++) {
          const Eigen::Array3i coord =
              fast_floor_vector3f(points[i] * inv_leaf_size) + coord_offset;
          if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
            std::cerr << "warning: voxel coord is out of range!!" << std::endl;
            coord_pt[i] = {invalid_coord, i};
            continue;
          }

          const std::uint64_t bits =
              (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |
              (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |
              (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
          coord_pt[i] = {bits, i};
        }
      });

  tbb::parallel_sort(coord_pt,
                     [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  std::vector<Eigen::Vector3f> downsampled;
  downsampled.resize(num_raw_points);

  const int block_size            = 2048;
  std::atomic_uint64_t num_points = 0;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, num_raw_points, block_size),
                    [&](const tbb::blocked_range<size_t>& range) {
                      std::vector<Eigen::Vector3f> sub_points;
                      sub_points.reserve(block_size);

                      Eigen::Vector3f sum_pt = points[coord_pt[range.begin()].second];
                      float count            = 1.0;
                      for (size_t i = range.begin() + 1; i != range.end(); i++) {
                        if (coord_pt[i].first == invalid_coord) {
                          continue;
                        }

                        if (coord_pt[i - 1].first != coord_pt[i].first) {
                          sub_points.emplace_back(sum_pt / count);
                          sum_pt.setZero();
                          count = 0.0;
                        }
                        sum_pt += points[coord_pt[i].second];
                        count += 1.0;
                      }
                      sub_points.emplace_back(sum_pt / count);

                      const size_t point_index_begin = num_points.fetch_add(sub_points.size());
                      for (size_t i = 0; i < sub_points.size(); i++) {
                        downsampled[point_index_begin + i] = sub_points[i];
                      }
                    });

  downsampled.resize(num_points);

  return downsampled;
}

}  // namespace kiss_matcher
