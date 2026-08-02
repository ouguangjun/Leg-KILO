// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <vector>

#include <robin/core.hpp>
#include <robin/graph.hpp>
#include <robin/macros.hpp>
#include <robin/problems.hpp>
#include <robin/utils.hpp>

/**
 * @brief API for robin. These are some easy access functions that allow for you to test the
 * functionalities of robin. Both MATLAB and Python bindings expose these functions.
 */
namespace robin {

//
// Graph structures
//
enum class InlierGraphStructure {
  MAX_CORE = 0,
  MAX_CLIQUE = 1,
};

std::vector<size_t> FindInlierStructure(const IGraph* g, InlierGraphStructure graph_structure);

//
// Vector Averaging
//
/**
 * @brief Select inliers for vector averaging problems
 * @param measurements
 * @param graph_structure
 * @param noise_bound
 * @return
 */
IGraph*
MakeVecAvgInvGraph(const Eigen::MatrixXd& measurements, double noise_bound,
                   GraphsStorageType graph_storage_type = robin::GraphsStorageType::ATOMIC_CSR);

//
// Single Rotation Averaging
//
enum class So3Distance {
  CHORDAL_DISTANCE = 0,
  GEODESIC_DISTANCE = 1,
};

/**
 * @brief Select inliers for single rotation averaging
 * @param measurements
 * @param graph_structure
 * @param dist_type
 * @param noise_bound
 * @return
 */
IGraph*
MakeRotAvgInvGraph(const std::vector<Eigen::Matrix3d>& measurements, So3Distance dist_type,
                   double noise_bound,
                   GraphsStorageType graph_storage_type = robin::GraphsStorageType::ATOMIC_CSR);

//
// 3D Registration
//
/**
 * @brief Select inliers for 3D registration
 * @param measurements_3d_points
 * @param graph_structure
 * @param noise_bound
 * @return
 */
IGraph*
Make3dRegInvGraph(const Eigen::Matrix3Xd& src_3d_points, const Eigen::Matrix3Xd& dst_3d_points,
                  double noise_bound,
                  GraphsStorageType graph_storage_type = robin::GraphsStorageType::ATOMIC_CSR);

//
// Mesh Registration
//
/**
 * @brief Select inliers for 3D registration with normals
 * @param measurements_3d_points
 * @param measurements_normals
 * @param graph_structure
 * @param noise_bound
 * @return
 */
IGraph* Make3dNormalRegInvGraph(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& src_3d_points_with_normals,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& dst_3d_points_with_normals,
    const Eigen::Vector2d& noise_bound,
    GraphsStorageType graph_storage_type = robin::GraphsStorageType::ATOMIC_CSR);

} // namespace robin
