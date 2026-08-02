// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#include <robin/robin.hpp>

std::vector<size_t> robin::FindInlierStructure(const IGraph* g,
                                               InlierGraphStructure graph_structure) {
  // identify inlier structures
  switch (graph_structure) {
  case InlierGraphStructure::MAX_CORE: {
    KCoreDecompositionSolver k_core_decomposition_solver(
        robin::KCoreDecompositionSolver::KCORE_SOLVER_MODE::BZ_SERIAL);
    k_core_decomposition_solver.Solve(*g);
    return k_core_decomposition_solver.GetMaxKCore();
  }
  case InlierGraphStructure::MAX_CLIQUE: {
    robin::MaxCliqueSolver::Params clique_params;
    clique_params.solver_mode = robin::MaxCliqueSolver::CLIQUE_SOLVER_MODE::PMC_EXACT;
    robin::MaxCliqueSolver clique_solver(clique_params);
    return clique_solver.FindMaxClique(*g);
  }
  }
  return {};
}

robin::IGraph* robin::MakeVecAvgInvGraph(const Eigen::MatrixXd& measurements, double noise_bound,
                                         GraphsStorageType graph_storage_type) {
  // Measurements set
  VectorY y_set(measurements);

  // Compatibility check function
  robin::SvaCompCheck sva_comp_check(noise_bound);

  // Compatibility graph constructor
  robin::CompGraphConstructor<VectorY, robin::SvaCompCheck, 2> graph_constructor;
  graph_constructor.SetCompCheckFunction(&sva_comp_check);
  graph_constructor.SetMeasurements(&y_set);

  // InvGraphConstructor
  auto* g = graph_constructor.BuildCompGraph(graph_storage_type);
  return g;
}

robin::IGraph* robin::MakeRotAvgInvGraph(const std::vector<Eigen::Matrix3d>& measurements,
                                         So3Distance dist_type, double noise_bound,
                                         GraphsStorageType graph_storage_type) {
  // Measurements sets
  So3Y y_set(measurements);

  // InvFunc
  switch (dist_type) {
  case So3Distance::CHORDAL_DISTANCE: {
    // Distance function
    So3ChordalDist inv_func;

    // Compatibility check function
    robin::SraCompCheck<So3ChordalDist> comp_check(&inv_func, noise_bound);

    // InvGraphConstructor
    robin::CompGraphConstructor<So3Y, robin::SraCompCheck<So3ChordalDist>, 2> graph_constructor;
    graph_constructor.SetCompCheckFunction(&comp_check);
    graph_constructor.SetMeasurements(&y_set);

    // Build graph
    auto* g = graph_constructor.BuildCompGraph(graph_storage_type);

    return g;
  }
  case robin::So3Distance::GEODESIC_DISTANCE: {
    // InvFunc
    So3GeodesicDist inv_func;

    // Compatibility check function
    robin::SraCompCheck<So3GeodesicDist> comp_check(&inv_func, noise_bound);

    // InvGraphConstructor
    robin::CompGraphConstructor<So3Y, robin::SraCompCheck<So3GeodesicDist>, 2> graph_constructor;
    graph_constructor.SetCompCheckFunction(&comp_check);
    graph_constructor.SetMeasurements(&y_set);

    // Build graph
    auto* g = graph_constructor.BuildCompGraph(graph_storage_type);

    return g;
  }
  }
  return nullptr;
}

robin::IGraph* robin::Make3dRegInvGraph(const Eigen::Matrix3Xd& src_3d_points,
                                        const Eigen::Matrix3Xd& dst_3d_points, double noise_bound,
                                        GraphsStorageType graph_storage_type) {
  // Measurement 3D points set
  Points3d measurements(dst_3d_points);

  // Model set
  // Note: for our purpose measurements and model points can be flipped
  Points3d model(src_3d_points);

  // Compatibility function
  Points3dRegCompCheck comp_check(&model, noise_bound);

  // InvGraphConstructor
  robin::CompGraphConstructor<Points3d, robin::Points3dRegCompCheck, 2> graph_constructor;
  graph_constructor.SetCompCheckFunction(&comp_check);
  graph_constructor.SetMeasurements(&measurements);

  // Build graph
  auto* g = graph_constructor.BuildCompGraph(graph_storage_type);

  return g;
}

robin::IGraph* robin::Make3dNormalRegInvGraph(
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& src_3d_points_with_normals,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& dst_3d_points_with_normals,
    const Eigen::Vector2d& noise_bound, GraphsStorageType graph_storage_type) {
  assert(noise_bound.size() == 2);
  assert(src_3d_points_with_normals.cols() == dst_3d_points_with_normals.cols());

  // Measurements
  Points3dWithNormals measurements(dst_3d_points_with_normals);

  // Models
  Points3dWithNormals models(src_3d_points_with_normals);

  // Compatibility check function
  Eigen::Vector2d updated_noise_bound = noise_bound;
  updated_noise_bound(1) = std::cos(updated_noise_bound(1));
  PointsNormals3dRegCompCheck comp_check(&models, updated_noise_bound[0], updated_noise_bound[1]);

  // InvGraphConstructor
  robin::CompGraphConstructor<Points3dWithNormals, robin::PointsNormals3dRegCompCheck, 2>
      graph_constructor;
  graph_constructor.SetCompCheckFunction(&comp_check);
  graph_constructor.SetMeasurements(&measurements);

  // Build graph
  auto* g = graph_constructor.BuildCompGraph(graph_storage_type);

  return g;
}
