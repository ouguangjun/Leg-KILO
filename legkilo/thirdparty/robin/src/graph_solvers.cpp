// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#include <pmc/pmc.h>
#include <robin/pkc.hpp>
#include <robin/graph_core.hpp>
#include <robin/graph_solvers.hpp>

namespace robin {

//
// Definitions for the KCoreDecompositionSolver class
//
void KCoreDecompositionSolver::Solve(const IGraph& g) {
  core_.resize(g.VertexCount());
  switch (mode_) {
  case KCORE_SOLVER_MODE::PKC_PARALLEL:
    pkc::PKC_parallel(g, &core_, false);
    break;
  case KCORE_SOLVER_MODE::PKC_PARALLEL_OPTIMIZED:
    pkc::PKC_parallel(g, &core_, true);
    break;
  case KCORE_SOLVER_MODE::PKC_SERIAL:
    pkc::PKC_original_serial(g, &core_);
    break;
  case KCORE_SOLVER_MODE::BZ_SERIAL:
    pkc::BZ_kCores(g, &core_);
    break;
  }

  // update max k-core
  auto max_core = std::max_element(core_.begin(), core_.end());
  if (max_core == core_.end()) {
    max_k_core_number_ = 0;
    core_.resize(g.VertexCount());
  } else {
    max_k_core_number_ = *max_core;
    for (size_t i = 0; i < core_.size(); ++i) {
      if (core_[i] == *max_core) {
        max_k_core_.push_back(i);
      }
    }
  }
}

std::vector<size_t> KCoreDecompositionSolver::GetKCore(const size_t& k) {
  std::vector<size_t> k_core;
  for (size_t i = 0; i < core_.size(); ++i) {
    if (core_[i] >= k) {
      k_core.push_back(i);
    }
  }
  return k_core;
}

//
// Definitions for the MaxCliqueSolver class
//
std::vector<size_t> MaxCliqueSolver::FindMaxClique(const IGraph& graph) const {
  // Create a PMC graph from the robin graph
  std::vector<int> edges;
  std::vector<long long> vertices;
  vertices.push_back(edges.size());

  size_t vertex_count = graph.VertexCount();
  for (size_t i = 0; i < vertex_count; ++i) {
    size_t v_edge_count = graph.GetVertexDegree(i);
    for (size_t k = 0; k < v_edge_count; ++k) {
      auto u = graph.GetVertexEdge(i, k);
      edges.push_back(u);
    }
    //edges.insert(edges.end(), c_edges.begin(), c_edges.end());
    vertices.push_back(edges.size());
  }

  // Use PMC to calculate
  pmc::pmc_graph G(vertices, edges);

  // Prepare PMC input
  // TODO: Incorporate this to the constructor
  pmc::input in;
  in.algorithm = 0;
  in.threads = 12;
  in.experiment = 0;
  in.lb = 0;
  in.ub = 0;
  in.param_ub = 0;
  in.adj_limit = 20000;
  in.time_limit = params_.time_limit;
  in.remove_time = 4;
  in.graph_stats = false;
  in.verbose = false;
  in.help = false;
  in.MCE = false;
  in.decreasing_order = false;
  in.heu_strat = "kcore";
  in.vertex_search_order = "deg";

  // vector to represent max clique
  std::vector<int> C;

  // upper-bound of max clique
  G.compute_cores();
  auto max_core = G.get_max_core();

  ROBIN_DEBUG_INFO_MSG("Max core number: " << max_core);
  ROBIN_DEBUG_INFO_MSG("Num vertices: " << vertices.size());

  if (in.ub == 0) {
    in.ub = max_core + 1;
  }

  // lower-bound of max clique
  if (in.lb == 0 && in.heu_strat != "0") { // skip if given as input
    pmc::pmc_heu maxclique(G, in);
    in.lb = maxclique.search(G, C);
  }

  if (in.lb == 0) {
    // This means that max clique has a size of one
    ROBIN_DEBUG_ERROR_MSG("Max clique has a size of 1.");
    std::vector<size_t> C_result(C.begin(), C.end());
    return C_result;
  }

  if (in.lb == in.ub) {
    std::vector<size_t> C_result(C.begin(), C.end());
    return C_result;
  }

  // Optional exact max clique finding
  if (params_.solver_mode == CLIQUE_SOLVER_MODE::PMC_EXACT) {
    // The following methods are used:
    // 1. k-core pruning
    // 2. neigh-core pruning/ordering
    // 3. dynamic coloring bounds/sort
    // see the original PMC paper and implementation for details:
    // R. A. Rossi, D. F. Gleich, and A. H. Gebremedhin, “Parallel Maximum Clique Algorithms with
    // Applications to Network Analysis,” SIAM J. Sci. Comput., vol. 37, no. 5, pp. C589–C616, Jan.
    // 2015.
    if (G.num_vertices() < in.adj_limit) {
      G.create_adj();
      pmc::pmcx_maxclique finder(G, in);
      finder.search_dense(G, C);
    } else {
      pmc::pmcx_maxclique finder(G, in);
      finder.search(G, C);
    }
  }

  std::vector<size_t> C_result(C.begin(), C.end());
  return C_result;
}

}