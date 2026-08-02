// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <robin/graph_core.hpp>

namespace robin {

/**
 * @brief Finding K-cores using Parallel K-Core (PKC) algorithm
 *
 * For details about the PKC algorithm, please refer to:
 * Kabir, Humayun, and Kamesh Madduri. "Parallel k-core decomposition on multicore platforms." 2017
 * IEEE International Parallel and Distributed Processing Symposium Workshops (IPDPSW). IEEE, 2017.
 */
class KCoreDecompositionSolver {
public:
  enum class KCORE_SOLVER_MODE {
    PKC_PARALLEL = 0,
    PKC_PARALLEL_OPTIMIZED = 1,
    PKC_SERIAL = 2,
    BZ_SERIAL = 3,
  };

  KCoreDecompositionSolver() = default;

  explicit KCoreDecompositionSolver(const KCORE_SOLVER_MODE& mode) : mode_(mode){};

  /**
   * @brief Calculate K-core decomposition on the graph provided
   * @param g
   */
  void Solve(const IGraph& g);

  /**
   * @brief Return the core numbers of all vertices
   * @return
   */
  const std::vector<size_t>& GetCoreNumbers() { return core_; };

  /**
   * @brief Return the vertices of the largest k-core
   * @return
   */
  const std::vector<size_t>& GetMaxKCore() { return max_k_core_; };

  /**
   * @brief Return the maximum core number of all vertices in the graph.
   * @return
   */
  size_t GetMaxCoreNumber() const { return max_k_core_number_; };

  /**
   * @brief Return k-core of a specific size
   * @param[in] k size of the k-core we are looking for
   * @return
   */
  std::vector<size_t> GetKCore(const size_t& k);

private:
  KCORE_SOLVER_MODE mode_ = KCORE_SOLVER_MODE::PKC_PARALLEL_OPTIMIZED;
  std::vector<size_t> core_;
  std::vector<size_t> max_k_core_;
  size_t max_k_core_number_ = 0;
};

/**
 * A facade to the Parallel Maximum Clique (PMC) library.
 *
 * For details about PMC, please refer to:
 * https://github.com/ryanrossi/pmc
 * and
 * Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa Patwary, A Fast Parallel
 * Maximum Clique Algorithm for Large Sparse Graphs and Temporal Strong Components, arXiv preprint
 * 1302.6256, 2013.
 */
class MaxCliqueSolver {
public:
  /**
   * Enum representing the solver algorithm to use
   */
  enum class CLIQUE_SOLVER_MODE {
    PMC_EXACT = 0,
    PMC_HEU = 1,
  };

  /**
   * Parameter struct for MaxCliqueSolver
   */
  struct Params {

    /**
     * Algorithm used for finding max clique.
     */
    CLIQUE_SOLVER_MODE solver_mode = CLIQUE_SOLVER_MODE::PMC_EXACT;

    /**
     * Time limit on running the solver.
     */
    double time_limit = 3600;
  };

  MaxCliqueSolver() = default;

  explicit MaxCliqueSolver(const Params& params) : params_(params){};

  /**
   * Find the maximum clique within the graph provided. By maximum clique, it means the clique of
   * the largest size in an undirected graph.
   * @param graph
   * @return a vector of indices of cliques
   */
  std::vector<size_t> FindMaxClique(const IGraph& graph) const;

private:
  Params params_;
};

}