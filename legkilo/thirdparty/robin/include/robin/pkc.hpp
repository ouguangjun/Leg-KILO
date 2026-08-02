// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <cassert>
#include <iostream>
#include <omp.h>

#include <robin/graph.hpp>

namespace robin {

/**
 * @brief Namespace for encapsulating the kcore related functions from the PKC paper
 *
 * Credit goes to:
 * H. Kabir and K. Madduri, "Parallel k-core Decomposition on Multicore Platforms," in The 2nd IEEE
 * Workshop on Parallel and Distributed Processing for Computational Social Systems
 * (ParSocial 2017), June 2017.
 *
 * Necessary modifications have been made to make them work with the graph types we have in robin.
 */
namespace pkc {

typedef size_t vid_t;
typedef size_t eid_t;

/**
 * @brief PKC graph type
 */
struct graph_t {
  long n = 0;
  long m = 0;

  /**
   * @brief Pointer to adj list array
   */
  vid_t* adj = nullptr;

  /**
   * @brief Pointer to the row index array in a CSR graph format
   *
   * num_edges[i+1] - num_edges[i] = degree of vertex i
   */
  eid_t* num_edges = nullptr;
};

/**
 * @brief Routine for running BZ algorithm for k-core decomposition.
 * BZ algorithm is described in this paper:
 * Batagelj, Vladimir, and Matjaz Zaversnik. "An O (m) algorithm for cores decomposition of
 * networks." arXiv preprint cs/0310049 (2003).
 *
 * @param g pointer to graph_t
 * @param deg pointer to output vector for storing core number
 */
void BZ_kCores(const IGraph& g, std::vector<size_t>* deg);

/**
 * @brief K-core decomposition with parallel PKC. This is the non-optimized version.
 * Each thread has its own queue -- parallel PKC_org for k-core decomposition
 *
 * @param g
 * @param deg
 */
void PKC_original(const IGraph& g, size_t* deg);

/**
 * @brief K-core decomposition with parallel PKC. This particular function is suitable for large
 * graph, with optimization detailed in the original paper.
 *
 * @param g
 * @param deg
 */
void PKC_optimized(const IGraph& g, size_t* deg);

/**
 * @brief Interface with robin::Graph for K-core decomposition with parallel PKC.
 * @param g
 * @param core
 */
void PKC_parallel(const IGraph& g, std::vector<size_t>* core, bool use_optimized);

/**
 * @brief K-core decomposition with serial PKC
 * @param g
 * @param core
 */
void PKC_original_serial(const IGraph& g, std::vector<size_t>* deg);

} // namespace pkc
} // namespace robin