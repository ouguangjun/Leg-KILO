// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <atomic>
#include <cassert>
#include <chrono>
#include <queue>
#include <thread>
#include <vector>

#include <omp.h>

#include <xenium/ramalhete_queue.hpp>
#include <xenium/reclamation/stamp_it.hpp>

#include <robin/graph.hpp>
#include <robin/macros.hpp>
#include <robin/math.hpp>
#include <robin/utils.hpp>

/// Core library header file

namespace robin {

using EdgeType = std::pair<size_t, size_t>;
using EdgeQueueType =
    xenium::ramalhete_queue<EdgeType*, xenium::policy::reclaimer<xenium::reclamation::stamp_it>,
                            xenium::policy::pop_retries<0>>;

/**
 * @brief Interface for the measurement and model sets
 * @tparam P return type that needs to be accepted by the InvFunc used in Problem
 */
template <typename P> class SetBase {
public:
  /**
   * @brief Return the number of correspondences
   * @return
   */
  virtual size_t size() const = 0;

  /**
   * @brief Return the ith point in the set
   * @param i
   * @return
   */
  virtual P operator[](size_t i) const = 0;
};

/**
 * @brief Base builder for generating compatibility graphs. For internal use.
 *
 * See below for the trick for allowing partial specialization for the BuildCompGraph function
 * https://stackoverflow.com/questions/27453449/c-template-partial-specialization-with-inheritance
 *
 * @tparam Measurements
 * @tparam CompCheckFunc a function for checking compatibilities among measurements in a subset. It
 * takes in two pointers, the first pointer pointing to the set of measurements, and the second
 * pointer pointing to the set of indices in a subset
 * @tparam CompCheckFuncArity
 */
template <typename Measurements, typename CompCheckFunc, size_t CompCheckFuncArity>
class BaseCompGraphConstructor {
public:
  BaseCompGraphConstructor(){};

  /**
   * Set the measurements used for contructing the compatibility graphs
   * @param measurements
   */
  void SetMeasurements(Measurements* measurements) { y_set_ = measurements; }

  /**
   * Set the compatibility check function of this problem
   * @param comp_check_func
   */
  void SetCompCheckFunction(CompCheckFunc* comp_check_func) { comp_check_func_ = comp_check_func; };

  /**
   * @brief Serial version of building the compatibility graph
   * @param g
   */
  void BuildCompGraphSerial(AdjListGraph* g) {
    // prepare the graph
    size_t N = y_set_->size();
    g->Clear();
    g->PopulateVertices(N);

    // total number of possible subsets / combinations of indices
    size_t nr_subsets = robin::Choose(N, CompCheckFuncArity);

    // for each possible subset, check compatibility
    auto* subset_indices = new size_t[CompCheckFuncArity];
    auto* edge_pair_indices = new size_t[2];
    for (size_t i = 0; i < nr_subsets; ++i) {
      // compute current subset indices
      robin::CombinationDecode(N, CompCheckFuncArity, i, subset_indices);

      // compute compatibility
      bool compatible = (*comp_check_func_)(y_set_, subset_indices);

      // add edges
      if (compatible) {
        // add edges for all n choose 2 pairs, where n=size of the subset of measurements
        for (size_t j = 0; j < CompCheckFuncArity - 1; ++j) {
          for (size_t k = j + 1; k < CompCheckFuncArity; ++k) {
            g->AddEdge(subset_indices[j], subset_indices[k]);
          }
        }
      }
    }

    delete[] edge_pair_indices;
    delete[] subset_indices;
  }

  /**
   * @brief Build the compatibility graph
   *
   * @param g a robin::Graph object
   */
  void BuildCompGraph(AdjListGraph* g) {
    // prepare the graph
    size_t N = y_set_->size();
    g->Clear();
    g->PopulateVertices(N);

    // concurrent edge queue
    EdgeQueueType edge_queue;

    // total number of possible subsets / combinations of indices
    size_t nr_subsets = robin::Choose(N, CompCheckFuncArity);

    // for each possible subset, check compatibility
#pragma omp parallel default(none) shared(N, nr_subsets, edge_queue)
    {
      auto* subset_indices = new size_t[CompCheckFuncArity];
      auto* edge_pair_indices = new size_t[2];
#pragma omp for
      for (size_t i = 0; i < nr_subsets; ++i) {
        // compute current subset indices
        robin::CombinationDecode(N, CompCheckFuncArity, i, subset_indices);

        // compute compatibility
        bool compatible = (*comp_check_func_)(y_set_, subset_indices);

        // add edges
        if (compatible) {
          if (CompCheckFuncArity == 2) {
            // for pairwise compatibility checks
            edge_queue.push(new EdgeType(subset_indices[0], subset_indices[1]));
            // edge_list.emplace_back(subset_indices[0], subset_indices[1]);
          } else {
            // add edges for all n choose 2 pairs, where n=size of the subset of measurements
            for (size_t j = 0; j < CompCheckFuncArity - 1; ++j) {
              for (size_t k = j + 1; k < CompCheckFuncArity; ++k) {
                edge_queue.push(new EdgeType(subset_indices[j], subset_indices[k]));
              }
            }
          }
        }
      }

      delete[] edge_pair_indices;
      delete[] subset_indices;
    };

    // add edges to graph
    EdgeType* edge = nullptr;
    while (edge_queue.try_pop(edge)) {
      g->AddEdge(edge->first, edge->second);
      delete edge;
    }
  };

protected:
  Measurements* y_set_ = nullptr;
  CompCheckFunc* comp_check_func_ = nullptr;
};

/**
 * @brief Builder for constructing compatibility graphs. Use this instead of
 * BaseCompGraphConstructor
 *
 * @tparam Measurements
 * @tparam CompCheckFunc
 * @tparam CompCheckFuncArity
 */
template <typename Measurements, typename CompCheckFunc, size_t CompCheckFuncArity>
class CompGraphConstructor
    : public BaseCompGraphConstructor<Measurements, CompCheckFunc, CompCheckFuncArity> {};

/**
 * @brief Specialization of CompGraphConstructor for pairwise compatibility checks (2-measurement
 * invariant)
 *
 * @tparam Measurements
 * @tparam CompCheckFunc
 */
template <typename Measurements, typename CompCheckFunc>
class CompGraphConstructor<Measurements, CompCheckFunc, 2>
    : public BaseCompGraphConstructor<Measurements, CompCheckFunc, 2> {

public:
  void BuildCompGraph_parallel_edge_buffer(AdjListGraph* g) {
    size_t N = this->y_set_->size();
    g->Clear();
    g->PopulateVertices(N);

    // parallel compatibility graph building
    // 1. each thread initialize a queue for storing edges
    // 2. a critical section for dequeuing edges into graph
#pragma omp parallel default(none) shared(g, N)
    {
      auto* subset_indices = new size_t[2];
      std::queue<EdgeType> edge_buffer;

#pragma omp for
      for (size_t k = 0; k < N * (N - 1) / 2; ++k) {
        size_t i = k / N;
        size_t j = k % N;
        if (j <= i) {
          i = N - i - 2;
          j = N - j - 1;
        }
        subset_indices[0] = i;
        subset_indices[1] = j;
        bool add_edge = (*(this->comp_check_func_))(this->y_set_, subset_indices);
        if (add_edge) {
          edge_buffer.push(EdgeType(i, j));
        }
      }

#pragma omp critical
      {
        while (!edge_buffer.empty()) {
          g->AddEdgeUnsafe(edge_buffer.front().first, edge_buffer.front().second);
          edge_buffer.pop();
        }
      };
      delete[] subset_indices;
    }
  }

  void BuildCompGraph_serial(AdjListGraph* g) {
    size_t N = this->y_set_->size();
    g->Clear();
    g->PopulateVertices(N);

    // parallel compatibility graph building
    // 1. each thread initialize a queue for storing edges
    // 2. a critical section for dequeuing edges into graph
    auto* subset_indices = new size_t[2];

    for (size_t k = 0; k < N * (N - 1) / 2; ++k) {
      size_t i = k / N;
      size_t j = k % N;
      if (j <= i) {
        i = N - i - 2;
        j = N - j - 1;
      }
      subset_indices[0] = i;
      subset_indices[1] = j;
      bool add_edge = (*(this->comp_check_func_))(this->y_set_, subset_indices);
      if (add_edge) {
        g->AddEdgeUnsafe(i, j);
      }
    }

    delete[] subset_indices;
  }

  /**
   * @brief A version of building compatibility graph parallel by vertex
   * @param g
   */
  void BuildCompGraph_parallel_vertex(AdjListGraph* g) {
    size_t N = this->y_set_->size();
    std::vector<std::vector<size_t>> adj_list(N);
    std::atomic<size_t> edge_count = 0;

    // parallel compatibility graph building
    // 1. each thread initialize a queue for storing edges
    // 2. a critical section for dequeuing edges into graph
#pragma omp parallel default(none) shared(g, N, adj_list, edge_count)
    {
      auto* subset_indices = new size_t[2];

#pragma omp for
      // visit each edge twice
      for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
          subset_indices[0] = i;
          subset_indices[1] = j;
          bool add_edge = (*(this->comp_check_func_))(this->y_set_, subset_indices);
          if (add_edge && i != j) {
            adj_list[i].push_back(j);
            edge_count++;
          }
        }
      }

      delete[] subset_indices;
    }

    g->Clear();
    g->SetFromAdjList(std::move(adj_list), edge_count / 2);
  }

  /**
   * @brief Build a compatibility graph in CSR format
   * @param g An empty CSRGraph
   */
  void BuildCSRCompGraph_parallel_two_passes(CSRGraph* g) {
    // first pass: determine the number of edges
    size_t N = this->y_set_->size();
    // offsets array: each entry stores the number of neighbors belonging to that vertex
    // initialize to all zeros
    auto* offsets = new size_t[N + 1];
    memset(offsets, 0, sizeof(size_t) * (N + 1));

    // loop through n(n-1) possibilities without atomic add
#pragma omp parallel default(none) shared(g, N, offsets)
    {
      // subset_indices: a buffer for each thread
      auto* subset_indices = new size_t[2];

#pragma omp for
      // for loop that goes through each (i,j) edge twice
      for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
          subset_indices[0] = i;
          subset_indices[1] = j;
          bool add_edge = (*(this->comp_check_func_))(this->y_set_, subset_indices);
          if (add_edge && i != j) {
            // add one to the offsets array
            offsets[i + 1]++;
          }
        }
      }

      delete[] subset_indices;
    }

    // prefix sum to convert vertex degrees to offsets array for CSR graph
    robin::PrefixSum<size_t>(offsets, offsets, N + 1);

    // allocate the edge array: last entry of offsets array is the (total number of edges * 2)
    size_t edge_count_double = offsets[N];
    auto* edges = new size_t[edge_count_double];

    // vertex_local_offsets: the local offset of each vertex in the edge array. Initialize to zeros.
    auto* vertex_local_offsets = new size_t[N];
    memset(vertex_local_offsets, 0, sizeof(size_t) * N);

    // second pass: insert indices to the edge array
#pragma omp parallel default(none) shared(g, N, edges, offsets, vertex_local_offsets)
    {
      // subset_indices: a buffer for each thread
      auto* subset_indices = new size_t[2];

#pragma omp for
      // for loop that goes through each (i,j) edge twice
      for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
          subset_indices[0] = i;
          subset_indices[1] = j;
          bool add_edge = (*(this->comp_check_func_))(this->y_set_, subset_indices);

          if (add_edge && i != j) {
            // put the edge in the edge array & increment local edge index offset
            edges[offsets[i] + vertex_local_offsets[i]] = j;
            vertex_local_offsets[i]++;
          }
        }
      }

      delete[] subset_indices;
    }

    // now the edge & offsets array are ready to be moved into a CSR graph
    g->SetEdges(edge_count_double, edges);
    g->SetOffsets(N, offsets);

    delete[] vertex_local_offsets;
  }

  /**
   * @brief Build a compatibility graph in CSR format
   * @param g An empty CSRGraph
   */
  void BuildCSRCompGraph_parallel_two_passes_atomic(AtomicCSRGraph* g) {
    // first pass: determine the number of edges
    size_t N = this->y_set_->size();
    // offsets array: each entry stores the number of neighbors belonging to that vertex
    // initialize to all zeros
    auto* offsets = new std::atomic<size_t>[N + 1];
    for (size_t i = 0; i < N + 1; ++i) {
      offsets[i].store(0);
    }

    // for loop through n*(n-1)/2 possibilities and use atomic operations
#pragma omp parallel default(none) shared(g, N, offsets)
    {
      // subset_indices: a buffer for each thread
      auto* subset_indices = new size_t[2];

#pragma omp for
      for (size_t k = 0; k < N * (N - 1) / 2; ++k) {
        size_t i = k / N;
        size_t j = k % N;
        if (j <= i) {
          i = N - i - 2;
          j = N - j - 1;
        }
        subset_indices[0] = i;
        subset_indices[1] = j;
        bool add_edge = (*(this->comp_check_func_))(this->y_set_, subset_indices);
        if (add_edge) {
          offsets[i + 1]++;
          offsets[j + 1]++;
        }
      }

      delete[] subset_indices;
    }

    // prefix sum to convert vertex degrees to offsets array for CSR graph
    robin::PrefixSumAtomic<std::atomic<size_t>>(offsets, offsets, N + 1);

    // allocate the edge array: last entry of offsets array is the total number of edges
    // Note: edge_count_double = 2 * number of edges in the undirected graph
    size_t edge_count_double = offsets[N];
    auto* edges = new std::atomic<size_t>[edge_count_double];

    // vertex_local_offsets: the local offset of each vertex in the edge array. Initialize to zeros.
    auto* vertex_local_offsets = new std::atomic<size_t>[N];
    for (size_t i = 0; i < N; ++i) {
      vertex_local_offsets[i].store(0);
    }

    // second pass: insert indices to the edge array
#pragma omp parallel default(none) shared(g, N, edges, offsets, vertex_local_offsets)
    {
      // subset_indices: a buffer for each thread
      auto* subset_indices = new size_t[2];

#pragma omp for
      for (size_t k = 0; k < N * (N - 1) / 2; ++k) {
        size_t i = k / N;
        size_t j = k % N;
        if (j <= i) {
          i = N - i - 2;
          j = N - j - 1;
        }
        subset_indices[0] = i;
        subset_indices[1] = j;
        bool add_edge = (*(this->comp_check_func_))(this->y_set_, subset_indices);
        if (add_edge) {
          // update edge from side of vertex i
          // put the edge in the edge array & increment local edge index offset
          auto idx_i = vertex_local_offsets[i]++;
          edges[offsets[i].load() + idx_i].store(j);

          // update edge from side of vertex j
          // put the edge in the edge array & increment local edge index offset
          auto idx_j = vertex_local_offsets[j]++;
          edges[offsets[j].load() + idx_j].store(i);        }
      }

      delete[] subset_indices;
    }

    // now the edge & offsets array are ready to be moved into a CSR graph
    g->SetEdges(edge_count_double / 2, edges);
    g->SetOffsets(N, offsets);

    delete[] vertex_local_offsets;
  }

  IGraph* BuildCompGraph(GraphsStorageType storage_type = GraphsStorageType::ADJ_LIST) {
    switch (storage_type) {
    case GraphsStorageType::ADJ_LIST: {
      auto* g = new robin::AdjListGraph();
      BuildCompGraph_parallel_vertex(g);
      return g;
    }
    case GraphsStorageType::ATOMIC_CSR: {
      auto* g = new robin::AtomicCSRGraph();
      BuildCSRCompGraph_parallel_two_passes_atomic(g);
      return g;
    }
    case GraphsStorageType::CSR: {
      auto* g = new robin::CSRGraph();
      BuildCSRCompGraph_parallel_two_passes(g);
      return g;
    }
    }
    return nullptr;
  }
};

} // namespace robin
