// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <robin/macros.hpp>
#include <robin/utils.hpp>
#include <robin/views.hpp>

namespace robin {

//
// Graph types
//
enum class GraphsStorageType {
  ATOMIC_CSR = 0,
  CSR = 1,
  ADJ_LIST = 2,
};

/**
 * @brief Abstract class interface to represent graphs with some accessors
 */
class IGraph {
public:
  virtual ~IGraph(){};
  virtual size_t VertexCount() const = 0;
  virtual size_t GetVertexDegree(const size_t& i) const = 0;
  virtual size_t GetVertexEdge(const size_t& vertex_id, const size_t& edge_id) const = 0;
  /**
   * @brief Return a vector of edges (no duplicates)
   * @return
   */
  std::vector<std::pair<size_t, size_t>> ToEdgeList() const;

  /**
   * @brief Return a pair holding (offsets array, edges array).
   * Length of the offsets array equals to the total number of vertex plus one.
   * Length of the edges array equals to the total number of edges multiplied by two (undirected
   * graph, each edge has two copies).
   * @return
   */
  std::pair<std::vector<size_t>, std::vector<size_t>> ToCSRArrays() const;
};

/**
 * @brief A light interface for accessing compressed sparse row graph (undirected).
 * Does not support graph mutations.
 */
class CSRGraph : public IGraph {
public:
  /**
   * @brief Construct an empty CSR graph.
   */
  CSRGraph() = default;

  /**
   * @brief Move constructor
   */
  CSRGraph(CSRGraph&& o) noexcept
      : offsets_(o.offsets_), edges_(o.edges_), edge_count_(o.edge_count_),
        vertex_count_(o.vertex_count_) {
    o.offsets_ = nullptr;
    o.edges_ = nullptr;
  }

  /**
   * @brief Initialize CSRGraph with an offset and edge array with known vertex count and edge
   * count. Note that the input pointers will be set back to nullptr to prevent double delete.
   * @param vertex_count
   * @param edge_count
   * @param offsets Reference to pointer to the offsets array. Size expected to be vertex_count + 1
   * @param edges Reference to pointer to the edge array. Size expected to be edge_count * 2
   */
  CSRGraph(size_t vertex_count, size_t edge_count, size_t*& offsets, size_t*& edges)
      : vertex_count_(vertex_count), edge_count_(edge_count) {
    // prevent accidental memory leak
    delete[] offsets_;
    delete[] edges_;
    offsets_ = offsets;
    edges_ = edges;
    offsets = nullptr;
    edges = nullptr;
  }

  /**
   * @brief Convert an edge list to CSR graph.
   * @param edge_list A vector of pair of edges.
   */
  explicit CSRGraph(const std::vector<std::pair<size_t, size_t>>& edge_list);

  /**
   * @brief Initialize the offsets and edge arrays, and update the vertex and edge counts
   * @param vertex_count
   * @param edge_count
   */
  void Allocate(size_t vertex_count, size_t edge_count) {
    offsets_ = new size_t[vertex_count + 1];
    // we store 2 copies of the same edge
    edges_ = new size_t[2 * edge_count];
    // the last entry in the offset array represents the number of edges
    offsets_[vertex_count] = 2 * edge_count;
    vertex_count_ = vertex_count;
    edge_count_ = edge_count;
  }

  /**
   * @brief Get the offset of a vertex (starting index of this vertex's edges in the edge array)
   * @param vertex_index
   * @return
   */
  size_t GetVertexOffset(const size_t& vertex_index) {
    assert(vertex_index < vertex_count_);
    return offsets_[vertex_index];
  }

  /**
   * @brief Set the edge offset of a vertex
   * @param vertex_index
   * @param offset
   */
  void SetVertexOffset(const size_t& vertex_index, const size_t& offset) {
    assert(vertex_index < vertex_count_);
    offsets_[vertex_index] = offset;
  }

  /**
   * @brief Replace the stored pointer to offsets array with the new one
   * @param vertex_count Number of vertices in the graph
   * @param input_offsets
   */
  void SetOffsets(const size_t& vertex_count, size_t*& input_offsets) {
    vertex_count_ = vertex_count;
    delete[] offsets_;
    offsets_ = input_offsets;
    input_offsets = nullptr;
  }

  /**
   * @brief Set the entire offset array
   * @param input_offsets A buffer of size (vertex count + 1)
   */
  void CopyOffsetsFrom(const size_t* input_offsets) {
    std::memcpy(offsets_, input_offsets, sizeof(size_t) * (vertex_count_ + 1));
  }

  /**
   * @brief Return an iterator view of the segment of edge array
   * @param v
   * @return
   */
  // RangeView<size_t> GetVertexEdges(const size_t& v) const override {
  //   // edges_+offsets_[v]: pointer to the starting index of the edges segment in the edges array
  //   for
  //   // vertex v offsets_[v+1] - offsets_[v]: number of edges (size of the edge segment for vertex
  //   v) RangeView<size_t> edge_segment(edges_ + offsets_[v], offsets_[v + 1] - offsets_[v]);
  //   return edge_segment;
  // }

  /**
   * @brief Return an edge of a vertex.
   * @param vertex_id Vertex id. Starts at zero.
   * @param edge_id Edge id. Starts at zero.
   * @return
   */
  size_t GetVertexEdge(const size_t& vertex_id, const size_t& edge_id) const override {
    return edges_[offsets_[vertex_id] + edge_id];
  }

  /**
   * @brief Set the edge array element directly. Make sure you know what you are doing before
   * calling this function.
   * @param offset
   * @param dst
   */
  void SetEdge(const size_t& offset, const size_t& dst_vertex) {
    assert(dst_vertex < vertex_count_);
    edges_[offset] = dst_vertex;
  }

  /**
   * @brief Replace the stored pointer to edges array with the new one
   * @param edge_count Number of edges in the graph
   * @param input_edges An array containing edges (size needs to be equal to 2 * edge_count)
   */
  void SetEdges(const size_t& edge_count, size_t*& input_edges) {
    edge_count_ = edge_count;
    delete[] edges_;
    edges_ = input_edges;
    input_edges = nullptr;
  }

  /**
   * @brief
   * @param input_edges
   */
  void CopyEdgesFrom(const size_t* input_edges) {
    std::memcpy(edges_, input_edges, sizeof(size_t) * (2 * edge_count_));
  }

  /**
   * @brief Copy a buffer containing edges to a vertex's edge segment. Make sure the offset array is
   * already set before calling this function.
   * @param vertex
   * @param vertex_edges
   */
  void CopyVertexEdgesFrom(const size_t& vertex, const size_t* vertex_edges) {
    auto offset = offsets_[vertex];
    auto num_edges = GetVertexDegree(vertex);
    std::memcpy(edges_ + offset, vertex_edges, sizeof(size_t) * num_edges);
  }

  /**
   * @brief Return the degree of the specified vertex
   * @param v
   * @return
   */
  size_t GetVertexDegree(const size_t& v) const override { return offsets_[v + 1] - offsets_[v]; };

  /**
   * Get the number of vertices
   * @return total number of vertices
   */
  [[nodiscard]] size_t VertexCount() const override { return vertex_count_; }

  /**
   * Get the number of edges
   * @return total number of edges
   */
  [[nodiscard]] size_t EdgeCount() const { return edge_count_; }

  /**
   * @brief Destructor for CSRGraph. Delete the two buffers storing edge offsets and edges
   */
  ~CSRGraph() override {
    delete[] offsets_;
    delete[] edges_;
  }

private:
  size_t vertex_count_ = 0;
  size_t edge_count_ = 0;
  size_t* offsets_ = nullptr;
  size_t* edges_ = nullptr;
};

/**
 * @brief A light interface for accessing compressed sparse row graph (undirected).
 * Does not support graph mutations.
 */
class AtomicCSRGraph : public IGraph {
public:
  /**
   * @brief Construct an empty CSR graph.
   */
  AtomicCSRGraph() = default;

  /**
   * @brief Move constructor
   */
  AtomicCSRGraph(AtomicCSRGraph&& o) noexcept
      : offsets_(o.offsets_), edges_(o.edges_), edge_count_(o.edge_count_),
        vertex_count_(o.vertex_count_) {
    o.offsets_ = nullptr;
    o.edges_ = nullptr;
  }

  /**
   * @brief Initialize CSRGraph with an offset and edge array with known vertex count and edge
   * count. Note that the input pointers will be set back to nullptr to prevent double delete.
   * @param vertex_count
   * @param edge_count
   * @param offsets Reference to pointer to the offsets array. Size expected to be vertex_count + 1
   * @param edges Reference to pointer to the edge array. Size expected to be edge_count * 2
   */
  AtomicCSRGraph(size_t vertex_count, size_t edge_count, std::atomic<size_t>*& offsets,
                 std::atomic<size_t>*& edges)
      : vertex_count_(vertex_count), edge_count_(edge_count) {
    // prevent accidental memory leak
    delete[] offsets_;
    delete[] edges_;
    offsets_ = offsets;
    edges_ = edges;
    offsets = nullptr;
    edges = nullptr;
  }

  /**
   * @brief Convert an edge list to CSR graph.
   * @param edge_list A vector of pair of edges.
   */
  explicit AtomicCSRGraph(const std::vector<std::pair<size_t, size_t>>& edge_list);

  /**
   * @brief Initialize the offsets and edge arrays, and update the vertex and edge counts
   * @param vertex_count
   * @param edge_count
   */
  void Allocate(size_t vertex_count, size_t edge_count) {
    offsets_ = new std::atomic<size_t>[vertex_count + 1];
    // we store 2 copies of the same edge
    edges_ = new std::atomic<size_t>[2 * edge_count];
    // the last entry in the offset array represents the number of edges
    offsets_[vertex_count] = 2 * edge_count;
    vertex_count_ = vertex_count;
    edge_count_ = edge_count;
  }

  /**
   * @brief Get the offset of a vertex (starting index of this vertex's edges in the edge array)
   * @param vertex_index
   * @return
   */
  size_t GetVertexOffset(const size_t& vertex_index) {
    assert(vertex_index < vertex_count_);
    return offsets_[vertex_index].load();
  }

  /**
   * @brief Set the edge offset of a vertex
   * @param vertex_index
   * @param offset
   */
  void SetVertexOffset(const size_t& vertex_index, const size_t& offset) {
    assert(vertex_index < vertex_count_);
    offsets_[vertex_index].store(offset);
  }

  /**
   * @brief Replace the stored pointer to offsets array with the new one
   * @param vertex_count Number of vertices in the graph
   * @param input_offsets Offset array, size equals to the number of vertices plus one
   */
  void SetOffsets(const size_t& vertex_count, std::atomic<size_t>*& input_offsets) {
    vertex_count_ = vertex_count;
    delete[] offsets_;
    offsets_ = input_offsets;
    input_offsets = nullptr;
  }

  /**
   * @brief Set the entire offset array
   * @param input_offsets A buffer of size (vertex count + 1)
   */
  void CopyOffsetsFrom(const std::atomic<size_t>* input_offsets) {
    std::memcpy(offsets_, input_offsets, sizeof(std::atomic<size_t>) * (vertex_count_ + 1));
  }

  /**
   * @brief Return an iterator view of the segment of edge array
   * @param v
   * @return
   */
  // RangeView<std::atomic<size_t>> GetVertexEdges(const size_t& v) {
  //   // edges_+offsets_[v]: pointer to the starting index of the edges segment in the edges array
  //   for
  //   // vertex v offsets_[v+1] - offsets_[v]: number of edges (size of the edge segment for vertex
  //   v) RangeView<std::atomic<size_t>> edge_segment(edges_ + offsets_[v], offsets_[v + 1] -
  //   offsets_[v]); return edge_segment;
  // }

  /**
   * @brief Return an edge of a vertex.
   * @param vertex_id Vertex id. Starts at zero.
   * @param edge_id Edge id. Starts at zero.
   * @return
   */
  size_t GetVertexEdge(const size_t& vertex_id, const size_t& edge_id) const override {
    return edges_[offsets_[vertex_id].load() + edge_id].load();
  }

  /**
   * @brief Set the edge array element directly. Make sure you know what you are doing before
   * calling this function.
   * @param offset
   * @param dst
   */
  void SetEdge(const size_t& offset, const size_t& dst_vertex) {
    assert(dst_vertex < vertex_count_);
    edges_[offset].store(dst_vertex);
  }

  /**
   * @brief Replace the stored pointer to edges array with the new one
   * @param edge_count Number of edges in the undirected graph
   * @param input_edges An array containing edges (size needs to be equal to 2 * edge_count)
   */
  void SetEdges(const size_t& edge_count, std::atomic<size_t>*& input_edges) {
    edge_count_ = edge_count;
    delete[] edges_;
    edges_ = input_edges;
    input_edges = nullptr;
  }

  /**
   * @brief
   * @param input_edges
   */
  void CopyEdgesFrom(const std::atomic<size_t>* input_edges) {
    std::memcpy(edges_, input_edges, sizeof(std::atomic<size_t>) * (2 * edge_count_));
  }

  /**
   * @brief Copy a buffer containing edges to a vertex's edge segment. Make sure the offset array is
   * already set before calling this function.
   * @param vertex
   * @param vertex_edges
   */
  void CopyVertexEdgesFrom(const size_t& vertex, const std::atomic<size_t>* vertex_edges) {
    auto offset = offsets_[vertex].load();
    auto num_edges = GetVertexDegree(vertex);
    std::memcpy(edges_ + offset, vertex_edges, sizeof(std::atomic<size_t>) * num_edges);
  }

  /**
   * @brief Return the degree of the specified vertex
   * @param v
   * @return
   */
  size_t GetVertexDegree(const size_t& v) const override {
    return offsets_[v + 1].load() - offsets_[v].load();
  };

  /**
   * Get the number of vertices
   * @return total number of vertices
   */
  [[nodiscard]] size_t VertexCount() const override { return vertex_count_; }

  /**
   * Get the number of edges
   * @return total number of edges
   */
  [[nodiscard]] size_t EdgeCount() const { return edge_count_; }

  /**
   * @brief Print the edge array
   */
  void PrintEdgeArray() const {
    std::cout << "[";
    for (size_t i = 0; i < edge_count_ * 2; ++i) {
      std::cout << edges_[i].load();
      if (i != edge_count_ * 2 - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }

  /**
   * @brief Print the offsets array
   */
  void PrintOffsetsArray() const {
    std::cout << "[";
    for (size_t i = 0; i < vertex_count_ + 1; ++i) {
      std::cout << offsets_[i].load();
      if (i != vertex_count_) {
        std::cout << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }

  /**
   * @brief Destructor for CSRGraph. Delete the two buffers storing edge offsets and edges
   */
  ~AtomicCSRGraph() {
    delete[] offsets_;
    delete[] edges_;
  }

private:
  size_t vertex_count_ = 0;
  size_t edge_count_ = 0;
  std::atomic<size_t>* offsets_ = nullptr;
  std::atomic<size_t>* edges_ = nullptr;
};

/**
 * A simple undirected graph class
 *
 * This graph assumes that vertices are numbered. In addition, the vertices numbers have to be
 * consecutive starting from 0.
 *
 * For example, if the graph have 3 vertices, they have to be named 0, 1, and 2.
 */
class AdjListGraph : public IGraph {
public:
  AdjListGraph() : num_edges_(0){};

  /**
   * Constructor that takes in an adjacency list. Notice that for an edge connecting two arbitrary
   * vertices v1 & v2, we assume that v2 exists in v1's list, and v1 also exists in v2's list. This
   * condition is not enforced. If violated, RemoveEdge() function might exhibit undefined
   * behaviors.
   * @param [in] adj_list an map representing an adjacency list
   */
  explicit AdjListGraph(const std::map<size_t, std::vector<size_t>>& adj_list);

  /**
   * @brief Copy and move constructor
   * @param adj_list
   * @param edge_count
   */
  AdjListGraph(std::vector<std::vector<size_t>> adj_list, size_t edge_count)
      : adj_list_(std::move(adj_list)), num_edges_(edge_count) {}
  AdjListGraph(std::vector<std::vector<size_t>>&& adj_list, size_t edge_count)
      : adj_list_(std::move(adj_list)), num_edges_(edge_count) {}

  /**
   * Add a vertex with no edges.
   * @param [in] id the id of vertex to be added
   */
  void AddVertex(const size_t& id) {
    if (id < adj_list_.size()) {
      ROBIN_DEBUG_ERROR_MSG("Vertex already exists.");
    } else {
      adj_list_.resize(id + 1);
    }
  }

  /**
   * Add an edge between two vertices
   * @param [in] vertex_1 one vertex of the edge
   * @param [in] vertex_2 another vertex of the edge
   */
  void AddEdge(const size_t& vertex_1, const size_t& vertex_2);

  /**
   * Add an edge between two vertices without checking whether the edge exists
   * @param [in] vertex_1 one vertex of the edge
   * @param [in] vertex_2 another vertex of the edge
   */
  void AddEdgeUnsafe(const size_t& vertex_1, const size_t& vertex_2);

  /**
   * Populate the graph with the provided number of vertices without any edges.
   * @param num_vertices
   */
  void PopulateVertices(const size_t& num_vertices) { adj_list_.resize(num_vertices); }

  /**
   * Return true if said edge exists
   * @param [in] vertex_1
   * @param [in] vertex_2
   */
  bool HasEdge(const size_t& vertex_1, const size_t& vertex_2);

  /**
   * Return true if the vertex exists.
   * @param vertex
   * @return
   */
  bool HasVertex(const size_t& vertex) { return vertex < adj_list_.size(); }

  /**
   * Remove the edge between two vertices.
   * @param [in] vertex_1 one vertex of the edge
   * @param [in] vertex_2 another vertex of the edge
   */
  void RemoveEdge(const size_t& vertex_1, const size_t& vertex_2);

  /**
   * @brief Set graph from an adjacency list and number of edges
   * @param adj_list
   */
  void SetFromAdjList(std::vector<std::vector<size_t>>&& adj_list, size_t num_edges) {
    adj_list_ = std::move(adj_list);
    num_edges_ = num_edges;
  }

  /**
   * Get the number of vertices
   * @return total number of vertices
   */
  [[nodiscard]] size_t VertexCount() const override { return adj_list_.size(); }

  /**
   * Get the number of edges
   * @return total number of edges
   */
  [[nodiscard]] size_t EdgeCount() const { return num_edges_; }

  /**
   * Get edges originated from a specific vertex
   * @param[in] id
   * @return an unordered set of edges
   */
  //[[nodiscard]] robin::RangeView<size_t> GetVertexEdges(size_t id) const {
  //  auto* ptr = adj_list_[id].data();
  //  robin::RangeView<size_t> result(ptr, adj_list_[id].size());
  //  return result;
  //}

  /**
   * @brief Return an edge of a vertex.
   * @param vertex_id Vertex id. Starts at zero.
   * @param edge_id Edge id. Starts at zero.
   * @return
   */
  size_t GetVertexEdge(const size_t& vertex_id, const size_t& edge_id) const override {
    return adj_list_[vertex_id][edge_id];
  }

  /**
   * @brief Get the degree of a specific vertex
   * @param[in] id
   * @return degree of the vertex
   */
  [[nodiscard]] size_t GetVertexDegree(const size_t& id) const override {
    return adj_list_[id].size();
  }

  /**
   * Get all vertices
   * @return a vector of all vertices
   */
  [[nodiscard]] std::vector<int> GetVertices() const;

  [[nodiscard]] const std::vector<std::vector<size_t>>& GetAdjList() const { return adj_list_; }

  /**
   * @brief Return the adjacency matrix of the graph
   * @return
   */
  Eigen::MatrixXd GetAdjMat() const;

  /**
   * Reserve space for complete graph. A complete undirected graph should have N*(N-1)/2 edges
   * @param num_vertices
   */
  void ReserveForCompleteGraph(const int& num_vertices);

  /**
   * Preallocate spaces for vertices
   * @param num_vertices
   */
  void reserve(const int& num_vertices) { adj_list_.reserve(num_vertices); }

  /**
   * @brief Remove all vertices and set number of edges to zero
   */
  void Clear() {
    adj_list_.clear();
    num_edges_ = 0;
  }

  /**
   * @brief Generate a random graph using the Erdos-Renyi graph G(num_vertices, prob)
   * The graph is constructed by connecting nodes randomly. Each edge is included in the graph with
   * probability prob independent from every other edge.
   *
   * @return a Graph object
   */
  static AdjListGraph Random(size_t num_vertices, double prob = 0.1);

private:
  std::vector<std::vector<size_t>> adj_list_;
  size_t num_edges_;
};

} // namespace robin
