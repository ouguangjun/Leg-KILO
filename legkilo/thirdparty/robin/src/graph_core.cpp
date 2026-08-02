// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#include <random>

#include <robin/graph_core.hpp>

namespace robin {

//
// Definitions for CSRGraph class methods
//
CSRGraph::CSRGraph(const std::vector<std::pair<size_t, size_t>>& edge_list) {

  // first pass: number of vertices
  std::map<size_t, std::vector<size_t>> edge_map;
  for (const auto& e : edge_list) {
    const auto& k1 = e.first;
    const auto& k2 = e.second;
    bool has_k1 = edge_map.count(k1);
    bool has_k2 = edge_map.count(k2);

    // if k1 does not exist, create the vector
    if (!has_k1) {
      edge_map.insert({k1, {k2}});
    } else {
      edge_map[k1].push_back(k2);
    }

    // if k2 does not exist, create the vector
    if (!has_k2) {
      edge_map.insert({k2, {k1}});
    } else {
      edge_map[k2].push_back(k1);
    }
    // after this loop we should have two copies of the same edge
  }

  // allocate edges and offsets
  auto vertex_count = edge_map.size();
  auto edge_count = edge_list.size();
  Allocate(vertex_count, edge_count);

  // create the offsets array
  // first, we have a temp array containing the number of edges for each vertex
  // temp[0] = 0
  // temp[i] = number of edges of vertex i-1
  offsets_[0] = 0;
  for (size_t i = 0; i < vertex_count; ++i) {
    offsets_[i + 1] = edge_map[i].size();
  }

  // prefix sum to obtain edge offsets
  robin::PrefixSum<size_t>(offsets_, offsets_, vertex_count + 1);

  // for each vertex:
  // keep the local offset index
  // insert edge at local offset index
  // increase local offset index
  std::map<size_t, size_t> local_offset_map;
  for (const auto& e : edge_list) {
    const size_t& k1 = e.first;
    const size_t& k2 = e.second;

    const size_t local_offset_idx_k1 = local_offset_map[k1];
    edges_[offsets_[k1] + local_offset_idx_k1] = k2;

    const size_t local_offset_idx_k2 = local_offset_map[k2];
    edges_[offsets_[k2] + local_offset_idx_k2] = k1;

    local_offset_map[k1]++;
    local_offset_map[k2]++;
  }
}

//
// Definitions for Graph class methods
//
AdjListGraph::AdjListGraph(const std::map<size_t, std::vector<size_t>>& adj_list) {
  adj_list_.resize(adj_list.size());
  num_edges_ = 0;
  for (const auto& e_list : adj_list) {
    adj_list_[e_list.first] = e_list.second;
    num_edges_ += e_list.second.size();
  }
  num_edges_ /= 2;
}

void AdjListGraph::ReserveForCompleteGraph(const int& num_vertices) {
  adj_list_.resize(num_vertices);
  for (int i = 0; i < num_vertices - 1; ++i) {
    adj_list_[i].reserve(num_vertices - 1);
  }
}

std::vector<int> AdjListGraph::GetVertices() const {
  std::vector<int> v;
  v.reserve(adj_list_.size());
  for (int i = 0; i < adj_list_.size(); ++i) {
    v.push_back(i);
  }
  return v;
}

void AdjListGraph::RemoveEdge(const size_t& vertex_1, const size_t& vertex_2) {
  if (vertex_1 >= adj_list_.size() || vertex_2 >= adj_list_.size()) {
    ROBIN_DEBUG_ERROR_MSG("Trying to remove non-existent edge.");
    return;
  }

  adj_list_[vertex_1].erase(
      std::remove(adj_list_[vertex_1].begin(), adj_list_[vertex_1].end(), vertex_2),
      adj_list_[vertex_1].end());
  adj_list_[vertex_2].erase(
      std::remove(adj_list_[vertex_2].begin(), adj_list_[vertex_2].end(), vertex_1),
      adj_list_[vertex_2].end());

  num_edges_--;
}

bool AdjListGraph::HasEdge(const size_t& vertex_1, const size_t& vertex_2) {
  if (vertex_1 >= adj_list_.size() || vertex_2 >= adj_list_.size()) {
    return false;
  }
  auto& connected_vs = adj_list_[vertex_1];
  bool exists = std::find(connected_vs.begin(), connected_vs.end(), vertex_2) != connected_vs.end();
  return exists;
}

AdjListGraph AdjListGraph::Random(size_t num_vertices, double prob) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0, 1);
  AdjListGraph g;
  // add all vertices
  for (size_t i = 0; i < num_vertices; ++i) {
    g.AddVertex(i);
  }
  // add edges
  for (size_t i = 0; i < num_vertices - 1; ++i) {
    for (size_t j = i + 1; j < num_vertices; ++j) {
      bool add_edge = dist(gen) < prob;
      if (add_edge) {
        g.AddEdge(i, j);
      }
    }
  }
  return g;
}
Eigen::MatrixXd AdjListGraph::GetAdjMat() const {
  Eigen::MatrixXd adj_mat(this->VertexCount(), this->VertexCount());
  adj_mat = Eigen::MatrixXd::Zero(this->VertexCount(), this->VertexCount());
  for (size_t src_vert = 0; src_vert < this->VertexCount(); ++src_vert) {
    for (const auto& dst_vert : this->GetAdjList()[src_vert]) {
      adj_mat(src_vert, dst_vert) = 1;
    }
  }
  return adj_mat;
}

void AdjListGraph::AddEdge(const size_t& vertex_1, const size_t& vertex_2) {
  if (!HasVertex(vertex_1) || !HasVertex(vertex_2)) {
    ROBIN_DEBUG_ERROR_MSG("One of the edge vertices does not exists.");
    return;
  }
  if (HasEdge(vertex_1, vertex_2)) {
    ROBIN_DEBUG_ERROR_MSG("Edge exists.");
    return;
  }
  adj_list_[vertex_1].push_back(vertex_2);
  adj_list_[vertex_2].push_back(vertex_1);
  num_edges_++;
}

void AdjListGraph::AddEdgeUnsafe(const size_t& vertex_1, const size_t& vertex_2) {
  adj_list_[vertex_1].push_back(vertex_2);
  adj_list_[vertex_2].push_back(vertex_1);
  num_edges_++;
}

std::vector<std::pair<size_t, size_t>> IGraph::ToEdgeList() const {
  std::set<std::pair<size_t, size_t>> edge_set;
  auto N = VertexCount();
  for (size_t i = 0; i < N; ++i) {
    auto v_deg = GetVertexDegree(i);
    for (size_t j = 0; j < v_deg; ++j) {
      auto c_edge_dst = GetVertexEdge(i, j);
      auto c_edge_src = i;
      std::pair<size_t, size_t> edge;
      if (c_edge_src <= c_edge_dst) {
        edge = {c_edge_src, c_edge_dst};
      } else {
        edge = {c_edge_dst, c_edge_src};
      }
      edge_set.insert(edge);
    }
  }
  std::vector<std::pair<size_t, size_t>> result(edge_set.begin(), edge_set.end());
  return result;
}

std::pair<std::vector<size_t>, std::vector<size_t>> IGraph::ToCSRArrays() const {
  auto edge_list = ToEdgeList();

  // first pass: number of vertices
  std::map<size_t, std::vector<size_t>> edge_map;
  for (const auto& e : edge_list) {
    const auto& k1 = e.first;
    const auto& k2 = e.second;
    bool has_k1 = edge_map.count(k1);
    bool has_k2 = edge_map.count(k2);

    // if k1 does not exist, create the vector
    if (!has_k1) {
      edge_map.insert({k1, {k2}});
    } else {
      edge_map[k1].push_back(k2);
    }

    // if k2 does not exist, create the vector
    if (!has_k2) {
      edge_map.insert({k2, {k1}});
    } else {
      edge_map[k2].push_back(k1);
    }
    // after this loop we should have two copies of the same edge
  }

  // allocate edges and offsets
  auto vertex_count = VertexCount();
  auto edge_count = edge_list.size();

  // std::vector<size_t> offsets(vertex_count + 1);
  // std::vector<size_t> edges(2 * edge_count);

  auto* offsets = new size_t[vertex_count + 1];
  // we store 2 copies of the same edge
  auto* edges = new size_t[2 * edge_count];
  // the last entry in the offset array represents the number of edges
  offsets[vertex_count] = 2 * edge_count;

  // create the offsets array
  // first, we have a temp array containing the number of edges for each vertex
  // temp[0] = 0
  // temp[i] = number of edges of vertex i-1
  offsets[0] = 0;
  for (size_t i = 0; i < vertex_count; ++i) {
    offsets[i + 1] = edge_map[i].size();
  }

  // prefix sum to obtain edge offsets
  robin::PrefixSum<size_t>(offsets, offsets, vertex_count + 1);

  // for each vertex:
  // keep the local offset index
  // insert edge at local offset index
  // increase local offset index
  std::map<size_t, size_t> local_offset_map;

  // initialize local offset map to zeros
  for (size_t i = 0; i < vertex_count; ++i) {
    local_offset_map.insert({i, 0});
  }

  for (const auto& e : edge_list) {
    const size_t& k1 = e.first;
    const size_t& k2 = e.second;

    const size_t local_offset_idx_k1 = local_offset_map[k1];
    edges[offsets[k1] + local_offset_idx_k1] = k2;
    local_offset_map[k1]++;

    const size_t local_offset_idx_k2 = local_offset_map[k2];
    // TODO: fix seg fault, k2 sometimes greater than length of offsets array
    edges[offsets[k2] + local_offset_idx_k2] = k1;
    local_offset_map[k2]++;
  }

  std::pair<std::vector<size_t>, std::vector<size_t>> result;
  result.first.assign(offsets, offsets + vertex_count + 1);
  result.second.assign(edges, edges + edge_count * 2);

  delete[] offsets;
  delete[] edges;

  return result;
}
} // namespace robin