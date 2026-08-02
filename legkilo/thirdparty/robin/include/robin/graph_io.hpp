// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <cstdint>
#include <iostream>

#include <Eigen/Core>

#include <robin/graph_core.hpp>

namespace robin {

class MatrixMarketReader {
public:
  MatrixMarketReader() = default;

  /**
   * @brief Load a Matrix Market format file to graph
   * @param[in] filepath
   * @return
   */
  AdjListGraph readAdjListGraphFromFile(const std::string& filepath);
};

/**
 * @brief Helper function to print out a graph in an adj matrix
 * @param g
 */
inline void PrintGraph(const AdjListGraph& g) {
  Eigen::MatrixXi adj_mat = Eigen::MatrixXi::Zero(g.VertexCount(), g.VertexCount());

  auto adj_list = g.GetAdjList();
  for (size_t i = 0; i < adj_list.size(); ++i) {
    for (size_t j = 0; j < adj_list[i].size(); ++j) {
      adj_mat(i, adj_list[i][j]) = 1;
    }
  }
  std::cout << adj_mat << std::endl;
}

}
