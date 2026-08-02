// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <robin/graph.hpp>
#include <string>

namespace robin {

/**
 * Calculate (n choose k), also known as the binomial coefficient
 * @param n
 * @param k
 */
inline size_t Choose(const size_t& n, const size_t& k) {
  // reduce number of iterations
  size_t new_k = k;
  if (k > n - k) {
    new_k = n - k;
  }

  // calculate the result
  size_t res = 1;
  for (size_t i = 0; i < new_k; ++i) {
    res *= (n - i);
    res /= (i + 1);
  }

  return res;
}

/**
 * @brief get the [x]th lexicographically ordered set of [p] elements in [n]
 * output is in [c], and should be sizeof(int)*[p]
 *
 * Credit: https://stackoverflow.com/questions/561/how-to-use-combinations-of-sets-as-test-data#794
 *
 * Note: this implementation has zero-based indices.
 *
 * Reference:
 * "Algorithm 515: Generation of a Vector from the Lexicographical Index"; Buckles, B. P., and
 * Lybanon, M. ACM Transactions on Mathematical Software, Vol. 3, No. 2, June 1977.
 *
 * @param c
 * @param n
 * @param p
 * @param x
 */
inline void CombinationDecode(size_t n, size_t p, size_t x, size_t* c) {
  x += 1;
  int i, r, k = 0;
  int c_temp = 0;
  for (i = 0; i < p - 1; i++) {
    c[i] = (i != 0) ? c_temp : 0;
    do {
      c[i]++;
      r = Choose(n - c[i], p - (i + 1));
      k = k + r;
    } while (k < x);
    k = k - r;
    c_temp = c[i];
    c[i] = c[i] - 1;
  }
  c[p - 1] = c[p - 2] + x - k;
}

/**
 * @brief Utility function to check whether the provided matrix is in the SO(3) group
 * @param mat
 * @return
 */
inline bool IsSo3(const Eigen::Matrix3d& mat) {
  bool identity_check = (mat.transpose() * mat).isApprox(Eigen::Matrix3d::Identity());
  bool determinant_check = std::abs(mat.determinant() - 1) < 0.001;
  return identity_check & determinant_check;
}

/**
 * Return the Euclidean distance between the two provided Eigen vectors
 * @param a an Eigen vector
 * @param b an Eigen vector
 * @return
 */
template <typename Derived>
inline double EuclideanDistance(const Eigen::MatrixBase<Derived>& a,
                                const Eigen::MatrixBase<Derived>& b) {
  return (a - b).norm();
}

/**
 * Return the cosine similarity between two provided Eigen vectors
 * @param a
 * @param b
 * @return
 */
template <typename Derived>
inline double CosineSimilarity(const Eigen::MatrixBase<Derived>& a,
                               const Eigen::MatrixBase<Derived>& b) {
  double diff_angle = a.dot(b) / (a.norm() * b.norm());

  if (diff_angle > 1) {
    diff_angle = 1;
  } else if (diff_angle < -1) {
    diff_angle = -1;
  }
  return diff_angle;
}

} // namespace robin