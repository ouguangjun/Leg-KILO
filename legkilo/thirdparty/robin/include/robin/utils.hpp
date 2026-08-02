// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace robin {


/**
 * @brief Templated serial prefix sum function
 * @tparam T Type of which the pointers are pointing to
 * @param input Pointer to the input array (array to be prefix summed)
 * @param output Pointer to the output array
 * @param array_size Size of the input array
 */
template <typename T>
void PrefixSum(const T* input, T* output, const size_t& array_size) {
  output[0] = input[0];
  for (size_t i = 1; i < array_size; ++i) {
    output[i] = output[i-1] + input[i];
  }
}

template <typename T>
void PrefixSumAtomic(const T* input, T* output, const size_t& array_size) {
  output[0].store(input[0].load());
  for (size_t i = 1; i < array_size; ++i) {
    output[i] = output[i-1] + input[i];
  }
}

/**
 * @brief Thread guard for ensuring thread exception safety
 */
class ThreadGuard
{
  std::thread& t;
public:
  explicit ThreadGuard(std::thread& t_):
      t(t_)
  {}

  ~ThreadGuard()
  {
    if(t.joinable())
    {
      t.join();
    }
  }

  ThreadGuard(ThreadGuard const&)=delete;
  ThreadGuard& operator=(ThreadGuard const&)=delete;
};

} // namespace robin
