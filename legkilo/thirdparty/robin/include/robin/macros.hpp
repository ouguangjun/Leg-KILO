// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once

#include <iostream>

#define ROBIN_INFO_MSG(x)                                                                          \
  do {                                                                                             \
    std::cout << x;                                                                                \
  } while (0)

#define ROBIN_INFO_MSG_THROTTLE(x, i, N)                                                           \
  do {                                                                                             \
    if (i % N == 0) {                                                                              \
      std::cout << x;                                                                              \
    }                                                                                              \
  } while (0)

#if defined(NDEBUG) && !defined(ROBIN_DIAG_PRINT)
// Use NOOP to turn off the defined debug macros
#define ROBIN_DEBUG_ERROR_MSG(x)                                                                   \
  do {                                                                                             \
  } while (0)
#define ROBIN_DEBUG_INFO_MSG(x)                                                                    \
  do {                                                                                             \
  } while (0)
// Timing macros
#define ROBIN_DEBUG_DECLARE_TIMING(s)                                                              \
  do {                                                                                             \
  } while (0)
#define ROBIN_DEBUG_START_TIMING(s)                                                                \
  do {                                                                                             \
  } while (0)
#define ROBIN_DEBUG_STOP_TIMING(s)                                                                 \
  do {                                                                                             \
  } while (0)
#define ROBIN_DEBUG_GET_TIMING(s)                                                                  \
  do {                                                                                             \
  } while (0)
#else
// Debug messages
#define ROBIN_DEBUG_ERROR_MSG(x)                                                                   \
  do {                                                                                             \
    std::cerr << __FILE__ << " L" << __LINE__ << ":" << x << std::endl;                                    \
  } while (0)
#define ROBIN_DEBUG_INFO_MSG(x)                                                                    \
  do {                                                                                             \
    std::cout << x << std::endl;                                                                   \
  } while (0)
// Timing macros
#define ROBIN_DEBUG_DECLARE_TIMING(s) std::chrono::steady_clock clock_##s;
#define ROBIN_DEBUG_START_TIMING(s) auto t_start_##s = clock_##s.now();
#define ROBIN_DEBUG_STOP_TIMING(s)                                                                 \
  auto t_end_##s = clock_##s.now();                                                                \
  std::chrono::duration<double, std::milli> diff_dur_##s = t_end_##s - t_start_##s;                \
  double diff_##s = diff_dur_##s.count();
#define ROBIN_DEBUG_GET_TIMING(s) (double)(diff_##s / 1000.0)
#endif