// Copyright (c) 2020, Massachusetts Institute of Technology,
// Cambridge, MA 02139
// All Rights Reserved
// Authors: Jingnan Shi, et al. (see THANKS for the full author list)
// See LICENSE for the license information

#pragma once
#include <algorithm>
#include <memory>

namespace robin {

/**
 * @brief For interfacing to a raw pointer array with iterators
 * Credit: https://stackoverflow.com/questions/41962903/stliterators-with-raw-pointers
 * @tparam T
 */
template <typename T> class RangeView {
public:
  RangeView(T* data, std::size_t size) : data_{data}, size_{size} {}
  RangeView(const T* data, const std::size_t size) : data_{data}, size_{size} {}

  const T* begin() const { return data_; }
  const T* end() const { return data_ + size_; }

  [[nodiscard]] size_t size() const { return size_; }

  /**
   * @brief [] operator (not safe!)
   * @param i
   * @return
   */
  T& operator[](size_t i) { return data_[i];}

private:
  const T* data_;
  const std::size_t size_;
};

} // namespace robin
