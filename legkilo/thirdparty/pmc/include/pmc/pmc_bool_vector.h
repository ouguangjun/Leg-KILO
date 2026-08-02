#ifndef PMC_BOOL_VECTOR_H_
#define PMC_BOOL_VECTOR_H_

#include <cstdint>
#include <vector>

namespace pmc {

/// A bare minimum implementation of a boolean vector.
///
/// This class is recommended in place of std::vector<bool> or std::vector<int> for thread-safety.
/// std::vector<bool> could cause a race condition if it is implemented as a dynamic bitset.
/// std::vector<int> is not memory efficient and misleading as an element can hold other than 0 or 1.
class bool_wrapper {
 public:
  /// NOTE: It should not be marked `explicit` to allow implicit conversion from `bool` to `bool_wrapper`.
  bool_wrapper(bool value = false) : byte_(to_int(value)) {}

  operator bool() const noexcept { return byte_ != 0; }

  bool_wrapper& operator=(bool value) noexcept {
    byte_ = to_int(value);
    return *this;
  }

 private:
  static constexpr std::uint8_t to_int(bool value) noexcept {
    return value ? 1 : 0;
  }

  std::uint8_t byte_;
};

using bool_vector = std::vector<bool_wrapper>;

} // namespace pmc

#endif