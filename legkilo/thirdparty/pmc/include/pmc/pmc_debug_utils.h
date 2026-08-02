//
// Created by jnshi on 10/2/22.
//

#ifndef PMC_DEBUG_UTILS_H_
#define PMC_DEBUG_UTILS_H_

template <class... Args>
constexpr void discard(Args&&... /*args*/) {}

#ifdef PMC_ENABLE_DEBUG
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) discard(__VA_ARGS__)
#endif

#endif //PMC_DEBUG_UTILS_H_
