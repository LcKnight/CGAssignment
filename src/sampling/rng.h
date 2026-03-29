#pragma once
#include <cstdint>

// xorshift32 fast RNG — one state per thread, no locking needed
inline uint32_t xorshift(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s <<  5;
    return s;
}
// Returns uniform float in [0, 1)
inline float randf(uint32_t& s) {
    return (xorshift(s) & 0xFFFFFF) / float(0xFFFFFF);
}