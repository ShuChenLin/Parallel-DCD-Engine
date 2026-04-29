#pragma once
#include "vec3.h"
#include "aabb.h"
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>

// Expand a 10-bit integer into 30 bits by inserting 2 zeros before each bit
inline uint32_t expand_bits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

// Compute 30-bit Morton code for a point normalized to [0,1]^3
inline uint32_t morton3D(float x, float y, float z) {
    x = std::min(std::max(x * 1024.0f, 0.0f), 1023.0f);
    y = std::min(std::max(y * 1024.0f, 0.0f), 1023.0f);
    z = std::min(std::max(z * 1024.0f, 0.0f), 1023.0f);
    uint32_t xx = expand_bits(static_cast<uint32_t>(x));
    uint32_t yy = expand_bits(static_cast<uint32_t>(y));
    uint32_t zz = expand_bits(static_cast<uint32_t>(z));
    return (xx * 4) + (yy * 2) + zz;
}

<<<<<<< Updated upstream
// Sequential morton + sort
void compute_morton_codes_seq(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes);
void radix_sort_seq(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes);

// OpenMP morton + sort
void compute_morton_codes_omp(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes);
=======
// --- Sequential versions ---
void compute_morton_codes_seq(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes);

void radix_sort_seq(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes);

// --- OpenMP versions ---
void compute_morton_codes_omp(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes);

>>>>>>> Stashed changes
void radix_sort_omp(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes);
