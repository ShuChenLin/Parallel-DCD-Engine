#pragma once
#include "vec3.h"
#include "aabb.h"
#include <cstdint>
#include <vector>

// Expand a 10-bit integer into 30 bits by inserting 2 zeros before each bit
uint32_t expand_bits(uint32_t v);

// Compute 30-bit Morton code for a point normalized to [0,1]^3
uint32_t morton3D(float x, float y, float z);

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
void radix_sort_omp(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes);
