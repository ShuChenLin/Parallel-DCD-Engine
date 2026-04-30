#pragma once
#include "vec3.h"
#include "aabb.h"
#include <cstdint>
#include <vector>

uint32_t expand_bits(uint32_t v);

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
