/**
 * @file morton.h
 * @brief Morton-code helpers and sort interfaces.
 */

#pragma once
#include "vec3.h"
#include "aabb.h"
#include <cstdint>
#include <vector>

/**
 * @brief Expands bits for Morton encoding.
 */
uint32_t expand_bits(uint32_t v);

/**
 * @brief Encodes a normalized 3D point into a Morton code.
 */
uint32_t morton3D(float x, float y, float z);

/**
 * @brief Computes Morton codes in the sequential path.
 */
void compute_morton_codes_seq(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes);
/**
 * @brief Sorts Morton codes in the sequential path.
 */
void radix_sort_seq(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes);

/**
 * @brief Computes Morton codes in the OpenMP path.
 */
void compute_morton_codes_omp(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes);
/**
 * @brief Sorts Morton codes in the OpenMP path.
 */
void radix_sort_omp(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes);
