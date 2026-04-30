/**
 * @file morton_seq.cpp
 * @brief Sequential Morton-code generation and sorting.
 */

#include "morton.h"
#include <algorithm>
#include <numeric>

/**
 * @brief Computes Morton codes for all centroids in the sequential path.
 */
void compute_morton_codes_seq(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes) {
    Vec3 extent = scene_bounds.hi - scene_bounds.lo;
    float inv_x = (extent.x > 1e-6f) ? 1.0f / extent.x : 0.0f;
    float inv_y = (extent.y > 1e-6f) ? 1.0f / extent.y : 0.0f;
    float inv_z = (extent.z > 1e-6f) ? 1.0f / extent.z : 0.0f;

    codes.resize(centroids.size());
    for (size_t i = 0; i < centroids.size(); i++) {
        float nx = (centroids[i].x - scene_bounds.lo.x) * inv_x;
        float ny = (centroids[i].y - scene_bounds.lo.y) * inv_y;
        float nz = (centroids[i].z - scene_bounds.lo.z) * inv_z;
        codes[i] = morton3D(nx, ny, nz);
    }
}

/**
 * @brief Sorts object indices by Morton code with std::sort.
 */
void radix_sort_seq(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes) {
    const int n = static_cast<int>(codes.size());
    sorted_indices.resize(n);
    sorted_codes.resize(n);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

    // Simple std::sort by morton code
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&codes](int a, int b) { return codes[a] < codes[b]; });

    for (int i = 0; i < n; i++) {
        sorted_codes[i] = codes[sorted_indices[i]];
    }
}
