#include "morton.h"
#include <algorithm>
#include <numeric>
#include <cstring>
#include <omp.h>

void compute_morton_codes_omp(const std::vector<Vec3>& centroids,
                              const AABB& scene_bounds,
                              std::vector<uint32_t>& codes) {
    Vec3 extent = scene_bounds.hi - scene_bounds.lo;
    float inv_x = (extent.x > 1e-6f) ? 1.0f / extent.x : 0.0f;
    float inv_y = (extent.y > 1e-6f) ? 1.0f / extent.y : 0.0f;
    float inv_z = (extent.z > 1e-6f) ? 1.0f / extent.z : 0.0f;

    int n = static_cast<int>(centroids.size());
    codes.resize(n);
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        float nx = (centroids[i].x - scene_bounds.lo.x) * inv_x;
        float ny = (centroids[i].y - scene_bounds.lo.y) * inv_y;
        float nz = (centroids[i].z - scene_bounds.lo.z) * inv_z;
        codes[i] = morton3D(nx, ny, nz);
    }
}

// 8-bit parallel radix sort (4 passes over 32-bit keys)
void radix_sort_omp(const std::vector<uint32_t>& codes,
                    std::vector<int>& sorted_indices,
                    std::vector<uint32_t>& sorted_codes) {
    const int n = static_cast<int>(codes.size());
    sorted_indices.resize(n);
    sorted_codes.resize(n);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

    if (n <= 1) {
        if (n == 1) sorted_codes[0] = codes[0];
        return;
    }

    constexpr int RADIX_BITS = 8;
    constexpr int NUM_BUCKETS = 1 << RADIX_BITS;
    constexpr uint32_t MASK = NUM_BUCKETS - 1;

    std::vector<int> idx_buf(n);
    int* src_idx = sorted_indices.data();
    int* dst_idx = idx_buf.data();

    int num_threads = omp_get_max_threads();
    std::vector<int> histograms(num_threads * NUM_BUCKETS, 0);

    for (int pass = 0; pass < 4; pass++) {
        int shift = pass * RADIX_BITS;

        std::memset(histograms.data(), 0, sizeof(int) * num_threads * NUM_BUCKETS);

        // Step 1: Build per-thread histograms
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            int* my_hist = &histograms[tid * NUM_BUCKETS];
            #pragma omp for schedule(static)
            for (int i = 0; i < n; i++) {
                uint32_t bucket = (codes[src_idx[i]] >> shift) & MASK;
                my_hist[bucket]++;
            }
        }

        // Step 2: Prefix sum across all threads
        std::vector<int> global_offsets(NUM_BUCKETS, 0);
        for (int b = 0; b < NUM_BUCKETS; b++) {
            for (int t = 0; t < num_threads; t++) {
                global_offsets[b] += histograms[t * NUM_BUCKETS + b];
            }
        }
        int sum = 0;
        for (int b = 0; b < NUM_BUCKETS; b++) {
            int count = global_offsets[b];
            global_offsets[b] = sum;
            sum += count;
        }
        std::vector<int> thread_offsets(num_threads * NUM_BUCKETS);
        for (int b = 0; b < NUM_BUCKETS; b++) {
            int off = global_offsets[b];
            for (int t = 0; t < num_threads; t++) {
                thread_offsets[t * NUM_BUCKETS + b] = off;
                off += histograms[t * NUM_BUCKETS + b];
            }
        }

        // Step 3: Scatter
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            int* my_offsets = &thread_offsets[tid * NUM_BUCKETS];
            #pragma omp for schedule(static)
            for (int i = 0; i < n; i++) {
                uint32_t bucket = (codes[src_idx[i]] >> shift) & MASK;
                dst_idx[my_offsets[bucket]++] = src_idx[i];
            }
        }

        std::swap(src_idx, dst_idx);
    }

    if (src_idx != sorted_indices.data()) {
        std::memcpy(sorted_indices.data(), src_idx, sizeof(int) * n);
    }

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        sorted_codes[i] = codes[sorted_indices[i]];
    }
}
