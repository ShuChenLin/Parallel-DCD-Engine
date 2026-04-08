#include "morton.h"
#include <algorithm>
#include <numeric>
#include <cstring>
#include <omp.h>

uint32_t expand_bits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

uint32_t morton3D(float x, float y, float z) {
    x = std::min(std::max(x * 1024.0f, 0.0f), 1023.0f);
    y = std::min(std::max(y * 1024.0f, 0.0f), 1023.0f);
    z = std::min(std::max(z * 1024.0f, 0.0f), 1023.0f);
    uint32_t xx = expand_bits(static_cast<uint32_t>(x));
    uint32_t yy = expand_bits(static_cast<uint32_t>(y));
    uint32_t zz = expand_bits(static_cast<uint32_t>(z));
    return (xx * 4) + (yy * 2) + zz;
}

void compute_morton_codes(const std::vector<Vec3>& centroids,
                          const AABB& scene_bounds,
                          std::vector<uint32_t>& codes) {
    Vec3 extent = scene_bounds.hi - scene_bounds.lo;
    // Avoid division by zero
    float inv_x = (extent.x > 1e-6f) ? 1.0f / extent.x : 0.0f;
    float inv_y = (extent.y > 1e-6f) ? 1.0f / extent.y : 0.0f;
    float inv_z = (extent.z > 1e-6f) ? 1.0f / extent.z : 0.0f;

    codes.resize(centroids.size());
    // OpenMP parallel: each Morton code is independent
    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < centroids.size(); i++) {
        float nx = (centroids[i].x - scene_bounds.lo.x) * inv_x;
        float ny = (centroids[i].y - scene_bounds.lo.y) * inv_y;
        float nz = (centroids[i].z - scene_bounds.lo.z) * inv_z;
        codes[i] = morton3D(nx, ny, nz);
    }
}

// 8-bit radix sort (4 passes over 32-bit keys), parallelized with OpenMP.
// Sorts indices[0..n) by codes[indices[i]].
void parallel_radix_sort(const std::vector<uint32_t>& codes,
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
    constexpr int NUM_BUCKETS = 1 << RADIX_BITS;  // 256
    constexpr uint32_t MASK = NUM_BUCKETS - 1;

    // Double-buffer: swap src/dst each pass
    std::vector<int> idx_buf(n);
    int* src_idx = sorted_indices.data();
    int* dst_idx = idx_buf.data();

    int num_threads = omp_get_max_threads();
    // Per-thread histogram: [thread][bucket]
    std::vector<int> histograms(num_threads * NUM_BUCKETS, 0);

    for (int pass = 0; pass < 4; pass++) {
        int shift = pass * RADIX_BITS;

        // Clear histograms
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

        // Step 2: Prefix sum across all threads for global offsets
        // global_offsets[bucket] = where bucket starts in output
        std::vector<int> global_offsets(NUM_BUCKETS, 0);
        // First compute total count per bucket
        for (int b = 0; b < NUM_BUCKETS; b++) {
            for (int t = 0; t < num_threads; t++) {
                global_offsets[b] += histograms[t * NUM_BUCKETS + b];
            }
        }
        // Exclusive prefix sum
        int sum = 0;
        for (int b = 0; b < NUM_BUCKETS; b++) {
            int count = global_offsets[b];
            global_offsets[b] = sum;
            sum += count;
        }
        // Compute per-thread write offsets:
        // thread t writes bucket b starting at global_offsets[b] + sum of histograms[0..t-1][b]
        // Store back into histograms as the write position for each thread
        std::vector<int> thread_offsets(num_threads * NUM_BUCKETS);
        for (int b = 0; b < NUM_BUCKETS; b++) {
            int off = global_offsets[b];
            for (int t = 0; t < num_threads; t++) {
                thread_offsets[t * NUM_BUCKETS + b] = off;
                off += histograms[t * NUM_BUCKETS + b];
            }
        }

        // Step 3: Scatter (parallel, each thread writes to its own range)
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

        // Swap buffers
        std::swap(src_idx, dst_idx);
    }

    // After 4 passes (even number), result is in sorted_indices
    // If src_idx != sorted_indices.data(), copy back
    if (src_idx != sorted_indices.data()) {
        std::memcpy(sorted_indices.data(), src_idx, sizeof(int) * n);
    }

    // Fill sorted_codes
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        sorted_codes[i] = codes[sorted_indices[i]];
    }
}
