#pragma once
#include <cuda_runtime.h>
#include <cstdio>
#include <cstdlib>
#include <cfloat>

#define CUDA_CHECK(call) do { \
    cudaError_t err = (call); \
    if (err != cudaSuccess) { \
        fprintf(stderr, "CUDA error at %s:%d: %s\n", \
                __FILE__, __LINE__, cudaGetErrorString(err)); \
        exit(1); \
    } \
} while(0)

#define MAX_VERTS 12
#define TRAVERSE_STACK_SIZE 64
#define BLOCK_SIZE 256
#define GJK_BLOCK_SIZE 128   // reduced block size so smem fits: 128*24*12 = 36KB < 48KB

/* ---- float3 helpers ---- */

inline __device__ __host__ float3 operator+(float3 a, float3 b) {
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline __device__ __host__ float3 operator-(float3 a, float3 b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline __device__ __host__ float3 operator*(float3 a, float s) {
    return make_float3(a.x * s, a.y * s, a.z * s);
}
inline __device__ __host__ float3 operator-(float3 a) {
    return make_float3(-a.x, -a.y, -a.z);
}
inline __device__ __host__ float dot3(float3 a, float3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __device__ __host__ float3 cross3(float3 a, float3 b) {
    return make_float3(a.y * b.z - a.z * b.y,
                       a.z * b.x - a.x * b.z,
                       a.x * b.y - a.y * b.x);
}
inline __device__ __host__ float len2_f3(float3 a) { return dot3(a, a); }
inline __device__ __host__ float3 f3min(float3 a, float3 b) {
    return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
inline __device__ __host__ float3 f3max(float3 a, float3 b) {
    return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}

/* ---- GPU BVH node ---- */

struct GBVHNode {
    float lo_x, lo_y, lo_z;
    float hi_x, hi_y, hi_z;
    int left, right, parent, object_idx;
};

inline __device__ bool gnode_is_leaf(const GBVHNode& nd) { return nd.object_idx >= 0; }

inline __device__ bool gnode_overlaps(const GBVHNode& a, const GBVHNode& b) {
    return a.lo_x <= b.hi_x && b.lo_x <= a.hi_x &&
           a.lo_y <= b.hi_y && b.lo_y <= a.hi_y &&
           a.lo_z <= b.hi_z && b.lo_z <= a.hi_z;
}

/* ---- CUDA context (persistent GPU allocations) ---- */

struct CUDAContext {
    float3*   d_positions;
    float3*   d_velocities;
    float3*   d_vertices;       // AoS: [body * MAX_VERTS + j]
    int*      d_vert_counts;
    float3*   d_aabb_lo;
    float3*   d_aabb_hi;
    float3*   d_centroids;

    float3*   d_block_lo;       // reduction scratch
    float3*   d_block_hi;
    int       num_reduce_blocks;

    uint32_t* d_codes;
    uint32_t* d_sorted_codes;
    int*      d_sorted_indices;

    GBVHNode* d_nodes;          // [2n-1]
    int*      d_flags;          // [n-1] atomic refit

    int2*     d_broad_pairs;
    int*      d_broad_count;    // [1]
    int       max_broad;

    int2*     d_narrow_pairs;
    int*      d_narrow_count;   // [1]
    int       max_narrow;

    int2*     h_narrow_pinned;  // pinned host buffer for pair download (avoids staging copy)

    int*      d_overflow;       // [1]

    int  allocated_n;
    bool shapes_uploaded;   // vertices + vert_counts uploaded once per scene init

    CUDAContext() : allocated_n(0), shapes_uploaded(false), h_narrow_pinned(nullptr) {}

    void ensure(int n) {
        if (n <= allocated_n) return;
        free_all();
        allocated_n = n;

        CUDA_CHECK(cudaMalloc(&d_positions,   n * sizeof(float3)));
        CUDA_CHECK(cudaMalloc(&d_velocities,  n * sizeof(float3)));
        CUDA_CHECK(cudaMalloc(&d_vertices,    (size_t)MAX_VERTS * n * sizeof(float3)));
        CUDA_CHECK(cudaMalloc(&d_vert_counts, n * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&d_aabb_lo,     n * sizeof(float3)));
        CUDA_CHECK(cudaMalloc(&d_aabb_hi,     n * sizeof(float3)));
        CUDA_CHECK(cudaMalloc(&d_centroids,   n * sizeof(float3)));

        num_reduce_blocks = min((n + BLOCK_SIZE - 1) / BLOCK_SIZE, 1024);
        CUDA_CHECK(cudaMalloc(&d_block_lo, num_reduce_blocks * sizeof(float3)));
        CUDA_CHECK(cudaMalloc(&d_block_hi, num_reduce_blocks * sizeof(float3)));

        CUDA_CHECK(cudaMalloc(&d_codes,          n * sizeof(uint32_t)));
        CUDA_CHECK(cudaMalloc(&d_sorted_codes,   n * sizeof(uint32_t)));
        CUDA_CHECK(cudaMalloc(&d_sorted_indices, n * sizeof(int)));

        int nn = 2 * n - 1;
        CUDA_CHECK(cudaMalloc(&d_nodes, nn * sizeof(GBVHNode)));
        if (n > 1) CUDA_CHECK(cudaMalloc(&d_flags, (n - 1) * sizeof(int)));
        else d_flags = nullptr;

        max_broad = n * 64;
        CUDA_CHECK(cudaMalloc(&d_broad_pairs, max_broad * sizeof(int2)));
        CUDA_CHECK(cudaMalloc(&d_broad_count, sizeof(int)));

        max_narrow = n * 64;
        CUDA_CHECK(cudaMalloc(&d_narrow_pairs, max_narrow * sizeof(int2)));
        CUDA_CHECK(cudaMalloc(&d_narrow_count, sizeof(int)));
        CUDA_CHECK(cudaMallocHost(&h_narrow_pinned, max_narrow * sizeof(int2)));

        CUDA_CHECK(cudaMalloc(&d_overflow, sizeof(int)));
    }

    void free_all() {
        if (!allocated_n) return;
        shapes_uploaded = false;
        cudaFree(d_positions);  cudaFree(d_velocities);
        cudaFree(d_vertices);   cudaFree(d_vert_counts);
        cudaFree(d_aabb_lo);    cudaFree(d_aabb_hi);
        cudaFree(d_centroids);
        cudaFree(d_block_lo);   cudaFree(d_block_hi);
        cudaFree(d_codes);      cudaFree(d_sorted_codes); cudaFree(d_sorted_indices);
        cudaFree(d_nodes);      cudaFree(d_flags);
        cudaFree(d_broad_pairs); cudaFree(d_broad_count);
        cudaFree(d_narrow_pairs); cudaFree(d_narrow_count);
        cudaFreeHost(h_narrow_pinned);
        cudaFree(d_overflow);
        allocated_n = 0;
    }

    ~CUDAContext() { free_all(); }
};
