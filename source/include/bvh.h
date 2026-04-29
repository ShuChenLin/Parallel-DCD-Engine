#pragma once
#include "aabb.h"
#include "collision.h"
#include <vector>
#include <cstdint>

struct BVHNode {
    AABB box;
    int left;        // left child index (internal) or -1
    int right;       // right child index (internal) or -1
    int parent;
    int object_idx;  // >= 0 for leaves, -1 for internal nodes
    int leaf_count;  // number of leaves under this node

    bool is_leaf() const { return object_idx >= 0; }
};

struct BVH {
    std::vector<BVHNode> nodes;  // [0, n-2] internal, [n-1, 2n-2] leaves
    int root = 0;
    int num_objects = 0;

    void clear() { nodes.clear(); root = 0; num_objects = 0; }
};

<<<<<<< Updated upstream
// Sequential BVH operations
=======
// --- Sequential BVH operations ---
>>>>>>> Stashed changes
void bvh_build_seq(BVH& bvh,
                   const std::vector<uint32_t>& sorted_codes,
                   const std::vector<int>& sorted_indices,
                   const std::vector<AABB>& object_aabbs);
<<<<<<< Updated upstream
void bvh_refit_seq(BVH& bvh, const std::vector<AABB>& object_aabbs);
void bvh_traverse_seq(const BVH& bvh, std::vector<CollisionPair>& pairs);

// OpenMP BVH operations
=======

void bvh_refit_seq(BVH& bvh, const std::vector<AABB>& object_aabbs);

void bvh_traverse_seq(const BVH& bvh, std::vector<CollisionPair>& pairs);

// --- OpenMP BVH operations ---
>>>>>>> Stashed changes
void bvh_build_omp(BVH& bvh,
                   const std::vector<uint32_t>& sorted_codes,
                   const std::vector<int>& sorted_indices,
                   const std::vector<AABB>& object_aabbs);
<<<<<<< Updated upstream
void bvh_refit_omp(BVH& bvh, const std::vector<AABB>& object_aabbs);
void bvh_traverse_omp(const BVH& bvh, std::vector<CollisionPair>& pairs,
                      bool weighted_split = false);
=======

// Parallel bottom-up refit using atomic propagation
void bvh_refit_omp(BVH& bvh, const std::vector<AABB>& object_aabbs);

void bvh_traverse_omp(const BVH& bvh, std::vector<CollisionPair>& pairs);
>>>>>>> Stashed changes
