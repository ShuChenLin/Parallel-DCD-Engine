/**
 * @file bvh.h
 * @brief BVH node types and BVH function declarations.
 */

#pragma once
#include "aabb.h"
#include "collision.h"
#include <vector>
#include <cstdint>

/**
 * @brief One BVH node.
 */
struct BVHNode {
    AABB box;
    int left;        // left child index (internal) or -1
    int right;       // right child index (internal) or -1
    int parent;
    int object_idx;  // >= 0 for leaves, -1 for internal nodes
    int leaf_count;  // number of leaves under this node

    bool is_leaf() const { return object_idx >= 0; }
};

/**
 * @brief Bounding volume hierarchy used in broad phase.
 */
struct BVH {
    std::vector<BVHNode> nodes;  // [0, n-2] internal, [n-1, 2n-2] leaves
    int root = 0;
    int num_objects = 0;

    void clear() { nodes.clear(); root = 0; num_objects = 0; }
};

/**
 * @brief Builds the sequential LBVH.
 */
void bvh_build_seq(BVH& bvh,
                   const std::vector<uint32_t>& sorted_codes,
                   const std::vector<int>& sorted_indices,
                   const std::vector<AABB>& object_aabbs);
/**
 * @brief Refits node bounds in the sequential BVH.
 */
void bvh_refit_seq(BVH& bvh, const std::vector<AABB>& object_aabbs);
/**
 * @brief Traverses the sequential BVH and emits candidate pairs.
 */
void bvh_traverse_seq(const BVH& bvh, std::vector<CollisionPair>& pairs);

/**
 * @brief Builds the OpenMP LBVH.
 */
void bvh_build_omp(BVH& bvh,
                   const std::vector<uint32_t>& sorted_codes,
                   const std::vector<int>& sorted_indices,
                   const std::vector<AABB>& object_aabbs);
/**
 * @brief Refits node bounds in the OpenMP BVH.
 */
void bvh_refit_omp(BVH& bvh, const std::vector<AABB>& object_aabbs);
/**
 * @brief Traverses the OpenMP BVH and emits candidate pairs.
 */
void bvh_traverse_omp(const BVH& bvh, std::vector<CollisionPair>& pairs,
                      bool weighted_split = false);
