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

    bool is_leaf() const { return object_idx >= 0; }
};

struct BVH {
    std::vector<BVHNode> nodes;  // [0, n-2] internal, [n-1, 2n-2] leaves
    int root;
    int num_objects;

    // Build LBVH from sorted Morton codes and object indices.
    // sorted_codes and sorted_indices must already be sorted by Morton code.
    void build(const std::vector<uint32_t>& sorted_codes,
               const std::vector<int>& sorted_indices,
               const std::vector<AABB>& object_aabbs);

    // Refit: update AABBs bottom-up keeping existing topology
    void refit(const std::vector<AABB>& object_aabbs);

    // Traverse: find all overlapping pairs using self-query
    void traverse(std::vector<CollisionPair>& pairs) const;

private:
    // Karras delta function: length of longest common prefix
    int delta(const std::vector<uint32_t>& codes, int i, int j) const;

    // Determine range for internal node i
    void determine_range(const std::vector<uint32_t>& codes, int i,
                         int& left, int& right) const;

    // Find split position within range
    int find_split(const std::vector<uint32_t>& codes, int left, int right) const;

    // Stack-based traversal for one query object
    void traverse_node(int query_obj, const AABB& query_aabb,
                       std::vector<CollisionPair>& pairs) const;
};
