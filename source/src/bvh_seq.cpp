#include "bvh.h"
#include <algorithm>
#include <climits>
#include <functional>

// Count leading zeros of XOR to measure common prefix length
static int clz(uint32_t x) {
    if (x == 0) return 32;
    return __builtin_clz(x);
}

int BVH::delta(const std::vector<uint32_t>& codes, int i, int j) const {
    if (j < 0 || j >= num_objects) return -1;
    if (codes[i] == codes[j]) {
        // Tie-break with index
        return 32 + clz(static_cast<uint32_t>(i ^ j));
    }
    return clz(codes[i] ^ codes[j]);
}

void BVH::determine_range(const std::vector<uint32_t>& codes, int i,
                           int& out_left, int& out_right) const {
    // Determine direction of the range
    int d_left = delta(codes, i, i - 1);
    int d_right = delta(codes, i, i + 1);
    int d = (d_right >= d_left) ? 1 : -1;

    // Compute upper bound for the length of the range
    int d_min = delta(codes, i, i - d);
    int lmax = 2;
    while (delta(codes, i, i + lmax * d) > d_min) {
        lmax *= 2;
    }

    // Find the other end using binary search
    int l = 0;
    for (int t = lmax / 2; t >= 1; t /= 2) {
        if (delta(codes, i, i + (l + t) * d) > d_min) {
            l = l + t;
        }
    }
    int j = i + l * d;

    out_left = std::min(i, j);
    out_right = std::max(i, j);
}

int BVH::find_split(const std::vector<uint32_t>& codes, int left, int right) const {
    uint32_t first_code = codes[left];
    uint32_t last_code = codes[right];

    if (first_code == last_code) {
        return (left + right) / 2;
    }

    int common_prefix = clz(first_code ^ last_code);

    // Use binary search to find where the next bit differs
    int split = left;
    int step = right - left;

    do {
        step = (step + 1) / 2;
        int new_split = split + step;
        if (new_split < right) {
            int prefix = clz(first_code ^ codes[new_split]);
            if (prefix > common_prefix) {
                split = new_split;
            }
        }
    } while (step > 1);

    return split;
}

void BVH::build(const std::vector<uint32_t>& sorted_codes,
                const std::vector<int>& sorted_indices,
                const std::vector<AABB>& object_aabbs) {
    int n = static_cast<int>(sorted_codes.size());
    if (n == 0) return;
    num_objects = n;

    // Special case: single object
    if (n == 1) {
        nodes.resize(1);
        nodes[0].box = object_aabbs[sorted_indices[0]];
        nodes[0].left = -1;
        nodes[0].right = -1;
        nodes[0].parent = -1;
        nodes[0].object_idx = sorted_indices[0];
        root = 0;
        return;
    }

    // n-1 internal nodes + n leaf nodes = 2n-1 total
    nodes.resize(2 * n - 1);

    // Initialize leaf nodes [n-1, 2n-2]
    for (int i = 0; i < n; i++) {
        int leaf = n - 1 + i;
        nodes[leaf].object_idx = sorted_indices[i];
        nodes[leaf].box = object_aabbs[sorted_indices[i]];
        nodes[leaf].left = -1;
        nodes[leaf].right = -1;
        nodes[leaf].parent = -1;
    }

    // Build internal nodes [0, n-2] using Karras algorithm
    for (int i = 0; i < n - 1; i++) {
        int left_bound, right_bound;
        determine_range(sorted_codes, i, left_bound, right_bound);
        int split = find_split(sorted_codes, left_bound, right_bound);

        // Left child
        int left_child;
        if (split == left_bound) {
            left_child = n - 1 + split;  // leaf
        } else {
            left_child = split;  // internal
        }

        // Right child
        int right_child;
        if (split + 1 == right_bound) {
            right_child = n - 1 + (split + 1);  // leaf
        } else {
            right_child = split + 1;  // internal
        }

        nodes[i].left = left_child;
        nodes[i].right = right_child;
        nodes[i].object_idx = -1;
        nodes[left_child].parent = i;
        nodes[right_child].parent = i;
    }

    root = 0;
    nodes[root].parent = -1;

    // Compute internal node AABBs bottom-up
    // We do a post-order traversal
    refit(object_aabbs);
}

void BVH::refit(const std::vector<AABB>& object_aabbs) {
    if (nodes.empty()) return;

    // Update leaf AABBs
    for (int i = 0; i < num_objects; i++) {
        int leaf = num_objects - 1 + i;
        nodes[leaf].box = object_aabbs[nodes[leaf].object_idx];
    }

    // Post-order traversal: children updated before parent
    std::function<void(int)> update = [&](int idx) {
        if (nodes[idx].is_leaf()) return;
        update(nodes[idx].left);
        update(nodes[idx].right);
        nodes[idx].box = AABB::merge(nodes[nodes[idx].left].box,
                                     nodes[nodes[idx].right].box);
    };
    update(root);
}

void BVH::traverse_node(int query_obj, const AABB& query_aabb,
                         std::vector<CollisionPair>& pairs) const {
    _stack_buf.clear();
    _stack_buf.push_back(root);

    while (!_stack_buf.empty()) {
        int idx = _stack_buf.back(); _stack_buf.pop_back();

        if (!AABB::overlaps(query_aabb, nodes[idx].box)) continue;

        if (nodes[idx].is_leaf()) {
            int other = nodes[idx].object_idx;
            if (query_obj < other) {  // avoid duplicates
                pairs.push_back({query_obj, other});
            }
        } else {
            _stack_buf.push_back(nodes[idx].left);
            _stack_buf.push_back(nodes[idx].right);
        }
    }
}

void BVH::traverse(std::vector<CollisionPair>& pairs) const {
    pairs.clear();
    if (nodes.empty()) return;

    for (int i = 0; i < num_objects; i++) {
        int leaf = num_objects - 1 + i;
        traverse_node(nodes[leaf].object_idx, nodes[leaf].box, pairs);
    }
}
