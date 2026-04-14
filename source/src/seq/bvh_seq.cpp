#include "bvh.h"
#include <algorithm>
#include <functional>

static int clz(uint32_t x) {
    if (x == 0) return 32;
    return __builtin_clz(x);
}

static int bvh_delta(const std::vector<uint32_t>& codes, int num_objects,
                     int i, int j) {
    if (j < 0 || j >= num_objects) return -1;
    if (codes[i] == codes[j])
        return 32 + clz(static_cast<uint32_t>(i ^ j));
    return clz(codes[i] ^ codes[j]);
}

static void determine_range(const std::vector<uint32_t>& codes, int num_objects,
                             int i, int& out_left, int& out_right) {
    int d_left = bvh_delta(codes, num_objects, i, i - 1);
    int d_right = bvh_delta(codes, num_objects, i, i + 1);
    int d = (d_right >= d_left) ? 1 : -1;

    int d_min = bvh_delta(codes, num_objects, i, i - d);
    int lmax = 2;
    while (bvh_delta(codes, num_objects, i, i + lmax * d) > d_min) {
        lmax *= 2;
    }

    int l = 0;
    for (int t = lmax / 2; t >= 1; t /= 2) {
        if (bvh_delta(codes, num_objects, i, i + (l + t) * d) > d_min) {
            l = l + t;
        }
    }
    int j = i + l * d;

    out_left = std::min(i, j);
    out_right = std::max(i, j);
}

static int find_split(const std::vector<uint32_t>& codes,
                      int left, int right) {
    uint32_t first_code = codes[left];
    uint32_t last_code = codes[right];

    if (first_code == last_code)
        return (left + right) / 2;

    int common_prefix = clz(first_code ^ last_code);

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

void bvh_build_seq(BVH& bvh,
                   const std::vector<uint32_t>& sorted_codes,
                   const std::vector<int>& sorted_indices,
                   const std::vector<AABB>& object_aabbs) {
    int n = static_cast<int>(sorted_codes.size());
    if (n == 0) return;
    bvh.num_objects = n;

    if (n == 1) {
        bvh.nodes.resize(1);
        bvh.nodes[0].box = object_aabbs[sorted_indices[0]];
        bvh.nodes[0].left = -1;
        bvh.nodes[0].right = -1;
        bvh.nodes[0].parent = -1;
        bvh.nodes[0].object_idx = sorted_indices[0];
        bvh.root = 0;
        return;
    }

    bvh.nodes.resize(2 * n - 1);

    // Initialize leaf nodes [n-1, 2n-2]
    for (int i = 0; i < n; i++) {
        int leaf = n - 1 + i;
        bvh.nodes[leaf].object_idx = sorted_indices[i];
        bvh.nodes[leaf].box = object_aabbs[sorted_indices[i]];
        bvh.nodes[leaf].left = -1;
        bvh.nodes[leaf].right = -1;
        bvh.nodes[leaf].parent = -1;
    }

    // Build internal nodes using Karras algorithm
    for (int i = 0; i < n - 1; i++) {
        int left_bound, right_bound;
        determine_range(sorted_codes, n, i, left_bound, right_bound);
        int split = find_split(sorted_codes, left_bound, right_bound);

        int left_child = (split == left_bound) ? (n - 1 + split) : split;
        int right_child = (split + 1 == right_bound) ? (n - 1 + (split + 1)) : (split + 1);

        bvh.nodes[i].left = left_child;
        bvh.nodes[i].right = right_child;
        bvh.nodes[i].object_idx = -1;
        bvh.nodes[left_child].parent = i;
        bvh.nodes[right_child].parent = i;
    }

    bvh.root = 0;
    bvh.nodes[bvh.root].parent = -1;

    bvh_refit_seq(bvh, object_aabbs);
}

void bvh_refit_seq(BVH& bvh, const std::vector<AABB>& object_aabbs) {
    if (bvh.nodes.empty()) return;

    // Update leaf AABBs
    for (int i = 0; i < bvh.num_objects; i++) {
        int leaf = bvh.num_objects - 1 + i;
        bvh.nodes[leaf].box = object_aabbs[bvh.nodes[leaf].object_idx];
    }

    // Post-order traversal for internal nodes
    std::function<void(int)> update = [&](int idx) {
        if (bvh.nodes[idx].is_leaf()) return;
        update(bvh.nodes[idx].left);
        update(bvh.nodes[idx].right);
        bvh.nodes[idx].box = AABB::merge(bvh.nodes[bvh.nodes[idx].left].box,
                                          bvh.nodes[bvh.nodes[idx].right].box);
    };
    update(bvh.root);
}

// Iterative dual traversal.
// b < 0: self_traverse(a) — all pairs within subtree a
// b >= 0: cross(a, b)     — all pairs between subtrees a and b
static void dual_traverse_seq(const std::vector<BVHNode>& nodes,
                               int a, int b,
                               std::vector<CollisionPair>& out) {
    struct Frame { int a, b; };
    std::vector<Frame> stk;
    stk.reserve(64);
    stk.push_back({a, b});

    while (!stk.empty()) {
        auto [na, nb] = stk.back(); stk.pop_back();

        if (nb < 0) {
            // self_traverse(na)
            if (nodes[na].is_leaf()) continue;
            stk.push_back({nodes[na].left,  -1});
            stk.push_back({nodes[na].right, -1});
            stk.push_back({nodes[na].left,  nodes[na].right});
        } else {
            // cross(na, nb)
            if (!AABB::overlaps(nodes[na].box, nodes[nb].box)) continue;
            bool al = nodes[na].is_leaf(), bl = nodes[nb].is_leaf();
            if (al && bl) {
                int oa = nodes[na].object_idx, ob = nodes[nb].object_idx;
                if (oa < ob) out.push_back({oa, ob});
                else if (ob < oa) out.push_back({ob, oa});
                continue;
            }
            if (al) {
                stk.push_back({na, nodes[nb].left});
                stk.push_back({na, nodes[nb].right});
            } else {
                stk.push_back({nodes[na].left,  nb});
                stk.push_back({nodes[na].right, nb});
            }
        }
    }
}

void bvh_traverse_seq(const BVH& bvh, std::vector<CollisionPair>& pairs) {
    pairs.clear();
    if (bvh.nodes.empty()) return;
    dual_traverse_seq(bvh.nodes, bvh.root, -1, pairs);
}
