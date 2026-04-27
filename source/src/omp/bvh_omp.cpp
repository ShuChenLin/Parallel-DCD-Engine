#include "bvh.h"
#include <algorithm>
#include <functional>
#include <omp.h>

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

void bvh_build_omp(BVH& bvh,
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
        bvh.nodes[0].leaf_count = 1;
        bvh.root = 0;
        return;
    }

    bvh.nodes.resize(2 * n - 1);

    // Initialize leaf nodes [n-1, 2n-2]
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        int leaf = n - 1 + i;
        bvh.nodes[leaf].object_idx = sorted_indices[i];
        bvh.nodes[leaf].box = object_aabbs[sorted_indices[i]];
        bvh.nodes[leaf].left = -1;
        bvh.nodes[leaf].right = -1;
        bvh.nodes[leaf].parent = -1;
        bvh.nodes[leaf].leaf_count = 1;
    }

    // Build internal nodes using Karras algorithm (parallel)
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n - 1; i++) {
        int left_bound, right_bound;
        determine_range(sorted_codes, n, i, left_bound, right_bound);
        int split = find_split(sorted_codes, left_bound, right_bound);

        int left_child = (split == left_bound) ? (n - 1 + split) : split;
        int right_child = (split + 1 == right_bound) ? (n - 1 + (split + 1)) : (split + 1);

        bvh.nodes[i].left = left_child;
        bvh.nodes[i].right = right_child;
        bvh.nodes[i].object_idx = -1;
        bvh.nodes[i].leaf_count = 0;
        bvh.nodes[left_child].parent = i;
        bvh.nodes[right_child].parent = i;
    }

    bvh.root = 0;
    bvh.nodes[bvh.root].parent = -1;

    bvh_refit_omp(bvh, object_aabbs);
}

// Parallel bottom-up refit using atomic propagation.
// Each leaf signals its parent; the second thread to arrive at an internal node
// merges both children's AABBs and propagates upward.
void bvh_refit_omp(BVH& bvh, const std::vector<AABB>& object_aabbs) {
    if (bvh.nodes.empty()) return;
    int n = bvh.num_objects;

    if (n <= 1) {
        if (n == 1) bvh.nodes[0].box = object_aabbs[bvh.nodes[0].object_idx];
        return;
    }

    // Update leaf AABBs
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        int leaf = n - 1 + i;
        bvh.nodes[leaf].box = object_aabbs[bvh.nodes[leaf].object_idx];
    }

    // Atomic flags for internal nodes [0, n-2]
    // 0 = no child done yet, 1 = one child done (first arrival → exit)
    std::vector<int> flags(n - 1, 0);

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        int leaf = n - 1 + i;
        int node = bvh.nodes[leaf].parent;
        while (node >= 0) {
            // Atomically increment; first thread to arrive sees 0 → exits
            if (__sync_fetch_and_add(&flags[node], 1) == 0) break;
            // Second thread: both children are ready
            bvh.nodes[node].box = AABB::merge(
                bvh.nodes[bvh.nodes[node].left].box,
                bvh.nodes[bvh.nodes[node].right].box);
            bvh.nodes[node].leaf_count =
                bvh.nodes[bvh.nodes[node].left].leaf_count +
                bvh.nodes[bvh.nodes[node].right].leaf_count;
            node = bvh.nodes[node].parent;
        }
    }
}

// Iterative dual traversal (used per-task in parallel phase).
// b < 0: self_traverse(a) — all pairs within subtree a
// b >= 0: cross(a, b)     — all pairs between subtrees a and b
static void dual_traverse_omp(const std::vector<BVHNode>& nodes,
                               int a, int b,
                               std::vector<CollisionPair>& out) {
    struct Frame { int a, b; };
    std::vector<Frame> stk;
    stk.reserve(64);
    stk.push_back({a, b});

    while (!stk.empty()) {
        auto [na, nb] = stk.back(); stk.pop_back();

        if (nb < 0) {
            if (nodes[na].is_leaf()) continue;
            stk.push_back({nodes[na].left,  -1});
            stk.push_back({nodes[na].right, -1});
            stk.push_back({nodes[na].left,  nodes[na].right});
        } else {
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

void bvh_traverse_omp(const BVH& bvh, std::vector<CollisionPair>& pairs,
                      bool weighted_split) {
    pairs.clear();
    if (bvh.nodes.empty()) return;

    int num_threads = omp_get_max_threads();

    struct Task { int a, b; };
    std::vector<Task> pending = {{bvh.root, -1}};
    std::vector<Task> tasks;
    tasks.reserve(num_threads * (weighted_split ? 64 : 16));

    const int TARGET = num_threads * (weighted_split ? 64 : 16);
    const long long MAX_TASK_WEIGHT = 4096;
    int head = 0;

    auto task_weight = [&](const Task& t) -> long long {
        if (t.b < 0) {
            long long leaves = bvh.nodes[t.a].leaf_count;
            return leaves * leaves;
        }
        return (long long)bvh.nodes[t.a].leaf_count * bvh.nodes[t.b].leaf_count;
    };

    auto should_expand = [&]() {
        if (head >= (int)pending.size()) return false;
        if (!weighted_split) return (int)pending.size() < TARGET;
        return (int)tasks.size() < TARGET || task_weight(pending[head]) > MAX_TASK_WEIGHT;
    };

    while (should_expand()) {
        Task t = pending[head++];

        if (t.b < 0) {
            if (bvh.nodes[t.a].is_leaf()) continue;
            pending.push_back({bvh.nodes[t.a].left,  -1});
            pending.push_back({bvh.nodes[t.a].right, -1});
            pending.push_back({bvh.nodes[t.a].left,  bvh.nodes[t.a].right});
        } else {
            if (!AABB::overlaps(bvh.nodes[t.a].box, bvh.nodes[t.b].box)) continue;
            bool al = bvh.nodes[t.a].is_leaf(), bl = bvh.nodes[t.b].is_leaf();
            if (al && bl) {
                int oa = bvh.nodes[t.a].object_idx, ob = bvh.nodes[t.b].object_idx;
                if (oa < ob) pairs.push_back({oa, ob});
                else if (ob < oa) pairs.push_back({ob, oa});
                continue;
            }
            if (al) {
                pending.push_back({t.a, bvh.nodes[t.b].left});
                pending.push_back({t.a, bvh.nodes[t.b].right});
            } else if (!weighted_split || bl ||
                       bvh.nodes[t.a].leaf_count >= bvh.nodes[t.b].leaf_count) {
                pending.push_back({bvh.nodes[t.a].left,  t.b});
                pending.push_back({bvh.nodes[t.a].right, t.b});
            } else {
                pending.push_back({t.a, bvh.nodes[t.b].left});
                pending.push_back({t.a, bvh.nodes[t.b].right});
            }
        }

        while (weighted_split && head < (int)pending.size() && (int)tasks.size() < TARGET) {
            Task next = pending[head];
            if (task_weight(next) > MAX_TASK_WEIGHT) break;
            tasks.push_back(next);
            head++;
        }
    }

    tasks.insert(tasks.end(), pending.begin() + head, pending.end());
    int ntasks = (int)tasks.size();

    std::vector<std::vector<CollisionPair>> thread_pairs(num_threads);
    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < ntasks; i++) {
        int tid = omp_get_thread_num();
        dual_traverse_omp(bvh.nodes, tasks[i].a, tasks[i].b, thread_pairs[tid]);
    }

    for (int t = 0; t < num_threads; t++) {
        pairs.insert(pairs.end(), thread_pairs[t].begin(), thread_pairs[t].end());
    }
}
