/**
 * @file bvh_omp.cpp
 * @brief OpenMP BVH construction, refit, and broad-phase traversal.
 *
 * This file implements the parallel LBVH build, bottom-up refit, dual
 * traversal, and optional traversal load-balance instrumentation used by the
 * OpenMP collision-detection path.
 */

#include "bvh.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <omp.h>

struct TraverseThreadStats {
    long long tasks = 0;
    long long stack_visits = 0;
    long long overlap_tests = 0;
    long long leaf_pairs = 0;
    long long emitted_pairs = 0;
    double work_ms = 0.0;
};

/**
 * @brief Reads the traversal load-balance instrumentation level.
 *
 * The level is controlled through DCD_LOAD_BALANCE and enables summary or
 * detailed per-thread reporting for broad-phase traversal.
 */
static int load_balance_level() {
    const char* env = std::getenv("DCD_LOAD_BALANCE");
    return env ? std::atoi(env) : 0;
}

/**
 * @brief Prints min/avg/max imbalance statistics for a traversal counter.
 *
 * This helper is used to summarize traversal work counts and the associated
 * per-thread execution time after the parallel broad phase completes.
 */
static void print_balance_summary(const char* label,
                                  const std::vector<TraverseThreadStats>& stats,
                                  long long TraverseThreadStats::*count_member,
                                  double TraverseThreadStats::*time_member) {
    long long min_count = 0, max_count = 0, sum_count = 0;
    double min_ms = 0.0, max_ms = 0.0, sum_ms = 0.0;
    bool initialized = false;

    for (const auto& s : stats) {
        long long count = s.*count_member;
        double ms = s.*time_member;
        if (!initialized) {
            min_count = max_count = count;
            min_ms = max_ms = ms;
            initialized = true;
        } else {
            min_count = std::min(min_count, count);
            max_count = std::max(max_count, count);
            min_ms = std::min(min_ms, ms);
            max_ms = std::max(max_ms, ms);
        }
        sum_count += count;
        sum_ms += ms;
    }

    double avg_count = stats.empty() ? 0.0 : (double)sum_count / stats.size();
    double avg_ms = stats.empty() ? 0.0 : sum_ms / stats.size();
    double count_imbalance = avg_count > 0.0 ? max_count / avg_count : 0.0;
    double time_imbalance = avg_ms > 0.0 ? max_ms / avg_ms : 0.0;

    std::printf("    [load-balance] %s count min/avg/max=%lld/%.1f/%lld imbalance=%.2fx"
                " | time min/avg/max=%.3f/%.3f/%.3f ms imbalance=%.2fx\n",
                label, min_count, avg_count, max_count, count_imbalance,
                min_ms, avg_ms, max_ms, time_imbalance);
}

/**
 * @brief Returns the number of leading zero bits in a 32-bit integer.
 */
static int clz(uint32_t x) {
    if (x == 0) return 32;
    return __builtin_clz(x);
}

/**
 * @brief Computes the Karras delta metric between two Morton-code positions.
 *
 * The delta is the common-prefix length used by LBVH range determination.
 * When the Morton codes are identical, the object indices break the tie.
 */
static int bvh_delta(const std::vector<uint32_t>& codes, int num_objects,
                     int i, int j) {
    if (j < 0 || j >= num_objects) return -1;
    if (codes[i] == codes[j])
        return 32 + clz(static_cast<uint32_t>(i ^ j));
    return clz(codes[i] ^ codes[j]);
}

/**
 * @brief Determines the object range covered by internal node i.
 *
 * This follows the LBVH construction method from Karras by expanding in the
 * dominant direction until the longest valid Morton-code range is found.
 */
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

/**
 * @brief Finds the split point for a Morton-code range.
 *
 * The split separates a node's range into its left and right child ranges
 * based on the shared Morton-code prefix.
 */
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

/**
 * @brief Builds an LBVH in parallel from sorted Morton codes and AABBs.
 *
 * Leaf nodes are initialized from the sorted object order, internal topology
 * is generated independently per node, and the final node bounds are filled by
 * the parallel refit pass.
 */
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

/**
 * @brief Refits BVH node bounds in parallel using atomic bottom-up propagation.
 *
 * Each leaf updates its own bounding box and walks toward the root. The first
 * child to reach an internal node exits, while the second merges both children
 * and continues the upward propagation.
 */
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

/**
 * @brief Traverses one BVH task iteratively and emits overlapping leaf pairs.
 *
 * When b < 0 the task performs self-traversal within subtree a; otherwise it
 * performs cross traversal between subtrees a and b.
 */
static void dual_traverse_omp(const std::vector<BVHNode>& nodes,
                               int a, int b,
                               std::vector<CollisionPair>& out,
                               TraverseThreadStats* stats = nullptr) {
    struct Frame { int a, b; };
    std::vector<Frame> stk;
    stk.reserve(64);
    stk.push_back({a, b});

    while (!stk.empty()) {
        auto [na, nb] = stk.back(); stk.pop_back();
        if (stats) stats->stack_visits++;

        if (nb < 0) {
            if (nodes[na].is_leaf()) continue;
            stk.push_back({nodes[na].left,  -1});
            stk.push_back({nodes[na].right, -1});
            stk.push_back({nodes[na].left,  nodes[na].right});
        } else {
            if (stats) stats->overlap_tests++;
            if (!AABB::overlaps(nodes[na].box, nodes[nb].box)) continue;
            bool al = nodes[na].is_leaf(), bl = nodes[nb].is_leaf();
            if (al && bl) {
                if (stats) stats->leaf_pairs++;
                int oa = nodes[na].object_idx, ob = nodes[nb].object_idx;
                if (oa < ob) {
                    out.push_back({oa, ob});
                    if (stats) stats->emitted_pairs++;
                } else if (ob < oa) {
                    out.push_back({ob, oa});
                    if (stats) stats->emitted_pairs++;
                }
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

/**
 * @brief Runs the OpenMP broad phase with task-based dual BVH traversal.
 *
 * The traversal first expands the root into a frontier of subtree-pair tasks,
 * optionally applies extra weighted splitting for dense scenes, and then
 * processes those tasks in parallel to produce candidate collision pairs.
 */
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
    int balance_level = load_balance_level();
    std::vector<TraverseThreadStats> stats(balance_level ? num_threads : 0);

    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < ntasks; i++) {
        int tid = omp_get_thread_num();
        double t0 = balance_level ? omp_get_wtime() : 0.0;
        if (balance_level) stats[tid].tasks++;
        dual_traverse_omp(bvh.nodes, tasks[i].a, tasks[i].b, thread_pairs[tid],
                          balance_level ? &stats[tid] : nullptr);
        if (balance_level) stats[tid].work_ms += (omp_get_wtime() - t0) * 1000.0;
    }

    for (int t = 0; t < num_threads; t++) {
        pairs.insert(pairs.end(), thread_pairs[t].begin(), thread_pairs[t].end());
    }

    if (balance_level) {
        std::printf("    [load-balance] BVH traverse tasks=%d output_pairs=%zu weighted=%s\n",
                    ntasks, pairs.size(), weighted_split ? "yes" : "no");
        print_balance_summary("traverse stack_visits", stats,
                              &TraverseThreadStats::stack_visits,
                              &TraverseThreadStats::work_ms);
        print_balance_summary("traverse emitted_pairs", stats,
                              &TraverseThreadStats::emitted_pairs,
                              &TraverseThreadStats::work_ms);

        if (balance_level >= 2) {
            std::printf("    [load-balance] BVH per-thread: tid tasks visits overlap leaf emitted work_ms\n");
            for (int t = 0; t < num_threads; t++) {
                std::printf("    [load-balance] BVH tid=%d tasks=%lld visits=%lld overlap=%lld"
                            " leaf=%lld emitted=%lld work_ms=%.3f\n",
                            t, stats[t].tasks, stats[t].stack_visits,
                            stats[t].overlap_tests, stats[t].leaf_pairs,
                            stats[t].emitted_pairs, stats[t].work_ms);
            }
        }
    }
}
