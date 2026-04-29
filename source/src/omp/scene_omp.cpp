#include "scene.h"
#include "morton.h"
#include "bvh.h"
#include "gjk.h"
#include "timer.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <omp.h>

struct GjkThreadStats {
    long long calls = 0;
    long long confirmed = 0;
    double work_ms = 0.0;
};

static int scene_load_balance_level() {
    const char* env = std::getenv("DCD_LOAD_BALANCE");
    return env ? std::atoi(env) : 0;
}

static void print_gjk_balance(const std::vector<GjkThreadStats>& stats) {
    long long min_calls = 0, max_calls = 0, sum_calls = 0;
    long long min_confirmed = 0, max_confirmed = 0, sum_confirmed = 0;
    double min_ms = 0.0, max_ms = 0.0, sum_ms = 0.0;
    bool initialized = false;

    for (const auto& s : stats) {
        if (!initialized) {
            min_calls = max_calls = s.calls;
            min_confirmed = max_confirmed = s.confirmed;
            min_ms = max_ms = s.work_ms;
            initialized = true;
        } else {
            min_calls = std::min(min_calls, s.calls);
            max_calls = std::max(max_calls, s.calls);
            min_confirmed = std::min(min_confirmed, s.confirmed);
            max_confirmed = std::max(max_confirmed, s.confirmed);
            min_ms = std::min(min_ms, s.work_ms);
            max_ms = std::max(max_ms, s.work_ms);
        }
        sum_calls += s.calls;
        sum_confirmed += s.confirmed;
        sum_ms += s.work_ms;
    }

    double avg_calls = stats.empty() ? 0.0 : (double)sum_calls / stats.size();
    double avg_confirmed = stats.empty() ? 0.0 : (double)sum_confirmed / stats.size();
    double avg_ms = stats.empty() ? 0.0 : sum_ms / stats.size();
    double call_imbalance = avg_calls > 0.0 ? max_calls / avg_calls : 0.0;
    double time_imbalance = avg_ms > 0.0 ? max_ms / avg_ms : 0.0;

    std::printf("    [load-balance] GJK calls min/avg/max=%lld/%.1f/%lld imbalance=%.2fx"
                " | confirmed min/avg/max=%lld/%.1f/%lld"
                " | time min/avg/max=%.3f/%.3f/%.3f ms imbalance=%.2fx\n",
                min_calls, avg_calls, max_calls, call_imbalance,
                min_confirmed, avg_confirmed, max_confirmed,
                min_ms, avg_ms, max_ms, time_imbalance);
}

void Scene::step_omp() {
    int n = static_cast<int>(bodies.size());
    if ((int)_positions.size() != n) {
        _positions.resize(n);
        _velocities.resize(n);
        _shape_types.assign(n, ShapeType::Cube);
        for (int i = 0; i < n; i++) {
            _positions[i] = bodies[i].position;
            _velocities[i] = bodies[i].velocity;
        }
    }
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        Vec3 position = _positions[i] + _velocities[i] * dt;
        Vec3 velocity = _velocities[i];

        for (int axis = 0; axis < 3; axis++) {
            float* pos = &position.x + axis;
            float* vel = &velocity.x + axis;
            float lo = (&bounds.lo.x)[axis];
            float hi = (&bounds.hi.x)[axis];
            if (*pos < lo) { *pos = lo + (lo - *pos); *vel = std::abs(*vel); }
            if (*pos > hi) { *pos = hi - (*pos - hi); *vel = -std::abs(*vel); }
        }

        _positions[i] = position;
        _velocities[i] = velocity;
        bodies[i].position = position;
        bodies[i].velocity = velocity;
    }
}

std::vector<CollisionPair> Scene::detect_collisions_omp() {
    int n = static_cast<int>(bodies.size());
    if (n == 0) return {};

    Timer t;

    // 1. Update AABBs and gather centroids (always needed)
    if ((int)_positions.size() != n) {
        _positions.resize(n);
        _velocities.resize(n);
        _shape_types.assign(n, ShapeType::Cube);
        for (int i = 0; i < n; i++) {
            _positions[i] = bodies[i].position;
            _velocities[i] = bodies[i].velocity;
        }
    }
    _centroids.resize(n);
    _aabbs.resize(n);
    AABB scene_aabb;
    t.start();
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        AABB box = compute_shape_aabb(_shape_types[i], _positions[i]);
        _centroids[i] = box.centroid();
        _aabbs[i] = box;
        bodies[i].world_aabb = box;
    }
    // Parallel AABB reduction
    {
        int num_threads = omp_get_max_threads();
        std::vector<AABB> local_aabbs(num_threads);
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            #pragma omp for schedule(static)
            for (int i = 0; i < n; i++) {
                local_aabbs[tid].expand(_aabbs[i]);
            }
        }
        for (int ti = 0; ti < num_threads; ti++) {
            scene_aabb.expand(local_aabbs[ti]);
        }
    }
    stage_times.aabb_ms = t.stop();

    // Decide: full rebuild or refit-only
    bool do_full_rebuild = (frames_since_rebuild >= rebuild_interval)
                        || (last_bvh.nodes.empty());

    if (do_full_rebuild) {
        // 2. Compute Morton codes
        t.start();
        compute_morton_codes_omp(_centroids, scene_aabb, _codes);
        stage_times.morton_ms = t.stop();

        // 3. Sort by Morton code
        t.start();
        radix_sort_omp(_codes, _indices, _sorted_codes);
        stage_times.sort_ms = t.stop();

        // 4. Build LBVH (includes refit for initial AABBs)
        t.start();
        last_bvh.clear();
        bvh_build_omp(last_bvh, _sorted_codes, _indices, _aabbs);
        stage_times.build_ms = t.stop();

        frames_since_rebuild = 0;
    } else {
        // Refit only: keep tree topology, update bounding boxes
        stage_times.morton_ms = 0;
        stage_times.sort_ms = 0;

        t.start();
        bvh_refit_omp(last_bvh, _aabbs);
        stage_times.build_ms = t.stop();
    }
    frames_since_rebuild++;

    // 5. Broad phase: BVH traversal
    t.start();
    bvh_traverse_omp(last_bvh, _broad_pairs, scenario == Scenario::TWO_CLUSTER);
    stage_times.traverse_ms = t.stop();

    // 6. Narrow phase: GJK on each candidate pair
    t.start();
    _confirmed.clear();
    {
        int np = static_cast<int>(_broad_pairs.size());
        int num_threads = omp_get_max_threads();
        int balance_level = scene_load_balance_level();
        std::vector<std::vector<CollisionPair>> thread_confirmed(num_threads);
        std::vector<GjkThreadStats> gjk_stats(balance_level ? num_threads : 0);
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            double t0 = balance_level ? omp_get_wtime() : 0.0;
            #pragma omp for schedule(dynamic, 64) nowait
            for (int i = 0; i < np; i++) {
                const auto& p = _broad_pairs[i];
                if (balance_level) gjk_stats[tid].calls++;
                if (gjk_intersect_soa(_shape_types[p.a], _positions[p.a],
                                      _shape_types[p.b], _positions[p.b])) {
                    thread_confirmed[tid].push_back(p);
                    if (balance_level) gjk_stats[tid].confirmed++;
                }
            }
            if (balance_level) gjk_stats[tid].work_ms = (omp_get_wtime() - t0) * 1000.0;
            #pragma omp barrier
        }
        for (int ti = 0; ti < num_threads; ti++) {
            _confirmed.insert(_confirmed.end(),
                              thread_confirmed[ti].begin(),
                              thread_confirmed[ti].end());
        }
        if (balance_level) {
            std::printf("    [load-balance] GJK candidate_pairs=%d confirmed=%zu\n",
                        np, _confirmed.size());
            print_gjk_balance(gjk_stats);
            if (balance_level >= 2) {
                std::printf("    [load-balance] GJK per-thread: tid calls confirmed work_ms\n");
                for (int ti = 0; ti < num_threads; ti++) {
                    std::printf("    [load-balance] GJK tid=%d calls=%lld confirmed=%lld work_ms=%.3f\n",
                                ti, gjk_stats[ti].calls, gjk_stats[ti].confirmed,
                                gjk_stats[ti].work_ms);
                }
            }
        }
    }
    stage_times.gjk_ms = t.stop();

    return _confirmed;
}
