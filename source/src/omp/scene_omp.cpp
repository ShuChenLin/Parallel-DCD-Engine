#include "scene.h"
#include "morton.h"
#include "bvh.h"
#include "gjk.h"
#include "timer.h"
#include <omp.h>

void Scene::step_omp() {
    int n = static_cast<int>(bodies.size());
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        auto& b = bodies[i];
        b.position += b.velocity * dt;

        for (int axis = 0; axis < 3; axis++) {
            float* pos = &b.position.x + axis;
            float* vel = &b.velocity.x + axis;
            float lo = (&bounds.lo.x)[axis];
            float hi = (&bounds.hi.x)[axis];
            if (*pos < lo) { *pos = lo + (lo - *pos); *vel = std::abs(*vel); }
            if (*pos > hi) { *pos = hi - (*pos - hi); *vel = -std::abs(*vel); }
        }
    }
}

std::vector<CollisionPair> Scene::detect_collisions_omp() {
    int n = static_cast<int>(bodies.size());
    if (n == 0) return {};

    Timer t;

    // 1. Update AABBs and gather centroids (always needed)
    _centroids.resize(n);
    _aabbs.resize(n);
    AABB scene_aabb;
    t.start();
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n; i++) {
        bodies[i].update_aabb();
        _centroids[i] = bodies[i].world_aabb.centroid();
        _aabbs[i] = bodies[i].world_aabb;
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
    bvh_traverse_omp(last_bvh, _broad_pairs);
    stage_times.traverse_ms = t.stop();

    // 6. Narrow phase: GJK on each candidate pair
    t.start();
    _confirmed.clear();
    {
        int np = static_cast<int>(_broad_pairs.size());
        int num_threads = omp_get_max_threads();
        std::vector<std::vector<CollisionPair>> thread_confirmed(num_threads);
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            #pragma omp for schedule(dynamic, 64)
            for (int i = 0; i < np; i++) {
                const auto& p = _broad_pairs[i];
                if (gjk_intersect(bodies[p.a], bodies[p.b])) {
                    thread_confirmed[tid].push_back(p);
                }
            }
        }
        for (int ti = 0; ti < num_threads; ti++) {
            _confirmed.insert(_confirmed.end(),
                              thread_confirmed[ti].begin(),
                              thread_confirmed[ti].end());
        }
    }
    stage_times.gjk_ms = t.stop();

    return _confirmed;
}
