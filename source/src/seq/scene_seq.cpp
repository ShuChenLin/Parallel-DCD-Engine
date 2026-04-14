#include "scene.h"
#include "morton.h"
#include "bvh.h"
#include "gjk.h"
#include "timer.h"

void Scene::step() {
    int n = static_cast<int>(bodies.size());
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

std::vector<CollisionPair> Scene::detect_collisions_seq() {
    int n = static_cast<int>(bodies.size());
    if (n == 0) return {};

    Timer t;

    // 1. Update AABBs and gather centroids
    _centroids.resize(n);
    _aabbs.resize(n);
    AABB scene_aabb;
    t.start();
    for (int i = 0; i < n; i++) {
        bodies[i].update_aabb();
        _centroids[i] = bodies[i].world_aabb.centroid();
        _aabbs[i] = bodies[i].world_aabb;
        scene_aabb.expand(_aabbs[i]);
    }
    stage_times.aabb_ms = t.stop();

    // 2-4. Full rebuild every rebuild_interval frames; refit only otherwise.
    bool do_rebuild = (frames_since_rebuild >= rebuild_interval)
                   || last_bvh.nodes.empty();

    if (do_rebuild) {
        t.start();
        compute_morton_codes_seq(_centroids, scene_aabb, _codes);
        stage_times.morton_ms = t.stop();

        t.start();
        radix_sort_seq(_codes, _indices, _sorted_codes);
        stage_times.sort_ms = t.stop();

        t.start();
        last_bvh.clear();
        bvh_build_seq(last_bvh, _sorted_codes, _indices, _aabbs);
        stage_times.build_ms = t.stop();

        frames_since_rebuild = 0;
    } else {
        stage_times.morton_ms = 0;
        stage_times.sort_ms   = 0;

        t.start();
        bvh_refit_seq(last_bvh, _aabbs);
        stage_times.build_ms = t.stop();
    }
    frames_since_rebuild++;

    // 5. Broad phase: BVH traversal
    t.start();
    bvh_traverse_seq(last_bvh, _broad_pairs);
    stage_times.traverse_ms = t.stop();

    // 6. Narrow phase: GJK on each candidate pair
    t.start();
    _confirmed.clear();
    for (const auto& p : _broad_pairs) {
        if (gjk_intersect(bodies[p.a], bodies[p.b])) {
            _confirmed.push_back(p);
        }
    }
    stage_times.gjk_ms = t.stop();

    return _confirmed;
}
