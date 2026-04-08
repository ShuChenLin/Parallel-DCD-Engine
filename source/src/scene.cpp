#include "scene.h"
#include "morton.h"
#include "bvh.h"
#include "gjk.h"
#include <random>
#include <algorithm>
#include <numeric>

void Scene::init(int n, Scenario scenario, float world_size) {
    bodies.resize(n);
    bounds = AABB({0, 0, 0}, {world_size, world_size, world_size});
    dt = 0.016f;  // ~60fps

    std::mt19937 rng(42);
    auto rand_float = [&](float lo, float hi) {
        return std::uniform_real_distribution<float>(lo, hi)(rng);
    };

    float obj_size = 1.0f;
    float margin = obj_size * 2.0f;

    switch (scenario) {
        case Scenario::RANDOM_WALK: {
            for (int i = 0; i < n; i++) {
                int kind = i % 4;
                if      (kind == 0) bodies[i].shape = Shape::make_cube(obj_size * 0.5f);
                else if (kind == 1) bodies[i].shape = Shape::make_tetrahedron(obj_size);
                else if (kind == 2) bodies[i].shape = Shape::make_octahedron(obj_size * 0.5f);
                else                bodies[i].shape = Shape::make_icosphere(obj_size * 0.5f);
                bodies[i].position = {
                    rand_float(margin, world_size - margin),
                    rand_float(margin, world_size - margin),
                    rand_float(margin, world_size - margin)
                };
                bodies[i].velocity = {
                    rand_float(-10, 10),
                    rand_float(-10, 10),
                    rand_float(-10, 10)
                };
                bodies[i].update_aabb();
            }
            break;
        }

        case Scenario::TWO_CLUSTER: {
            // Two groups approaching each other
            float center = world_size * 0.5f;
            float cluster_spread = world_size * 0.15f;
            for (int i = 0; i < n; i++) {
                bodies[i].shape = Shape::make_cube(obj_size * 0.5f);
                if (i < n / 2) {
                    // Left cluster moving right
                    bodies[i].position = {
                        center - world_size * 0.25f + rand_float(-cluster_spread, cluster_spread),
                        center + rand_float(-cluster_spread, cluster_spread),
                        center + rand_float(-cluster_spread, cluster_spread)
                    };
                    bodies[i].velocity = {rand_float(5, 15), rand_float(-2, 2), rand_float(-2, 2)};
                } else {
                    // Right cluster moving left
                    bodies[i].position = {
                        center + world_size * 0.25f + rand_float(-cluster_spread, cluster_spread),
                        center + rand_float(-cluster_spread, cluster_spread),
                        center + rand_float(-cluster_spread, cluster_spread)
                    };
                    bodies[i].velocity = {rand_float(-15, -5), rand_float(-2, 2), rand_float(-2, 2)};
                }
                bodies[i].update_aabb();
            }
            break;
        }

        case Scenario::AVALANCHE: {
            // Objects stacked high, falling down (gravity-like)
            float spacing = obj_size * 2.5f;
            int per_row = std::max(1, static_cast<int>(std::cbrt(n)));
            int iz_max = (n - 1) / (per_row * per_row);
            // Fit all layers into the upper half of the world, guaranteed in bounds
            float avail_y = world_size - 2.0f * margin;
            float y_step = (iz_max > 0) ? (avail_y * 0.5f / iz_max) : spacing * 2.0f;
            float y_base = margin + avail_y * 0.5f;  // start at mid-height, stack up
            for (int i = 0; i < n; i++) {
                bodies[i].shape = Shape::make_cube(obj_size * 0.5f);
                int ix = i % per_row;
                int iy = (i / per_row) % per_row;
                int iz = i / (per_row * per_row);
                bodies[i].position = {
                    margin + ix * spacing + rand_float(-0.1f, 0.1f),
                    y_base  + iz * y_step,  // stacked high, always within bounds
                    margin + iy * spacing + rand_float(-0.1f, 0.1f)
                };
                bodies[i].velocity = {
                    rand_float(-1, 1),
                    rand_float(-20, -5),  // falling
                    rand_float(-1, 1)
                };
                bodies[i].update_aabb();
            }
            break;
        }
    }
}

void Scene::step() {
    for (auto& b : bodies) {
        b.position += b.velocity * dt;

        // Reflect off boundaries
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

    // 2. Compute Morton codes
    t.start();
    compute_morton_codes(_centroids, scene_aabb, _codes);
    stage_times.morton_ms = t.stop();

    // 3. Sort by Morton code
    _indices.resize(n);
    std::iota(_indices.begin(), _indices.end(), 0);
    t.start();
    std::sort(_indices.begin(), _indices.end(), [&](int a, int b) {
        return _codes[a] < _codes[b];
    });
    _sorted_codes.resize(n);
    for (int i = 0; i < n; i++) {
        _sorted_codes[i] = _codes[_indices[i]];
    }
    stage_times.sort_ms = t.stop();

    // 4. Build LBVH
    t.start();
    last_bvh.clear();
    last_bvh.build(_sorted_codes, _indices, _aabbs);
    stage_times.build_ms = t.stop();

    // 5. Broad phase: BVH traversal
    t.start();
    last_bvh.traverse(_broad_pairs);
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

std::vector<CollisionPair> Scene::detect_collisions_bruteforce() {
    int n = static_cast<int>(bodies.size());
    std::vector<CollisionPair> pairs;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (AABB::overlaps(bodies[i].world_aabb, bodies[j].world_aabb)) {
                if (gjk_intersect(bodies[i], bodies[j])) {
                    pairs.push_back({i, j});
                }
            }
        }
    }
    return pairs;
}
