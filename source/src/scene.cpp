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
                bodies[i].shape = Shape::make_cube(obj_size * 0.5f);
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
            for (int i = 0; i < n; i++) {
                bodies[i].shape = Shape::make_cube(obj_size * 0.5f);
                int ix = i % per_row;
                int iy = (i / per_row) % per_row;
                int iz = i / (per_row * per_row);
                bodies[i].position = {
                    margin + ix * spacing + rand_float(-0.1f, 0.1f),
                    margin + iz * spacing * 2.0f + world_size * 0.3f,  // stacked high
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

        b.update_aabb();
    }
}

std::vector<CollisionPair> Scene::detect_collisions_seq() {
    int n = static_cast<int>(bodies.size());
    if (n == 0) return {};

    // 1. Gather centroids and AABBs
    std::vector<Vec3> centroids(n);
    std::vector<AABB> aabbs(n);
    AABB scene_aabb;
    for (int i = 0; i < n; i++) {
        centroids[i] = bodies[i].world_aabb.centroid();
        aabbs[i] = bodies[i].world_aabb;
        scene_aabb.expand(aabbs[i]);
    }

    // 2. Compute Morton codes
    std::vector<uint32_t> codes;
    compute_morton_codes(centroids, scene_aabb, codes);

    // 3. Sort by Morton code
    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return codes[a] < codes[b];
    });
    std::vector<uint32_t> sorted_codes(n);
    for (int i = 0; i < n; i++) {
        sorted_codes[i] = codes[indices[i]];
    }

    // 4. Build LBVH
    last_bvh = BVH();
    last_bvh.build(sorted_codes, indices, aabbs);

    // 5. Broad phase: BVH traversal
    std::vector<CollisionPair> broad_pairs;
    last_bvh.traverse(broad_pairs);

    // 6. Narrow phase: GJK on each candidate pair
    std::vector<CollisionPair> confirmed;
    for (const auto& p : broad_pairs) {
        if (gjk_intersect(bodies[p.a], bodies[p.b])) {
            confirmed.push_back(p);
        }
    }

    return confirmed;
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
