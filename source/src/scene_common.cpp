#include "scene.h"
#include "gjk.h"
#include <random>
#include <algorithm>
#include <cmath>

const char* scenario_name(Scenario s) {
    switch (s) {
        case Scenario::RANDOM_WALK: return "random_walk";
        case Scenario::TWO_CLUSTER: return "two_cluster";
        case Scenario::AVALANCHE:   return "avalanche";
    }
    return "unknown";
}

void Scene::init(int n, Scenario scenario, float world_size) {
    bodies.resize(n);
    bounds = AABB({0, 0, 0}, {world_size, world_size, world_size});
    dt = 0.016f;  // ~60fps
    frames_since_rebuild = 0;

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
            float center = world_size * 0.5f;
            float cluster_spread = world_size * 0.15f;
            for (int i = 0; i < n; i++) {
                bodies[i].shape = Shape::make_cube(obj_size * 0.5f);
                if (i < n / 2) {
                    bodies[i].position = {
                        center - world_size * 0.25f + rand_float(-cluster_spread, cluster_spread),
                        center + rand_float(-cluster_spread, cluster_spread),
                        center + rand_float(-cluster_spread, cluster_spread)
                    };
                    bodies[i].velocity = {rand_float(5, 15), rand_float(-2, 2), rand_float(-2, 2)};
                } else {
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
            float spacing = obj_size * 2.5f;
            int per_row = std::max(1, static_cast<int>(std::cbrt(n)));
            int iz_max = (n - 1) / (per_row * per_row);
            float avail_y = world_size - 2.0f * margin;
            float y_step = (iz_max > 0) ? (avail_y * 0.5f / iz_max) : spacing * 2.0f;
            float y_base = margin + avail_y * 0.5f;
            for (int i = 0; i < n; i++) {
                bodies[i].shape = Shape::make_cube(obj_size * 0.5f);
                int ix = i % per_row;
                int iy = (i / per_row) % per_row;
                int iz = i / (per_row * per_row);
                bodies[i].position = {
                    margin + ix * spacing + rand_float(-0.1f, 0.1f),
                    y_base  + iz * y_step,
                    margin + iy * spacing + rand_float(-0.1f, 0.1f)
                };
                bodies[i].velocity = {
                    rand_float(-1, 1),
                    rand_float(-20, -5),
                    rand_float(-1, 1)
                };
                bodies[i].update_aabb();
            }
            break;
        }
    }
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
