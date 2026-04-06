#pragma once
#include "body.h"
#include "bvh.h"
#include "collision.h"
#include <vector>
#include <string>

enum class Scenario {
    RANDOM_WALK,
    TWO_CLUSTER,
    AVALANCHE
};

struct Scene {
    std::vector<Body> bodies;
    AABB bounds;  // world boundary for reflection
    float dt;
    BVH last_bvh;  // BVH from last detection (for visualization)

    // Initialize a scene with n bodies of a given scenario
    void init(int n, Scenario scenario, float world_size = 100.0f);

    // Advance one time step: move bodies, reflect off walls
    void step();

    // Run full collision detection pipeline (sequential):
    //   1. Update AABBs
    //   2. Build LBVH
    //   3. Broad phase traversal
    //   4. Narrow phase GJK
    // Returns confirmed collision pairs.
    std::vector<CollisionPair> detect_collisions_seq();

    // Brute-force O(N^2) for correctness validation
    std::vector<CollisionPair> detect_collisions_bruteforce();
};
