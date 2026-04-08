#pragma once
#include "body.h"
#include "bvh.h"
#include "collision.h"
#include <vector>
#include <string>
#include "timer.h"

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

    // Per-stage timing from last detect_collisions_seq() call
    struct StageTimes {
        double aabb_ms    = 0;
        double morton_ms  = 0;
        double sort_ms    = 0;
        double build_ms   = 0;
        double traverse_ms= 0;
        double gjk_ms     = 0;
    } stage_times;

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

private:
    // Scratch buffers reused each frame to avoid per-frame malloc
    std::vector<Vec3>          _centroids;
    std::vector<AABB>          _aabbs;
    std::vector<uint32_t>      _codes;
    std::vector<int>           _indices;
    std::vector<uint32_t>      _sorted_codes;
    std::vector<CollisionPair> _broad_pairs;
    std::vector<CollisionPair> _confirmed;
};
