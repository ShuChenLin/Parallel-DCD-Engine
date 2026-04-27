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

const char* scenario_name(Scenario s);

struct StageTimes {
    double aabb_ms = 0;
    double morton_ms = 0;
    double sort_ms = 0;
    double build_ms = 0;
    double traverse_ms = 0;
    double gjk_ms = 0;
};

struct Scene {
    std::vector<Body> bodies;
    AABB bounds;  // world boundary for reflection
    float dt = 0.016f;
    BVH last_bvh;  // BVH from last detection (for visualization)

    // Timing and rebuild control
    StageTimes stage_times;
    int frames_since_rebuild = 0;
    int rebuild_interval = 5;

    // Initialize a scene with n bodies of a given scenario
    void init(int n, Scenario scenario, float world_size = 100.0f);

    // Advance one time step (sequential)
    void step();

    // Advance one time step (OpenMP)
    void step_omp();

    // Full collision detection pipeline (sequential)
    std::vector<CollisionPair> detect_collisions_seq();

    // Full collision detection pipeline (OpenMP)
    std::vector<CollisionPair> detect_collisions_omp();

    // Advance one time step (CUDA)
    void step_cuda();

    // Full collision detection pipeline (CUDA)
    std::vector<CollisionPair> detect_collisions_cuda();

    // Brute-force O(N^2) for correctness validation
    std::vector<CollisionPair> detect_collisions_bruteforce();

    // Scratch buffers (reused across frames)
    std::vector<Vec3> _positions;
    std::vector<Vec3> _velocities;
    std::vector<ShapeType> _shape_types;
    std::vector<Vec3> _centroids;
    std::vector<AABB> _aabbs;
    std::vector<uint32_t> _codes;
    std::vector<int> _indices;
    std::vector<uint32_t> _sorted_codes;
    std::vector<CollisionPair> _broad_pairs;
    std::vector<CollisionPair> _confirmed;
};
