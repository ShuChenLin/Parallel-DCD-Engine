/**
 * @file scene.h
 * @brief Scene state and collision pipeline interfaces.
 */

#pragma once
#include "body.h"
#include "bvh.h"
#include "collision.h"
#include <vector>
#include <string>
#include <cstddef>
#include "timer.h"

/**
 * @brief Lightweight non-owning view used by the CUDA path.
 */
template<typename T>
struct Span {
    T* _ptr; int _n;
    Span() : _ptr(nullptr), _n(0) {}
    Span(T* p, int n) : _ptr(p), _n(n) {}
    T* begin() const { return _ptr; }
    T* end()   const { return _ptr + _n; }
    int  size()  const { return _n; }
    bool empty() const { return _n == 0; }
    T& operator[](int i) const { return _ptr[i]; }
};

/**
 * @brief Benchmark scenarios used in the project.
 */
enum class Scenario {
    RANDOM_WALK,
    TWO_CLUSTER,
    AVALANCHE
};

/**
 * @brief Returns a printable scenario name.
 */
const char* scenario_name(Scenario s);

/**
 * @brief Stores per-stage timing for one detection pass.
 */
struct StageTimes {
    double aabb_ms = 0;
    double morton_ms = 0;
    double sort_ms = 0;
    double build_ms = 0;
    double traverse_ms = 0;
    double gjk_ms = 0;
};

/**
 * @brief Holds scene state and collision-detection scratch buffers.
 */
struct Scene {
    std::vector<Body> bodies;
    AABB bounds;  // world boundary for reflection
    float dt = 0.016f;
    BVH last_bvh;  // BVH from last detection (for visualization)

    // Timing and rebuild control
    StageTimes stage_times;
    int frames_since_rebuild = 0;
    int rebuild_interval = 5;
    Scenario scenario = Scenario::RANDOM_WALK;

    /**
     * @brief Initializes a benchmark scene.
     */
    void init(int n, Scenario scenario, float world_size = 100.0f);

    /**
     * @brief Advances one frame on the sequential path.
     */
    void step();

    /**
     * @brief Advances one frame on the OpenMP path.
     */
    void step_omp();

    /**
     * @brief Runs the sequential collision pipeline.
     */
    std::vector<CollisionPair> detect_collisions_seq();

    /**
     * @brief Runs the OpenMP collision pipeline.
     */
    std::vector<CollisionPair> detect_collisions_omp();

    /**
     * @brief Advances one frame on the CUDA path.
     */
    void step_cuda();

    /**
     * @brief Runs the CUDA collision pipeline.
     */
    Span<CollisionPair> detect_collisions_cuda();

    /**
     * @brief Runs brute-force collision detection for validation.
     */
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
