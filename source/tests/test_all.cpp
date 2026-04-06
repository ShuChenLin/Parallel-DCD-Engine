#include "vec3.h"
#include "aabb.h"
#include "morton.h"
#include "bvh.h"
#include "gjk.h"
#include "scene.h"

#include <cstdio>
#include <cmath>
#include <cassert>
#include <vector>
#include <algorithm>
#include <numeric>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    printf("  %-45s ", #name); \
    try { test_##name(); tests_passed++; printf("[PASS]\n"); } \
    catch (...) { tests_failed++; printf("[FAIL]\n"); }

#define ASSERT(cond) do { if (!(cond)) { printf("ASSERT failed: %s (line %d)\n", #cond, __LINE__); throw 1; } } while(0)
#define ASSERT_NEAR(a, b, eps) ASSERT(std::fabs((a)-(b)) < (eps))

// ── Vec3 ─────────────────────────────────────────────────────────────────────
void test_vec3_basic() {
    Vec3 a(1, 2, 3), b(4, 5, 6);
    Vec3 c = a + b;
    ASSERT_NEAR(c.x, 5, 1e-6);
    ASSERT_NEAR(c.y, 7, 1e-6);
    ASSERT_NEAR(c.z, 9, 1e-6);
}

void test_vec3_dot() {
    Vec3 a(1, 0, 0), b(0, 1, 0);
    ASSERT_NEAR(a.dot(b), 0, 1e-6);
    ASSERT_NEAR(a.dot(a), 1, 1e-6);
}

void test_vec3_cross() {
    Vec3 x(1,0,0), y(0,1,0);
    Vec3 z = x.cross(y);
    ASSERT_NEAR(z.x, 0, 1e-6);
    ASSERT_NEAR(z.y, 0, 1e-6);
    ASSERT_NEAR(z.z, 1, 1e-6);
}

void test_vec3_normalize() {
    Vec3 v(3, 4, 0);
    Vec3 n = v.normalized();
    ASSERT_NEAR(n.length(), 1.0, 1e-6);
}

// ── AABB ─────────────────────────────────────────────────────────────────────
void test_aabb_overlap() {
    AABB a({0,0,0}, {2,2,2});
    AABB b({1,1,1}, {3,3,3});
    AABB c({5,5,5}, {6,6,6});
    ASSERT(AABB::overlaps(a, b));
    ASSERT(!AABB::overlaps(a, c));
}

void test_aabb_merge() {
    AABB a({0,0,0}, {1,1,1});
    AABB b({2,2,2}, {3,3,3});
    AABB m = AABB::merge(a, b);
    ASSERT_NEAR(m.lo.x, 0, 1e-6);
    ASSERT_NEAR(m.hi.x, 3, 1e-6);
}

// ── Morton ───────────────────────────────────────────────────────────────────
void test_morton_order() {
    // Points closer in space should have closer Morton codes
    uint32_t c1 = morton3D(0.1f, 0.1f, 0.1f);
    uint32_t c2 = morton3D(0.1f, 0.1f, 0.2f);
    uint32_t c3 = morton3D(0.9f, 0.9f, 0.9f);
    // c1 and c2 should be closer than c1 and c3
    ASSERT(std::abs((int)c1 - (int)c2) < std::abs((int)c1 - (int)c3));
}

void test_morton_batch() {
    std::vector<Vec3> pts = {{1,1,1},{5,5,5},{9,9,9}};
    AABB bounds({0,0,0},{10,10,10});
    std::vector<uint32_t> codes;
    compute_morton_codes(pts, bounds, codes);
    ASSERT(codes.size() == 3);
    ASSERT(codes[0] < codes[1]);
    ASSERT(codes[1] < codes[2]);
}

// ── BVH ──────────────────────────────────────────────────────────────────────
void test_bvh_build_small() {
    // 4 objects at known positions
    std::vector<AABB> aabbs = {
        {{0,0,0},{1,1,1}},
        {{2,2,2},{3,3,3}},
        {{5,5,5},{6,6,6}},
        {{8,8,8},{9,9,9}}
    };
    std::vector<Vec3> centroids;
    AABB scene_box;
    for (auto& a : aabbs) { centroids.push_back(a.centroid()); scene_box.expand(a); }

    std::vector<uint32_t> codes;
    compute_morton_codes(centroids, scene_box, codes);
    std::vector<int> idx(4); std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b){ return codes[a] < codes[b]; });
    std::vector<uint32_t> sorted(4);
    for (int i = 0; i < 4; i++) sorted[i] = codes[idx[i]];

    BVH bvh;
    bvh.build(sorted, idx, aabbs);

    // Root AABB should contain all objects
    ASSERT(AABB::overlaps(bvh.nodes[bvh.root].box, aabbs[0]));
    ASSERT(AABB::overlaps(bvh.nodes[bvh.root].box, aabbs[3]));
}

void test_bvh_traverse_overlapping() {
    // Two overlapping boxes should produce a pair
    std::vector<AABB> aabbs = {
        {{0,0,0},{2,2,2}},
        {{1,1,1},{3,3,3}},
        {{10,10,10},{11,11,11}}
    };
    std::vector<Vec3> centroids;
    AABB scene_box;
    for (auto& a : aabbs) { centroids.push_back(a.centroid()); scene_box.expand(a); }

    std::vector<uint32_t> codes;
    compute_morton_codes(centroids, scene_box, codes);
    std::vector<int> idx(3); std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&](int a, int b){ return codes[a] < codes[b]; });
    std::vector<uint32_t> sorted(3);
    for (int i = 0; i < 3; i++) sorted[i] = codes[idx[i]];

    BVH bvh;
    bvh.build(sorted, idx, aabbs);
    std::vector<CollisionPair> pairs;
    bvh.traverse(pairs);

    // Should find pair (0,1), not (0,2) or (1,2)
    bool found_01 = false;
    for (auto& p : pairs) {
        if ((p.a == 0 && p.b == 1) || (p.a == 1 && p.b == 0)) found_01 = true;
    }
    ASSERT(found_01);
}

// ── GJK ──────────────────────────────────────────────────────────────────────
void test_gjk_overlapping() {
    Body a, b;
    a.shape = Shape::make_cube(1.0f);
    b.shape = Shape::make_cube(1.0f);
    a.position = {0, 0, 0};
    b.position = {1, 0, 0};  // overlapping
    ASSERT(gjk_intersect(a, b));
}

void test_gjk_separated() {
    Body a, b;
    a.shape = Shape::make_cube(1.0f);
    b.shape = Shape::make_cube(1.0f);
    a.position = {0, 0, 0};
    b.position = {5, 0, 0};  // clearly separated
    ASSERT(!gjk_intersect(a, b));
}

void test_gjk_touching() {
    Body a, b;
    a.shape = Shape::make_cube(0.5f);
    b.shape = Shape::make_cube(0.5f);
    a.position = {0, 0, 0};
    b.position = {1.0f, 0, 0};  // exactly touching edge
    // Edge-touching may or may not register; just shouldn't crash
    gjk_intersect(a, b);
}

void test_gjk_tetrahedra() {
    Body a, b;
    a.shape = Shape::make_tetrahedron(2.0f);
    b.shape = Shape::make_tetrahedron(2.0f);
    a.position = {0, 0, 0};
    b.position = {0.5f, 0, 0};
    ASSERT(gjk_intersect(a, b));
    b.position = {10, 0, 0};
    ASSERT(!gjk_intersect(a, b));
}

// ── Scene pipeline (BVH vs brute-force) ──────────────────────────────────────
void test_pipeline_random_walk() {
    Scene s;
    s.init(200, Scenario::RANDOM_WALK);
    s.step();
    auto bvh_result = s.detect_collisions_seq();
    auto bf_result = s.detect_collisions_bruteforce();
    std::sort(bvh_result.begin(), bvh_result.end());
    std::sort(bf_result.begin(), bf_result.end());
    ASSERT(bvh_result == bf_result);
}

void test_pipeline_two_cluster() {
    Scene s;
    s.init(200, Scenario::TWO_CLUSTER);
    s.step();
    auto bvh_result = s.detect_collisions_seq();
    auto bf_result = s.detect_collisions_bruteforce();
    std::sort(bvh_result.begin(), bvh_result.end());
    std::sort(bf_result.begin(), bf_result.end());
    ASSERT(bvh_result == bf_result);
}

void test_pipeline_avalanche() {
    Scene s;
    s.init(200, Scenario::AVALANCHE);
    for (int i = 0; i < 10; i++) s.step();
    auto bvh_result = s.detect_collisions_seq();
    auto bf_result = s.detect_collisions_bruteforce();
    std::sort(bvh_result.begin(), bvh_result.end());
    std::sort(bf_result.begin(), bf_result.end());
    ASSERT(bvh_result == bf_result);
}

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    printf("\n== Vec3 ==\n");
    TEST(vec3_basic);
    TEST(vec3_dot);
    TEST(vec3_cross);
    TEST(vec3_normalize);

    printf("\n== AABB ==\n");
    TEST(aabb_overlap);
    TEST(aabb_merge);

    printf("\n== Morton ==\n");
    TEST(morton_order);
    TEST(morton_batch);

    printf("\n== BVH ==\n");
    TEST(bvh_build_small);
    TEST(bvh_traverse_overlapping);

    printf("\n== GJK ==\n");
    TEST(gjk_overlapping);
    TEST(gjk_separated);
    TEST(gjk_touching);
    TEST(gjk_tetrahedra);

    printf("\n== Pipeline (BVH vs brute-force) ==\n");
    TEST(pipeline_random_walk);
    TEST(pipeline_two_cluster);
    TEST(pipeline_avalanche);

    printf("\n── Results: %d passed, %d failed ──\n\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
