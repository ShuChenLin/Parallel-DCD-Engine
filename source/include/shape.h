#pragma once
#include "vec3.h"
#include "aabb.h"
#include <vector>
#include <cmath>

struct Shape {
    std::vector<Vec3> vertices;

    // Return the vertex furthest along direction d
    Vec3 support(const Vec3& d) const {
        float best = -1e30f;
        Vec3 result = vertices[0];
        for (const auto& v : vertices) {
            float proj = v.dot(d);
            if (proj > best) {
                best = proj;
                result = v;
            }
        }
        return result;
    }

    AABB compute_aabb() const {
        AABB box;
        for (const auto& v : vertices)
            box.expand(v);
        return box;
    }

    // Generate a unit cube [-0.5, 0.5]^3
    static Shape make_cube(float half_extent = 0.5f) {
        Shape s;
        for (int i = 0; i < 8; i++) {
            float x = (i & 1) ? half_extent : -half_extent;
            float y = (i & 2) ? half_extent : -half_extent;
            float z = (i & 4) ? half_extent : -half_extent;
            s.vertices.push_back({x, y, z});
        }
        return s;
    }

    // Generate a regular tetrahedron
    static Shape make_tetrahedron(float scale = 1.0f) {
        Shape s;
        s.vertices.push_back({1, 1, 1});
        s.vertices.push_back({1, -1, -1});
        s.vertices.push_back({-1, 1, -1});
        s.vertices.push_back({-1, -1, 1});
        for (auto& v : s.vertices) v = v * (scale * 0.5f);
        return s;
    }

    static Shape make_octahedron(float half_extent = 0.5f) {
        Shape s;
        float h = half_extent;
        s.vertices.push_back({ h, 0, 0});
        s.vertices.push_back({-h, 0, 0});
        s.vertices.push_back({0,  h, 0});
        s.vertices.push_back({0, -h, 0});
        s.vertices.push_back({0, 0,  h});
        s.vertices.push_back({0, 0, -h});
        return s;
    }

    static Shape make_icosphere(float radius = 0.5f) {
        Shape s;
        float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
        float scale = radius / std::sqrt(1.0f + t * t);
        float a = 1.0f * scale, b = t * scale;
        s.vertices.push_back({-a,  b, 0}); s.vertices.push_back({ a,  b, 0});
        s.vertices.push_back({-a, -b, 0}); s.vertices.push_back({ a, -b, 0});
        s.vertices.push_back({0, -a,  b}); s.vertices.push_back({0,  a,  b});
        s.vertices.push_back({0, -a, -b}); s.vertices.push_back({0,  a, -b});
        s.vertices.push_back({ b, 0, -a}); s.vertices.push_back({ b, 0,  a});
        s.vertices.push_back({-b, 0, -a}); s.vertices.push_back({-b, 0,  a});
        return s;
    }
};
