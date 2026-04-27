#pragma once
#include "vec3.h"
#include "aabb.h"
#include <cstdint>
#include <vector>
#include <cmath>

enum class ShapeType : uint8_t {
    Cube = 0,
    Tetrahedron = 1,
    Octahedron = 2,
    Icosphere = 3
};

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

    // Generate a regular octahedron: 6 vertices at ±scale on each axis
    static Shape make_octahedron(float scale = 1.0f) {
        Shape s;
        s.vertices.push_back({ scale,  0,      0});
        s.vertices.push_back({-scale,  0,      0});
        s.vertices.push_back({ 0,      scale,  0});
        s.vertices.push_back({ 0,     -scale,  0});
        s.vertices.push_back({ 0,      0,      scale});
        s.vertices.push_back({ 0,      0,     -scale});
        return s;
    }

    // Generate a regular icosahedron: 12 vertices, visually approximates a sphere
    static Shape make_icosphere(float scale = 1.0f) {
        const float phi = (1.0f + std::sqrt(5.0f)) / 2.0f;
        const float norm = std::sqrt(1.0f + phi * phi);
        const float a = scale / norm;
        const float b = scale * phi / norm;
        Shape s;
        s.vertices = {
            { 0,  a,  b}, { 0, -a,  b}, { 0,  a, -b}, { 0, -a, -b},
            { a,  b,  0}, {-a,  b,  0}, { a, -b,  0}, {-a, -b,  0},
            { b,  0,  a}, {-b,  0,  a}, { b,  0, -a}, {-b,  0, -a},
        };
        return s;
    }
};

inline const std::vector<Vec3>& shape_vertices(ShapeType type) {
    static const Shape cube = Shape::make_cube(0.5f);
    static const Shape tetrahedron = Shape::make_tetrahedron(1.0f);
    static const Shape octahedron = Shape::make_octahedron(0.5f);
    static const Shape icosphere = Shape::make_icosphere(0.5f);

    switch (type) {
        case ShapeType::Cube:        return cube.vertices;
        case ShapeType::Tetrahedron: return tetrahedron.vertices;
        case ShapeType::Octahedron:  return octahedron.vertices;
        case ShapeType::Icosphere:   return icosphere.vertices;
    }
    return cube.vertices;
}

inline Shape make_shape(ShapeType type) {
    Shape s;
    s.vertices = shape_vertices(type);
    return s;
}

inline AABB compute_shape_aabb(ShapeType type, const Vec3& position) {
    AABB box;
    for (const auto& v : shape_vertices(type)) {
        box.expand(v + position);
    }
    return box;
}
