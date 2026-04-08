#pragma once
#include "shape.h"
#include "aabb.h"

struct Body {
    Shape shape;
    Vec3 position;
    Vec3 velocity;
    AABB world_aabb;

    void update_aabb() {
        world_aabb = AABB();
        for (const auto& v : shape.vertices) {
            world_aabb.expand(v + position);
        }
    }

    // Get a world-space vertex
    Vec3 world_vertex(int i) const {
        return shape.vertices[i] + position;
    }
};
