/**
 * @file body.h
 * @brief Rigid-body data used by the engine.
 */

#pragma once
#include "shape.h"
#include "aabb.h"

/**
 * @brief One convex body with pose, velocity, and world AABB.
 */
struct Body {
    Shape shape;
    Vec3 position;
    Vec3 velocity;
    AABB world_aabb;

    /**
     * @brief Recomputes the body's world-space AABB.
     */
    void update_aabb() {
        world_aabb = AABB();
        for (const auto& v : shape.vertices) {
            world_aabb.expand(v + position);
        }
    }

    /**
     * @brief Returns one vertex in world space.
     */
    Vec3 world_vertex(int i) const {
        return shape.vertices[i] + position;
    }
};
