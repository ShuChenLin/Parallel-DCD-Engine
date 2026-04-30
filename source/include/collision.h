/**
 * @file collision.h
 * @brief Small types for collision results.
 */

#pragma once
#include <vector>

/**
 * @brief Stores one colliding object pair.
 */
struct CollisionPair {
    int a, b;
    bool operator==(const CollisionPair& o) const { return a == o.a && b == o.b; }
    bool operator<(const CollisionPair& o) const {
        return a < o.a || (a == o.a && b < o.b);
    }
};
