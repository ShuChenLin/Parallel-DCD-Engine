#pragma once
#include <vector>

struct CollisionPair {
    int a, b;
    bool operator==(const CollisionPair& o) const { return a == o.a && b == o.b; }
    bool operator<(const CollisionPair& o) const {
        return a < o.a || (a == o.a && b < o.b);
    }
};

