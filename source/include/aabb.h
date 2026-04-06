#pragma once
#include "vec3.h"
#include <limits>

struct AABB {
    Vec3 lo, hi;

    AABB()
        : lo(std::numeric_limits<float>::max(),
             std::numeric_limits<float>::max(),
             std::numeric_limits<float>::max()),
          hi(-std::numeric_limits<float>::max(),
             -std::numeric_limits<float>::max(),
             -std::numeric_limits<float>::max()) {}

    AABB(const Vec3& lo, const Vec3& hi) : lo(lo), hi(hi) {}

    Vec3 centroid() const { return (lo + hi) * 0.5f; }
    float surface_area() const {
        Vec3 d = hi - lo;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    static AABB merge(const AABB& a, const AABB& b) {
        return {Vec3::min(a.lo, b.lo), Vec3::max(a.hi, b.hi)};
    }

    static bool overlaps(const AABB& a, const AABB& b) {
        if (a.lo.x > b.hi.x || b.lo.x > a.hi.x) return false;
        if (a.lo.y > b.hi.y || b.lo.y > a.hi.y) return false;
        if (a.lo.z > b.hi.z || b.lo.z > a.hi.z) return false;
        return true;
    }

    void expand(const Vec3& p) {
        lo = Vec3::min(lo, p);
        hi = Vec3::max(hi, p);
    }

    void expand(const AABB& other) {
        lo = Vec3::min(lo, other.lo);
        hi = Vec3::max(hi, other.hi);
    }
};
