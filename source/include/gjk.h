#pragma once
#include "vec3.h"
#include "body.h"

// GJK intersection test between two bodies.
// Returns true if body A and body B are intersecting.
bool gjk_intersect(const Body& a, const Body& b);
