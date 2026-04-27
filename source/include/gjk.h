#pragma once
#include "vec3.h"
#include "body.h"
#include "shape.h"

// GJK intersection test between two bodies.
// Returns true if body A and body B are intersecting.
bool gjk_intersect(const Body& a, const Body& b);

// GJK intersection test for the OpenMP SoA path.
bool gjk_intersect_soa(ShapeType a_type, const Vec3& a_position,
                       ShapeType b_type, const Vec3& b_position);
