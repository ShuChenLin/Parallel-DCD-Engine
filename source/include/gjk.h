/**
 * @file gjk.h
 * @brief Interfaces for GJK narrow-phase tests.
 */

#pragma once
#include "vec3.h"
#include "body.h"
#include "shape.h"

/**
 * @brief Tests whether two bodies intersect with GJK.
 */
bool gjk_intersect(const Body& a, const Body& b);

/**
 * @brief GJK test for the SoA-based OpenMP path.
 */
bool gjk_intersect_soa(ShapeType a_type, const Vec3& a_position,
                       ShapeType b_type, const Vec3& b_position);
