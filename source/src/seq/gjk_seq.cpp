/**
 * @file gjk_seq.cpp
 * @brief Sequential GJK collision tests.
 */

#include "gjk.h"
#include <array>
#include <cmath>

/**
 * @brief Support point of the Minkowski difference for two bodies.
 */
static Vec3 support(const Body& a, const Body& b, const Vec3& d) {
    Vec3 sa = a.position;
    {
        float best = -1e30f;
        for (const auto& v : a.shape.vertices) {
            Vec3 w = v + a.position;
            float proj = w.dot(d);
            if (proj > best) { best = proj; sa = w; }
        }
    }
    Vec3 sb = b.position;
    {
        Vec3 nd = -d;
        float best = -1e30f;
        for (const auto& v : b.shape.vertices) {
            Vec3 w = v + b.position;
            float proj = w.dot(nd);
            if (proj > best) { best = proj; sb = w; }
        }
    }
    return sa - sb;
}

/**
 * @brief Support point used by the SoA GJK path.
 */
static Vec3 support_soa(ShapeType a_type, const Vec3& a_position,
                        ShapeType b_type, const Vec3& b_position,
                        const Vec3& d) {
    Vec3 sa = a_position;
    {
        float best = -1e30f;
        for (const auto& v : shape_vertices(a_type)) {
            Vec3 w = v + a_position;
            float proj = w.dot(d);
            if (proj > best) { best = proj; sa = w; }
        }
    }
    Vec3 sb = b_position;
    {
        Vec3 nd = -d;
        float best = -1e30f;
        for (const auto& v : shape_vertices(b_type)) {
            Vec3 w = v + b_position;
            float proj = w.dot(nd);
            if (proj > best) { best = proj; sb = w; }
        }
    }
    return sa - sb;
}

/**
 * @brief Small simplex container used by GJK.
 */
struct Simplex {
    std::array<Vec3, 4> pts;
    int count = 0;

    void push_front(const Vec3& p) {
        // Shift existing points (cap at 3 to avoid writing pts[4])
        int shift = std::min(count, 3);
        for (int i = shift; i > 0; i--)
            pts[i] = pts[i - 1];
        pts[0] = p;
        if (count < 4) count++;
    }
};

/**
 * @brief Handles the line simplex case in GJK.
 */
static bool do_line(Simplex& s, Vec3& dir) {
    Vec3 a = s.pts[0], b = s.pts[1];
    Vec3 ab = b - a;
    Vec3 ao = -a;

    if (ab.dot(ao) > 0) {
        dir = ab.cross(ao).cross(ab);
        if (dir.length2() < 1e-12f) {
            dir = ab.cross({1, 0, 0});
            if (dir.length2() < 1e-12f)
                dir = ab.cross({0, 1, 0});
        }
    } else {
        s.pts[0] = a;
        s.count = 1;
        dir = ao;
    }
    return false;
}

/**
 * @brief Handles the triangle simplex case in GJK.
 */
static bool do_triangle(Simplex& s, Vec3& dir) {
    Vec3 a = s.pts[0], b = s.pts[1], c = s.pts[2];
    Vec3 ab = b - a, ac = c - a, ao = -a;
    Vec3 abc = ab.cross(ac);

    if (abc.cross(ac).dot(ao) > 0) {
        if (ac.dot(ao) > 0) {
            s.pts[0] = a; s.pts[1] = c; s.count = 2;
            dir = ac.cross(ao).cross(ac);
        } else {
            s.pts[0] = a; s.pts[1] = b; s.count = 2;
            return do_line(s, dir);
        }
    } else {
        if (ab.cross(abc).dot(ao) > 0) {
            s.pts[0] = a; s.pts[1] = b; s.count = 2;
            return do_line(s, dir);
        } else {
            if (abc.dot(ao) > 0) {
                dir = abc;
            } else {
                s.pts[0] = a; s.pts[1] = c; s.pts[2] = b;
                dir = -abc;
            }
        }
    }
    return false;
}

/**
 * @brief Handles the tetrahedron simplex case in GJK.
 */
static bool do_tetrahedron(Simplex& s, Vec3& dir) {
    Vec3 a = s.pts[0], b = s.pts[1], c = s.pts[2], d = s.pts[3];
    Vec3 ab = b - a, ac = c - a, ad = d - a, ao = -a;

    Vec3 abc = ab.cross(ac);
    Vec3 acd = ac.cross(ad);
    Vec3 adb = ad.cross(ab);

    if (abc.dot(ao) > 0) {
        s.pts[0] = a; s.pts[1] = b; s.pts[2] = c; s.count = 3;
        return do_triangle(s, dir);
    }
    if (acd.dot(ao) > 0) {
        s.pts[0] = a; s.pts[1] = c; s.pts[2] = d; s.count = 3;
        return do_triangle(s, dir);
    }
    if (adb.dot(ao) > 0) {
        s.pts[0] = a; s.pts[1] = d; s.pts[2] = b; s.count = 3;
        return do_triangle(s, dir);
    }

    return true;
}

/**
 * @brief Dispatches simplex handling based on simplex size.
 */
static bool do_simplex(Simplex& s, Vec3& dir) {
    switch (s.count) {
        case 2: return do_line(s, dir);
        case 3: return do_triangle(s, dir);
        case 4: return do_tetrahedron(s, dir);
    }
    return false;
}

/**
 * @brief Runs the standard body-based GJK intersection test.
 */
bool gjk_intersect(const Body& a, const Body& b) {
    Vec3 dir = b.position - a.position;
    if (dir.length2() < 1e-10f) dir = {1, 0, 0};

    Simplex simplex;
    Vec3 sup = support(a, b, dir);
    simplex.push_front(sup);
    dir = -sup;

    for (int iter = 0; iter < 64; iter++) {
        sup = support(a, b, dir);

        if (sup.dot(dir) < 1e-6f) {
            return false;  // No intersection
        }

        simplex.push_front(sup);

        if (do_simplex(simplex, dir)) {
            return true;
        }

        if (dir.length2() < 1e-20f) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Runs the SoA-based GJK intersection test.
 */
bool gjk_intersect_soa(ShapeType a_type, const Vec3& a_position,
                       ShapeType b_type, const Vec3& b_position) {
    Vec3 dir = b_position - a_position;
    if (dir.length2() < 1e-10f) dir = {1, 0, 0};

    Simplex simplex;
    Vec3 sup = support_soa(a_type, a_position, b_type, b_position, dir);
    simplex.push_front(sup);
    dir = -sup;

    for (int iter = 0; iter < 64; iter++) {
        sup = support_soa(a_type, a_position, b_type, b_position, dir);

        if (sup.dot(dir) < 1e-6f) {
            return false;
        }

        simplex.push_front(sup);

        if (do_simplex(simplex, dir)) {
            return true;
        }

        if (dir.length2() < 1e-20f) {
            return true;
        }
    }

    return false;
}
