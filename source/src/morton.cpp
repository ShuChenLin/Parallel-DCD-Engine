#include "morton.h"
#include <algorithm>

uint32_t expand_bits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

uint32_t morton3D(float x, float y, float z) {
    x = std::min(std::max(x * 1024.0f, 0.0f), 1023.0f);
    y = std::min(std::max(y * 1024.0f, 0.0f), 1023.0f);
    z = std::min(std::max(z * 1024.0f, 0.0f), 1023.0f);
    uint32_t xx = expand_bits(static_cast<uint32_t>(x));
    uint32_t yy = expand_bits(static_cast<uint32_t>(y));
    uint32_t zz = expand_bits(static_cast<uint32_t>(z));
    return (xx * 4) + (yy * 2) + zz;
}

void compute_morton_codes(const std::vector<Vec3>& centroids,
                          const AABB& scene_bounds,
                          std::vector<uint32_t>& codes) {
    Vec3 extent = scene_bounds.hi - scene_bounds.lo;
    // Avoid division by zero
    float inv_x = (extent.x > 1e-6f) ? 1.0f / extent.x : 0.0f;
    float inv_y = (extent.y > 1e-6f) ? 1.0f / extent.y : 0.0f;
    float inv_z = (extent.z > 1e-6f) ? 1.0f / extent.z : 0.0f;

    codes.resize(centroids.size());
    for (size_t i = 0; i < centroids.size(); i++) {
        float nx = (centroids[i].x - scene_bounds.lo.x) * inv_x;
        float ny = (centroids[i].y - scene_bounds.lo.y) * inv_y;
        float nz = (centroids[i].z - scene_bounds.lo.z) * inv_z;
        codes[i] = morton3D(nx, ny, nz);
    }
}
