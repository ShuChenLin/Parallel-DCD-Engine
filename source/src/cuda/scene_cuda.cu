#include "scene.h"
#include "timer.h"
#include "cuda_utils.cuh"

#include <thrust/sort.h>
#include <thrust/device_ptr.h>
#include <thrust/sequence.h>

#include <vector>
#include <algorithm>

/* ================================================================
   Global persistent GPU context
   ================================================================ */
static CUDAContext g_ctx;

/* ================================================================
   1. AABB / centroid / Morton-code kernels
   ================================================================ */

__global__ void compute_aabb_kernel(
    const float3* __restrict__ positions,
    const float3* __restrict__ verts,   // AoS: [body * MAX_VERTS + j]
    const int*    __restrict__ vcnt,
    int n,
    float3* __restrict__ aabb_lo,
    float3* __restrict__ aabb_hi,
    float3* __restrict__ centroids)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float3 pos = positions[i];
    int nv = vcnt[i];
    const float3* vi = verts + (size_t)i * MAX_VERTS;

    float3 lo = pos + vi[0];
    float3 hi = lo;
    for (int j = 1; j < nv; j++) {
        float3 w = pos + vi[j];
        lo = f3min(lo, w);
        hi = f3max(hi, w);
    }
    aabb_lo[i]   = lo;
    aabb_hi[i]   = hi;
    centroids[i] = (lo + hi) * 0.5f;
}

/* Parallel scene-AABB reduction (one block per chunk) */
__global__ void reduce_aabb_kernel(
    const float3* __restrict__ lo_in,
    const float3* __restrict__ hi_in,
    int n,
    float3* __restrict__ blk_lo,
    float3* __restrict__ blk_hi)
{
    extern __shared__ float3 smem[];          // 2 * blockDim.x
    float3* s_lo = smem;
    float3* s_hi = smem + blockDim.x;

    int tid = threadIdx.x;
    float3 my_lo = make_float3( FLT_MAX,  FLT_MAX,  FLT_MAX);
    float3 my_hi = make_float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (int i = blockIdx.x * blockDim.x + tid; i < n;
         i += blockDim.x * gridDim.x) {
        my_lo = f3min(my_lo, lo_in[i]);
        my_hi = f3max(my_hi, hi_in[i]);
    }
    s_lo[tid] = my_lo;
    s_hi[tid] = my_hi;
    __syncthreads();

    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            s_lo[tid] = f3min(s_lo[tid], s_lo[tid + s]);
            s_hi[tid] = f3max(s_hi[tid], s_hi[tid + s]);
        }
        __syncthreads();
    }
    if (tid == 0) {
        blk_lo[blockIdx.x] = s_lo[0];
        blk_hi[blockIdx.x] = s_hi[0];
    }
}

/* Morton code helpers */
__device__ uint32_t d_expand_bits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

__device__ uint32_t d_morton3D(float x, float y, float z) {
    x = fminf(fmaxf(x * 1024.f, 0.f), 1023.f);
    y = fminf(fmaxf(y * 1024.f, 0.f), 1023.f);
    z = fminf(fmaxf(z * 1024.f, 0.f), 1023.f);
    uint32_t xx = d_expand_bits((uint32_t)x);
    uint32_t yy = d_expand_bits((uint32_t)y);
    uint32_t zz = d_expand_bits((uint32_t)z);
    return (xx << 2) | (yy << 1) | zz;
}

__global__ void morton_kernel(
    const float3* __restrict__ centroids,
    float3 scene_lo, float3 scene_inv,
    int n,
    uint32_t* __restrict__ codes)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    float3 c = centroids[i];
    float nx = (c.x - scene_lo.x) * scene_inv.x;
    float ny = (c.y - scene_lo.y) * scene_inv.y;
    float nz = (c.z - scene_lo.z) * scene_inv.z;
    codes[i] = d_morton3D(nx, ny, nz);
}

/* ================================================================
   2. LBVH build (Karras 2012)
   ================================================================ */

__device__ int d_clz(uint32_t x) {
    return x == 0 ? 32 : __clz(x);
}

__device__ int d_delta(const uint32_t* codes, int n, int i, int j) {
    if (j < 0 || j >= n) return -1;
    if (codes[i] == codes[j])
        return 32 + d_clz((uint32_t)(i ^ j));
    return d_clz(codes[i] ^ codes[j]);
}

__device__ void d_determine_range(const uint32_t* codes, int n,
                                  int i, int& out_l, int& out_r)
{
    int dl = d_delta(codes, n, i, i - 1);
    int dr = d_delta(codes, n, i, i + 1);
    int d  = (dr >= dl) ? 1 : -1;

    int d_min = d_delta(codes, n, i, i - d);
    int lmax  = 2;
    while (d_delta(codes, n, i, i + lmax * d) > d_min) lmax *= 2;

    int l = 0;
    for (int t = lmax / 2; t >= 1; t /= 2)
        if (d_delta(codes, n, i, i + (l + t) * d) > d_min) l += t;
    int j = i + l * d;

    out_l = min(i, j);
    out_r = max(i, j);
}

__device__ int d_find_split(const uint32_t* codes, int left, int right) {
    uint32_t fc = codes[left], lc = codes[right];
    if (fc == lc) return (left + right) / 2;
    int cp = d_clz(fc ^ lc);
    int split = left, step = right - left;
    do {
        step = (step + 1) / 2;
        int ns = split + step;
        if (ns < right && d_clz(fc ^ codes[ns]) > cp)
            split = ns;
    } while (step > 1);
    return split;
}

/* Initialise leaf nodes */
__global__ void build_leaves_kernel(
    GBVHNode*       nodes,
    const int*      sorted_idx,
    const float3*   aabb_lo,
    const float3*   aabb_hi,
    int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    int leaf = n - 1 + i;
    int oi   = sorted_idx[i];
    GBVHNode& nd = nodes[leaf];
    nd.lo_x = aabb_lo[oi].x; nd.lo_y = aabb_lo[oi].y; nd.lo_z = aabb_lo[oi].z;
    nd.hi_x = aabb_hi[oi].x; nd.hi_y = aabb_hi[oi].y; nd.hi_z = aabb_hi[oi].z;
    nd.left  = -1;
    nd.right = -1;
    nd.parent = -1;
    nd.object_idx = oi;
}

/* Build internal nodes (Karras algorithm) */
__global__ void build_internal_kernel(
    GBVHNode*       nodes,
    const uint32_t* sorted_codes,
    int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n - 1) return;

    int lb, rb;
    d_determine_range(sorted_codes, n, i, lb, rb);
    int split = d_find_split(sorted_codes, lb, rb);

    int lc = (split == lb)     ? (n - 1 + split)     : split;
    int rc = (split + 1 == rb) ? (n - 1 + split + 1) : (split + 1);

    nodes[i].left  = lc;
    nodes[i].right = rc;
    nodes[i].object_idx = -1;
    nodes[lc].parent = i;
    nodes[rc].parent = i;
}

/* Atomic bottom-up refit */
__global__ void refit_kernel(
    GBVHNode*     nodes,
    const float3* aabb_lo,
    const float3* aabb_hi,
    int*          flags,
    int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    /* refresh leaf AABB */
    int leaf = n - 1 + i;
    int oi   = nodes[leaf].object_idx;
    nodes[leaf].lo_x = aabb_lo[oi].x; nodes[leaf].lo_y = aabb_lo[oi].y; nodes[leaf].lo_z = aabb_lo[oi].z;
    nodes[leaf].hi_x = aabb_hi[oi].x; nodes[leaf].hi_y = aabb_hi[oi].y; nodes[leaf].hi_z = aabb_hi[oi].z;

    int node = nodes[leaf].parent;
    while (node >= 0) {
        if (atomicAdd(&flags[node], 1) == 0) return;   // first arrival → quit
        /* second arrival: merge children */
        int lc = nodes[node].left;
        int rc = nodes[node].right;
        nodes[node].lo_x = fminf(nodes[lc].lo_x, nodes[rc].lo_x);
        nodes[node].lo_y = fminf(nodes[lc].lo_y, nodes[rc].lo_y);
        nodes[node].lo_z = fminf(nodes[lc].lo_z, nodes[rc].lo_z);
        nodes[node].hi_x = fmaxf(nodes[lc].hi_x, nodes[rc].hi_x);
        nodes[node].hi_y = fmaxf(nodes[lc].hi_y, nodes[rc].hi_y);
        nodes[node].hi_z = fmaxf(nodes[lc].hi_z, nodes[rc].hi_z);
        node = nodes[node].parent;
    }
}

/* ================================================================
   3. BVH traversal (broad phase)
   ================================================================ */

__global__ void traverse_kernel(
    const GBVHNode* __restrict__ nodes,
    int n,
    int2* __restrict__ out_pairs,
    int*  __restrict__ out_count,
    int   max_pairs,
    int*  __restrict__ overflow)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    int leaf = n - 1 + i;
    int qobj = nodes[leaf].object_idx;
    float qlox = nodes[leaf].lo_x, qloy = nodes[leaf].lo_y, qloz = nodes[leaf].lo_z;
    float qhix = nodes[leaf].hi_x, qhiy = nodes[leaf].hi_y, qhiz = nodes[leaf].hi_z;

    int stack[TRAVERSE_STACK_SIZE];
    int top = 0;
    stack[top++] = 0;                       // push root

    while (top > 0) {
        int idx = stack[--top];
        const GBVHNode& nd = nodes[idx];

        /* AABB overlap test */
        if (qlox > nd.hi_x || nd.lo_x > qhix) continue;
        if (qloy > nd.hi_y || nd.lo_y > qhiy) continue;
        if (qloz > nd.hi_z || nd.lo_z > qhiz) continue;

        if (gnode_is_leaf(nd)) {
            int other = nd.object_idx;
            bool emit = (qobj < other);

            /* Warp ballot: all active threads vote, leader does one atomicAdd
               for the whole warp, then each thread writes to its reserved slot.
               Reduces atomicAdd contention by up to 32x. */
            unsigned active  = __activemask();
            unsigned ballot  = __ballot_sync(active, emit);
            if (ballot) {
                int lane        = threadIdx.x & 31;
                int leader      = __ffs(ballot) - 1;
                int warp_count  = __popc(ballot);
                int base        = 0;
                if (lane == leader)
                    base = atomicAdd(out_count, warp_count);
                base = __shfl_sync(active, base, leader);
                if (emit) {
                    int rank = __popc(ballot & ((1u << lane) - 1));
                    int pos  = base + rank;
                    if (pos < max_pairs)
                        out_pairs[pos] = make_int2(qobj, other);
                    else
                        *overflow = 1;
                }
            }
        } else {
            stack[top++] = nd.left;
            stack[top++] = nd.right;
        }
    }
}

/* ================================================================
   4. GJK narrow-phase kernel
   ================================================================ */

/* Support function reading world-space vertices from shared memory */
__device__ float3 gjk_support_smem(
    const float3* __restrict__ va, int na,
    const float3* __restrict__ vb, int nb,
    float3 dir)
{
    float best_a = -1e30f;
    float3 sa = va[0];
    for (int j = 0; j < na; j++) {
        float proj = dot3(va[j], dir);
        if (proj > best_a) { best_a = proj; sa = va[j]; }
    }
    float3 nd = -dir;
    float best_b = -1e30f;
    float3 sb = vb[0];
    for (int j = 0; j < nb; j++) {
        float proj = dot3(vb[j], nd);
        if (proj > best_b) { best_b = proj; sb = vb[j]; }
    }
    return sa - sb;
}

/* Simplex helpers (device) */

__device__ bool gjk_do_line(float3 pts[4], int& cnt, float3& dir) {
    float3 a = pts[0], b = pts[1];
    float3 ab = b - a, ao = -a;
    if (dot3(ab, ao) > 0) {
        dir = cross3(cross3(ab, ao), ab);
        if (len2_f3(dir) < 1e-12f) {
            dir = cross3(ab, make_float3(1, 0, 0));
            if (len2_f3(dir) < 1e-12f)
                dir = cross3(ab, make_float3(0, 1, 0));
        }
    } else {
        pts[0] = a; cnt = 1; dir = ao;
    }
    return false;
}

__device__ bool gjk_do_triangle(float3 pts[4], int& cnt, float3& dir) {
    float3 a = pts[0], b = pts[1], c = pts[2];
    float3 ab = b - a, ac = c - a, ao = -a;
    float3 abc = cross3(ab, ac);

    if (dot3(cross3(abc, ac), ao) > 0) {
        if (dot3(ac, ao) > 0) {
            pts[0] = a; pts[1] = c; cnt = 2;
            dir = cross3(cross3(ac, ao), ac);
        } else {
            pts[0] = a; pts[1] = b; cnt = 2;
            return gjk_do_line(pts, cnt, dir);
        }
    } else {
        if (dot3(cross3(ab, abc), ao) > 0) {
            pts[0] = a; pts[1] = b; cnt = 2;
            return gjk_do_line(pts, cnt, dir);
        } else {
            if (dot3(abc, ao) > 0) {
                dir = abc;
            } else {
                pts[0] = a; pts[1] = c; pts[2] = b;
                dir = -abc;
            }
        }
    }
    return false;
}

__device__ bool gjk_do_tetra(float3 pts[4], int& cnt, float3& dir) {
    float3 a = pts[0], b = pts[1], c = pts[2], d = pts[3];
    float3 ab = b - a, ac = c - a, ad = d - a, ao = -a;
    float3 abc_n = cross3(ab, ac);
    float3 acd_n = cross3(ac, ad);
    float3 adb_n = cross3(ad, ab);

    if (dot3(abc_n, ao) > 0) {
        pts[0]=a; pts[1]=b; pts[2]=c; cnt=3;
        return gjk_do_triangle(pts, cnt, dir);
    }
    if (dot3(acd_n, ao) > 0) {
        pts[0]=a; pts[1]=c; pts[2]=d; cnt=3;
        return gjk_do_triangle(pts, cnt, dir);
    }
    if (dot3(adb_n, ao) > 0) {
        pts[0]=a; pts[1]=d; pts[2]=b; cnt=3;
        return gjk_do_triangle(pts, cnt, dir);
    }
    return true;
}

__device__ bool gjk_do_simplex(float3 pts[4], int& cnt, float3& dir) {
    switch (cnt) {
        case 2: return gjk_do_line(pts, cnt, dir);
        case 3: return gjk_do_triangle(pts, cnt, dir);
        case 4: return gjk_do_tetra(pts, cnt, dir);
    }
    return false;
}

__device__ void simplex_push_front(float3 pts[4], int& cnt, float3 p) {
    int shift = cnt < 4 ? cnt : 3;
    for (int k = shift; k > 0; k--) pts[k] = pts[k - 1];
    pts[0] = p;
    if (cnt < 4) cnt++;
}

/* GJK using world-space vertices pre-loaded into shared memory */
__device__ bool gjk_intersect_dev(
    const float3* va, int na, float3 pos_a,
    const float3* vb, int nb, float3 pos_b)
{
    float3 dir = pos_b - pos_a;
    if (len2_f3(dir) < 1e-10f) dir = make_float3(1, 0, 0);

    float3 pts[4];
    int cnt = 0;
    float3 sup = gjk_support_smem(va, na, vb, nb, dir);
    simplex_push_front(pts, cnt, sup);
    dir = -sup;

    for (int iter = 0; iter < 64; iter++) {
        sup = gjk_support_smem(va, na, vb, nb, dir);
        if (dot3(sup, dir) < 1e-6f) return false;
        simplex_push_front(pts, cnt, sup);
        if (gjk_do_simplex(pts, cnt, dir)) return true;
        if (len2_f3(dir) < 1e-20f) return true;
    }
    return false;
}

__global__ void gjk_kernel(
    const int2*   __restrict__ broad_pairs,
    int           num_broad,
    const float3* __restrict__ verts,   // AoS: [body * MAX_VERTS + j]
    const int*    __restrict__ vcnt,
    const float3* __restrict__ positions,
    int2*         __restrict__ out_pairs,
    int*          __restrict__ out_count,
    int           max_out)
{
    __shared__ float3 smem_a[GJK_BLOCK_SIZE][MAX_VERTS];
    __shared__ float3 smem_b[GJK_BLOCK_SIZE][MAX_VERTS];

    int i   = blockIdx.x * blockDim.x + threadIdx.x;
    int tid = threadIdx.x;
    if (i >= num_broad) return;

    int2   p      = broad_pairs[i];
    int    body_a = p.x, body_b = p.y;
    float3 pos_a  = positions[body_a];
    float3 pos_b  = positions[body_b];
    int    na     = vcnt[body_a];
    int    nb     = vcnt[body_b];

    const float3* va = verts + (size_t)body_a * MAX_VERTS;
    const float3* vb = verts + (size_t)body_b * MAX_VERTS;
    for (int j = 0; j < na; j++) smem_a[tid][j] = pos_a + va[j];
    for (int j = 0; j < nb; j++) smem_b[tid][j] = pos_b + vb[j];

    if (gjk_intersect_dev(smem_a[tid], na, pos_a, smem_b[tid], nb, pos_b)) {
        int pos = atomicAdd(out_count, 1);
        if (pos < max_out) out_pairs[pos] = p;
    }
}

/* ================================================================
   5. Physics step kernel
   ================================================================ */

__global__ void step_kernel(
    float3* __restrict__ positions,
    float3* __restrict__ velocities,
    float3 bnd_lo, float3 bnd_hi,
    float dt, int n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float3 p = positions[i];
    float3 v = velocities[i];
    p = p + v * dt;

    /* reflect on each axis */
    auto reflect = [](float& pos, float& vel, float lo, float hi) {
        if (pos < lo) { pos = lo + (lo - pos); vel = fabsf(vel); }
        if (pos > hi) { pos = hi - (pos - hi); vel = -fabsf(vel); }
    };
    reflect(p.x, v.x, bnd_lo.x, bnd_hi.x);
    reflect(p.y, v.y, bnd_lo.y, bnd_hi.y);
    reflect(p.z, v.z, bnd_lo.z, bnd_hi.z);

    positions[i]  = p;
    velocities[i] = v;
}

/* ================================================================
   Host helpers: upload / download body data
   ================================================================ */

/* Upload static shape data (vertices + vert_counts) — called once per scene init */
static void upload_shapes(const Scene& scene, CUDAContext& ctx, int n) {
    std::vector<float3> h_verts((size_t)n * MAX_VERTS, make_float3(0, 0, 0));
    std::vector<int>    h_vcnt(n);
    for (int i = 0; i < n; i++) {
        const auto& b = scene.bodies[i];
        int nv = std::min((int)b.shape.vertices.size(), MAX_VERTS);
        h_vcnt[i] = nv;
        for (int j = 0; j < nv; j++) {
            const auto& sv = b.shape.vertices[j];
            h_verts[(size_t)i * MAX_VERTS + j] = make_float3(sv.x, sv.y, sv.z);
        }
    }
    CUDA_CHECK(cudaMemcpy(ctx.d_vertices,    h_verts.data(), (size_t)n * MAX_VERTS * sizeof(float3), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(ctx.d_vert_counts, h_vcnt.data(),  n * sizeof(int), cudaMemcpyHostToDevice));
    ctx.shapes_uploaded = true;
}

/* Upload dynamic data (positions + velocities) — called every frame */
static void upload_dynamic(const Scene& scene, CUDAContext& ctx, int n) {
    std::vector<float3> h_pos(n), h_vel(n);
    for (int i = 0; i < n; i++) {
        h_pos[i] = make_float3(scene.bodies[i].position.x, scene.bodies[i].position.y, scene.bodies[i].position.z);
        h_vel[i] = make_float3(scene.bodies[i].velocity.x, scene.bodies[i].velocity.y, scene.bodies[i].velocity.z);
    }
    CUDA_CHECK(cudaMemcpy(ctx.d_positions,  h_pos.data(), n * sizeof(float3), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(ctx.d_velocities, h_vel.data(), n * sizeof(float3), cudaMemcpyHostToDevice));
}

static void download_positions(Scene& scene, const CUDAContext& ctx, int n) {
    std::vector<float3> h_pos(n), h_vel(n);
    CUDA_CHECK(cudaMemcpy(h_pos.data(), ctx.d_positions,  n * sizeof(float3), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(h_vel.data(), ctx.d_velocities, n * sizeof(float3), cudaMemcpyDeviceToHost));
    for (int i = 0; i < n; i++) {
        scene.bodies[i].position = {h_pos[i].x, h_pos[i].y, h_pos[i].z};
        scene.bodies[i].velocity = {h_vel[i].x, h_vel[i].y, h_vel[i].z};
    }
}

/* ================================================================
   Scene::step_cuda
   ================================================================ */

void Scene::step_cuda() {
    int n = (int)bodies.size();
    if (n == 0) return;
    g_ctx.ensure(n);

    if (!g_ctx.shapes_uploaded || last_bvh.nodes.empty())
        upload_shapes(*this, g_ctx, n);
    upload_dynamic(*this, g_ctx, n);

    float3 blo = make_float3(bounds.lo.x, bounds.lo.y, bounds.lo.z);
    float3 bhi = make_float3(bounds.hi.x, bounds.hi.y, bounds.hi.z);

    int blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    step_kernel<<<blocks, BLOCK_SIZE>>>(
        g_ctx.d_positions, g_ctx.d_velocities, blo, bhi, dt, n);
    CUDA_CHECK(cudaGetLastError());

    download_positions(*this, g_ctx, n);
}

/* ================================================================
   Scene::detect_collisions_cuda
   ================================================================ */

Span<CollisionPair> Scene::detect_collisions_cuda() {
    int n = (int)bodies.size();
    if (n == 0) return {};

    g_ctx.ensure(n);
    /* positions/velocities already on GPU from step_cuda — no re-upload needed */

    int grid = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;

    cudaEvent_t ev0, ev1;
    CUDA_CHECK(cudaEventCreate(&ev0));
    CUDA_CHECK(cudaEventCreate(&ev1));
    float ms;

    /* --- 1. Compute AABBs + centroids --- */
    CUDA_CHECK(cudaEventRecord(ev0));
    compute_aabb_kernel<<<grid, BLOCK_SIZE>>>(
        g_ctx.d_positions, g_ctx.d_vertices, g_ctx.d_vert_counts, n,
        g_ctx.d_aabb_lo, g_ctx.d_aabb_hi, g_ctx.d_centroids);
    CUDA_CHECK(cudaEventRecord(ev1)); CUDA_CHECK(cudaEventSynchronize(ev1));
    CUDA_CHECK(cudaEventElapsedTime(&ms, ev0, ev1));
    stage_times.aabb_ms = ms;

    /* --- Scene AABB reduction --- */
    int rb = g_ctx.num_reduce_blocks;
    size_t smem = 2 * BLOCK_SIZE * sizeof(float3);
    reduce_aabb_kernel<<<rb, BLOCK_SIZE, smem>>>(
        g_ctx.d_aabb_lo, g_ctx.d_aabb_hi, n,
        g_ctx.d_block_lo, g_ctx.d_block_hi);
    CUDA_CHECK(cudaGetLastError());

    /* finish reduction on host */
    std::vector<float3> h_blo(rb), h_bhi(rb);
    CUDA_CHECK(cudaMemcpy(h_blo.data(), g_ctx.d_block_lo, rb * sizeof(float3), cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaMemcpy(h_bhi.data(), g_ctx.d_block_hi, rb * sizeof(float3), cudaMemcpyDeviceToHost));
    float3 scene_lo = h_blo[0], scene_hi = h_bhi[0];
    for (int i = 1; i < rb; i++) {
        scene_lo = f3min(scene_lo, h_blo[i]);
        scene_hi = f3max(scene_hi, h_bhi[i]);
    }

    /* Decide rebuild vs refit */
    bool full_rebuild = (frames_since_rebuild >= rebuild_interval)
                     || (last_bvh.nodes.empty());

    if (full_rebuild) {
        /* --- 2. Morton codes --- */
        float3 ext = scene_hi - scene_lo;
        float3 inv = make_float3(
            ext.x > 1e-6f ? 1.f / ext.x : 0.f,
            ext.y > 1e-6f ? 1.f / ext.y : 0.f,
            ext.z > 1e-6f ? 1.f / ext.z : 0.f);

        CUDA_CHECK(cudaEventRecord(ev0));
        morton_kernel<<<grid, BLOCK_SIZE>>>(
            g_ctx.d_centroids, scene_lo, inv, n, g_ctx.d_codes);
        CUDA_CHECK(cudaEventRecord(ev1)); CUDA_CHECK(cudaEventSynchronize(ev1));
        CUDA_CHECK(cudaEventElapsedTime(&ms, ev0, ev1));
        stage_times.morton_ms = ms;

        /* --- 3. Sort (thrust) --- */
        CUDA_CHECK(cudaEventRecord(ev0));
        CUDA_CHECK(cudaMemcpy(g_ctx.d_sorted_codes, g_ctx.d_codes,
                              n * sizeof(uint32_t), cudaMemcpyDeviceToDevice));
        thrust::device_ptr<uint32_t> kp(g_ctx.d_sorted_codes);
        thrust::device_ptr<int>      vp(g_ctx.d_sorted_indices);
        thrust::sequence(vp, vp + n);
        thrust::sort_by_key(kp, kp + n, vp);
        CUDA_CHECK(cudaEventRecord(ev1)); CUDA_CHECK(cudaEventSynchronize(ev1));
        CUDA_CHECK(cudaEventElapsedTime(&ms, ev0, ev1));
        stage_times.sort_ms = ms;

        /* --- 4. Build LBVH --- */
        CUDA_CHECK(cudaEventRecord(ev0));
        build_leaves_kernel<<<grid, BLOCK_SIZE>>>(
            g_ctx.d_nodes, g_ctx.d_sorted_indices,
            g_ctx.d_aabb_lo, g_ctx.d_aabb_hi, n);
        if (n > 1) {
            int igrid = (n - 1 + BLOCK_SIZE - 1) / BLOCK_SIZE;
            build_internal_kernel<<<igrid, BLOCK_SIZE>>>(
                g_ctx.d_nodes, g_ctx.d_sorted_codes, n);
        }
        /* set root parent to -1 */
        GBVHNode root_patch;
        CUDA_CHECK(cudaMemcpy(&root_patch, &g_ctx.d_nodes[0], sizeof(GBVHNode), cudaMemcpyDeviceToHost));
        root_patch.parent = -1;
        CUDA_CHECK(cudaMemcpy(&g_ctx.d_nodes[0], &root_patch, sizeof(GBVHNode), cudaMemcpyHostToDevice));

        /* refit (atomic bottom-up) */
        if (n > 1) {
            CUDA_CHECK(cudaMemset(g_ctx.d_flags, 0, (n - 1) * sizeof(int)));
            refit_kernel<<<grid, BLOCK_SIZE>>>(
                g_ctx.d_nodes, g_ctx.d_aabb_lo, g_ctx.d_aabb_hi,
                g_ctx.d_flags, n);
        }
        CUDA_CHECK(cudaEventRecord(ev1)); CUDA_CHECK(cudaEventSynchronize(ev1));
        CUDA_CHECK(cudaEventElapsedTime(&ms, ev0, ev1));
        stage_times.build_ms = ms;

        frames_since_rebuild = 0;
        /* mark host BVH as non-empty for refit path */
        last_bvh.num_objects = n;
        last_bvh.nodes.resize(1);   // just so .empty() is false
    } else {
        /* refit only */
        stage_times.morton_ms = 0;
        stage_times.sort_ms   = 0;
        CUDA_CHECK(cudaEventRecord(ev0));
        if (n > 1) {
            CUDA_CHECK(cudaMemset(g_ctx.d_flags, 0, (n - 1) * sizeof(int)));
            refit_kernel<<<grid, BLOCK_SIZE>>>(
                g_ctx.d_nodes, g_ctx.d_aabb_lo, g_ctx.d_aabb_hi,
                g_ctx.d_flags, n);
        }
        CUDA_CHECK(cudaEventRecord(ev1)); CUDA_CHECK(cudaEventSynchronize(ev1));
        CUDA_CHECK(cudaEventElapsedTime(&ms, ev0, ev1));
        stage_times.build_ms = ms;
    }
    frames_since_rebuild++;

    /* --- 5. BVH traversal (broad phase) --- */
    CUDA_CHECK(cudaEventRecord(ev0));
    CUDA_CHECK(cudaMemset(g_ctx.d_broad_count, 0, sizeof(int)));
    CUDA_CHECK(cudaMemset(g_ctx.d_overflow, 0, sizeof(int)));
    traverse_kernel<<<grid, BLOCK_SIZE>>>(
        g_ctx.d_nodes, n,
        g_ctx.d_broad_pairs, g_ctx.d_broad_count,
        g_ctx.max_broad, g_ctx.d_overflow);
    CUDA_CHECK(cudaEventRecord(ev1)); CUDA_CHECK(cudaEventSynchronize(ev1));
    CUDA_CHECK(cudaEventElapsedTime(&ms, ev0, ev1));
    stage_times.traverse_ms = ms;

    int h_broad_count = 0;
    CUDA_CHECK(cudaMemcpy(&h_broad_count, g_ctx.d_broad_count, sizeof(int), cudaMemcpyDeviceToHost));
    if (h_broad_count > g_ctx.max_broad) h_broad_count = g_ctx.max_broad;

    /* --- 6. Narrow phase: GJK --- */
    CUDA_CHECK(cudaEventRecord(ev0));
    CUDA_CHECK(cudaMemset(g_ctx.d_narrow_count, 0, sizeof(int)));
    if (h_broad_count > 0) {
        int gjk_grid = (h_broad_count + GJK_BLOCK_SIZE - 1) / GJK_BLOCK_SIZE;
        gjk_kernel<<<gjk_grid, GJK_BLOCK_SIZE>>>(
            g_ctx.d_broad_pairs, h_broad_count,
            g_ctx.d_vertices, g_ctx.d_vert_counts, g_ctx.d_positions,
            g_ctx.d_narrow_pairs, g_ctx.d_narrow_count, g_ctx.max_narrow);
    }
    CUDA_CHECK(cudaEventRecord(ev1)); CUDA_CHECK(cudaEventSynchronize(ev1));
    CUDA_CHECK(cudaEventElapsedTime(&ms, ev0, ev1));
    stage_times.gjk_ms = ms;

    int h_narrow_count = 0;
    CUDA_CHECK(cudaMemcpy(&h_narrow_count, g_ctx.d_narrow_count, sizeof(int), cudaMemcpyDeviceToHost));
    if (h_narrow_count > g_ctx.max_narrow) h_narrow_count = g_ctx.max_narrow;

    /* download confirmed pairs into pre-allocated pinned buffer (faster PCIe transfer) */
    if (h_narrow_count > 0)
        CUDA_CHECK(cudaMemcpy(g_ctx.h_narrow_pinned, g_ctx.d_narrow_pairs,
                              h_narrow_count * sizeof(int2), cudaMemcpyDeviceToHost));

    CUDA_CHECK(cudaEventDestroy(ev0));
    CUDA_CHECK(cudaEventDestroy(ev1));
    return Span<CollisionPair>(
        reinterpret_cast<CollisionPair*>(g_ctx.h_narrow_pinned), h_narrow_count);
}
