export default function Milestone() {
  return (
    <div className="max-w-3xl mx-auto px-6 py-10">

      <h1>Parallel Dynamic Discrete Collision Detection Engine — Milestone</h1>
      <p style={{ color: '#666', marginTop: '.3rem', marginBottom: '0' }}>
        Bill &amp; Shu Chen &nbsp;·&nbsp; <a href="https://github.com/ShuChenLin/Parallel-DCD-Engine">GitHub</a>
      </p>

      <hr />

      <h2>Revised Schedule</h2>
      <table>
        <thead>
          <tr>
            <th style={{ width: '2.5rem' }}>#</th>
            <th style={{ width: '8rem' }}>Dates</th>
            <th>Deliverables</th>
            <th style={{ width: '7rem' }}>Person</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td rowSpan={3}>1</td>
            <td rowSpan={3}>Apr 15 – Apr 18</td>
            <td>Try different methods to overcome the scalability cap with a high number of cores.</td>
            <td>Shu Chen</td>
          </tr>
          <tr>
            <td>OMP BVH build scalability (PSC 128-core): profile and fix current OpenMP codebase.</td>
            <td>Bill</td>
          </tr>
          <tr>
            <td>CUDA GJK regression at large N: diagnose degrades at N=500k (max_broad overflow in two_cluster).</td>
            <td>Bill</td>
          </tr>
          <tr>
            <td rowSpan={4}>2</td>
            <td rowSpan={4}>Apr 19 – Apr 22</td>
            <td>CUDA BVH build kernel: Karras internal-node construction.</td>
            <td>Shu Chen</td>
          </tr>
          <tr>
            <td>CUDA BVH traversal kernel: per-leaf queries (N-leaf style) mapped to CUDA threads.</td>
            <td>Shu Chen</td>
          </tr>
          <tr>
            <td>CUDA GJK optimization: GJK narrow phase.</td>
            <td>Bill</td>
          </tr>
          <tr>
            <td>OpenGL visualizer — dynamic BVH AABB overlay: render BVH node bounding boxes per frame.</td>
            <td>Bill</td>
          </tr>
          <tr>
            <td>3</td>
            <td>Apr 23 – Apr 25</td>
            <td>Benchmark all three scenarios (random walk, two clusters, avalanche) for seq / OMP / CUDA. Plot speedup and analysis graphs.</td>
            <td>Shu Chen &amp; Bill</td>
          </tr>
          <tr>
            <td>4</td>
            <td>Apr 26 – Apr 28</td>
            <td>Draft final report: introduction, related work, design decisions, results tables + figures, analysis.</td>
            <td>Shu Chen &amp; Bill</td>
          </tr>
          <tr>
            <td>5</td>
            <td>Apr 29 – May 1</td>
            <td>Finalize &amp; submit poster + report.</td>
            <td>Shu Chen &amp; Bill</td>
          </tr>
        </tbody>
      </table>

      <hr />

      <h2>Work Completed</h2>

      <h3>Sequential</h3>
      <p>
        Complete baseline implementing the full DCD pipeline: AABB update → Morton code computation →
        radix sort → LBVH build (Karras 2012) → dual BVH traversal → GJK narrow phase. Correctness
        validated against O(N²) brute-force on all 3 scenarios.
      </p>
      <p><strong>Optimizations:</strong></p>
      <ul>
        <li><strong>Dual traversal:</strong> simultaneous two-subtree descent reduces AABB tests from O(N log N) to O(N + K log N).</li>
        <li><strong>Periodic rebuild:</strong> refit-only for 4 of every 5 frames.</li>
      </ul>
      <p>
        However, these sequential optimizations decrease absolute time but make it harder to scale in
        further experiments (see the results below).
      </p>

      <h3>OpenMP</h3>
      <ul>
        <li><strong>AABB / Morton / GJK:</strong> <code>parallel for schedule(static)</code> — embarrassingly parallel.</li>
        <li><strong>Karras build:</strong> all n−1 internal nodes resolved independently via <code>parallel for</code>, no locks.</li>
        <li><strong>Refit:</strong> atomic flag per node; second arrival merges children AABBs. Wait-free, O(log N) critical path.</li>
        <li><strong>Dual traversal:</strong> serial BFS seeds task pool (target: threads × 16), then <code>schedule(dynamic)</code> parallel consumption into per-thread buffers.</li>
      </ul>

      <h3>CUDA (GPU Parallelization)</h3>
      <ul>
        <li><strong>AABB / Morton / GJK:</strong> one thread per body/pair — embarrassingly parallel kernels.</li>
        <li><strong>Radix sort:</strong> <code>thrust::sort_by_key</code> on (code, index) pairs.</li>
        <li><strong>Karras build:</strong> one thread per internal node runs <code>determine_range</code> + <code>find_split</code> — ideal GPU workload, all n−1 nodes independent, zero atomics.</li>
        <li><strong>Refit:</strong> one thread per leaf walks to root with <code>atomicAdd</code> flag; second arrival merges. Same algorithm as OpenMP but with thousands of threads in flight.</li>
        <li><strong>Traversal:</strong> one thread per leaf, per-thread register stack, emits pairs via <code>atomicAdd</code> into global buffer.</li>
        <li><strong>Periodic rebuild (interval = 5):</strong> identical to sequential / OpenMP baseline.</li>
      </ul>

      <h3>Visualization</h3>
      <p>
        An interactive OpenGL visualizer (<code>dcd_viz</code>) renders the simulation in real time:
        Phong-shaded convex bodies (tetrahedra, octahedra, icospheres) with colliding pairs highlighted
        in red, orbit camera (mouse drag/scroll), pause/resume and scenario switching (Space, 1/2/3),
        and a live HUD showing collision count and detection time.
      </p>

      <hr />

      <h2>Things to Show in Poster Session</h2>
      <ul>
        <li>Speedup curves: OpenMP (T=1 → 128) and CUDA vs. sequential.</li>
        <li>Per-stage breakdown charts across all 3 scenarios.</li>
        <li>Live OpenGL visualizer demo (with BVH boxes).</li>
        <li>Graph: Sequential vs OpenMP vs CUDA </li>
      </ul>

      <hr />

      <h2>Issues</h2>

      <h3>OpenMP</h3>
      <ul>
        <li><strong>Sort regression:</strong> serial 256-bucket prefix-sum per radix pass is an Amdahl bottleneck; sort regresses past T=32 despite parallelized histogram reduction.</li>
        <li><strong>Refit O(log N) critical path:</strong> atomic bottom-up walk caps build at ~2× speedup; always-rebuild doesn't help since <code>bvh_build_omp</code> still calls <code>bvh_refit_omp</code> internally.</li>
        <li><strong>Dual traversal:</strong> serial BFS seeding bottlenecks at high T; N-leaf queries win at T ≥ 32 despite more total AABB tests.</li>
      </ul>

      <h3>CUDA</h3>
      <ul>
        <li><strong>Broad-phase overflow:</strong> two_cluster N=500k exceeds the fixed N×32 buffer — silent truncation causes incorrect results and GJK throughput drops from 47× to 26×.</li>
        <li><strong>Unconfirmed GPU bottlenecks:</strong> suspected warp divergence in traversal and register spill in GJK under high pair counts — full profiling with Nsight / nvprof pending.</li>
      </ul>

      <hr />

      <h2>Progress and Deliverables</h2>
      <p>
        We think we will be able to produce all our deliverables. All three core implementations —
        sequential baseline, OpenMP, and CUDA — are complete and benchmarked. The OpenGL visualizer is
        almost finished. Both OpenMP and CUDA still have optimization headroom, and GPU profiling with
        <code>nsys</code> / <code>ncu</code> is planned for the remaining two weeks. We believe all
        deliverables will be completed in time for the poster session.
      </p>

      <hr />

      <h2>Preliminary Results</h2>

      <h3>OpenMP Scaling (GHC, N=50k)</h3>
      <p><em>Dual traversal, rebuild-every-5, T=1 → 8</em></p>
      <table>
        <thead>
          <tr><th>Scenario</th><th>T=1</th><th>T=8</th><th>Speedup</th></tr>
        </thead>
        <tbody>
          <tr><td>random_walk</td><td>11.82 ms</td><td>2.07 ms</td><td>5.72×</td></tr>
          <tr><td>two_cluster</td><td>54.15 ms</td><td>9.35 ms</td><td>5.79×</td></tr>
          <tr><td>avalanche</td><td>13.65 ms</td><td>2.46 ms</td><td>5.56×</td></tr>
        </tbody>
      </table>

      <p><em>N-leaf traversal, rebuild-every-5, T=1 → 8</em></p>
      <table>
        <thead>
          <tr><th>Scenario</th><th>T=1</th><th>T=8</th><th>Speedup</th></tr>
        </thead>
        <tbody>
          <tr><td>random_walk</td><td>20.75 ms</td><td>3.02 ms</td><td>6.88×</td></tr>
          <tr><td>two_cluster</td><td>66.73 ms</td><td>10.45 ms</td><td>6.39×</td></tr>
          <tr><td>avalanche</td><td>21.93 ms</td><td>3.32 ms</td><td>6.61×</td></tr>
        </tbody>
      </table>
      <p>
        N-leaf traversal is easier to parallelize but has slower sequential time. Dual traversal has
        faster sequential time, but this amplifies the cost of the unparallelized stages, yielding a
        lower apparent speedup.
      </p>

      <h3>Result on PSC (N=500,000)</h3>
      <img src="/Parallel-DCD-Engine/n500k_1_speedup.png" alt="Strong-scaling speedup" />
      <img src="/Parallel-DCD-Engine/n500k_3_stage_speedup_tc.png" alt="Per-stage speedup (two_cluster)" />
      <p>
        Best speedup occurs in the two_cluster scene since GJK dominates the workload. The best-scaling
        stages are GJK and BVH traversal. The collapse at 128 cores happens because of inter-socket
        communication on the dual-socket EPYC node, which causes many remote cache misses. Overcoming
        this memory bottleneck is our next target.
      </p>

      <h3>CUDA v2 (N=50,000)</h3>
      <p>
        CUDA v1 transferred ~17 MB H2D per frame (duplicate uploads). v2 splits into static shape
        upload (once / scenario) + dynamic position upload (once / frame), reducing to ~1.2 MB / frame.
      </p>
      <table>
        <thead>
          <tr><th>Scenario</th><th>Seq T=1</th><th>CUDA v2</th><th>Speedup</th></tr>
        </thead>
        <tbody>
          <tr><td>random_walk</td><td>12.92 ms</td><td>0.615 ms</td><td>21.0×</td></tr>
          <tr><td>two_cluster</td><td>53.32 ms</td><td>2.112 ms</td><td>25.2×</td></tr>
          <tr><td>avalanche</td><td>13.88 ms</td><td>0.701 ms</td><td>19.8×</td></tr>
        </tbody>
      </table>
      <p>
        Per-stage: GJK 47–79×, traversal 25–32×, build 15×, sort 6×.
      </p>

      <h3>CUDA v2 (N=500,000)</h3>
      <table>
        <thead>
          <tr><th>Scenario</th><th>Seq T=1</th><th>CUDA v2</th><th>Speedup</th></tr>
        </thead>
        <tbody>
          <tr><td>random_walk</td><td>541.07 ms</td><td>14.27 ms</td><td>37.9×</td></tr>
          <tr><td>two_cluster</td><td>3882.90 ms</td><td>235.23 ms</td><td>16.5×</td></tr>
          <tr><td>avalanche</td><td>617.32 ms</td><td>20.53 ms</td><td>30.1×</td></tr>
        </tbody>
      </table>
      <p>
        At N=500,000, CUDA v2 delivers 30–38x overall speedup on random_walk and avalanche. 
        The two_cluster scenario is limited to 16.5x. We suspected a buffer overflow: the broad-phase 
        output buffer (max_broad = N×32 = 16M) is exceeded by ~17.5M actual pairs, so we assume it cause 
        silent truncation of ∼1.5M pairs and L2 cache thrash from 16M random GJK accesses. We haven’t 
        profiled it yet and we’ll verify and solve it later.
      </p>

    </div>
  )
}
