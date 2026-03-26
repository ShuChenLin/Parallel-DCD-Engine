export default function Proposal() {
  return (
    <div className="max-w-3xl mx-auto px-6 py-10">

      <h1>Parallel Dynamic Discrete Collision Detection Engine</h1>
      <p style={{ color: '#666', marginTop: '.3rem', marginBottom: '0' }}>
        Bill &amp; Shu Chen &nbsp;·&nbsp; <a href="https://github.com/ShuChenLin/Parallel-DCD-Engine">GitHub</a>
      </p>

      <hr />

      <h2>Summary</h2>
      <p>
        We propose to build a high-performance dynamic collision detection engine capable of processing
        large numbers of moving 3D bodies in real time. We implement discrete collision detection (DCD),
        where collision status is evaluated per frame rather than continuously. Objects reflect off
        boundaries by negating their velocity. Rotational motion and elastic collision response between
        objects are intentionally out of scope, allowing us to focus on the core parallel challenges.
      </p>
      <p>
        The engine consists of two phases: <strong>Broad Phase</strong> (constructing a Bounding Volume
        Hierarchy to rapidly eliminate non-colliding pairs) and <strong>Narrow Phase</strong> (exact
        intersection tests on surviving candidate pairs). We will implement the full pipeline on both
        multicore CPU (OpenMP) and GPU (CUDA), and investigate an adaptive rebuild/refit policy as our
        core research contribution. Results will be visualized in real time using OpenGL.
      </p>

      <hr />

      <h2>Background</h2>
      <p>
        Collision detection is a fundamental bottleneck in physics simulation, robotics, and game engines.
        The standard approach splits into two phases:
      </p>
      <ul>
        <li>
          <strong>Broad Phase:</strong> A Bounding Volume Hierarchy (BVH) is used to quickly eliminate
          pairs of objects that cannot possibly intersect. Each object is enclosed in an Axis-Aligned
          Bounding Box (AABB); the hierarchy lets us prune large portions of the search space in
          O(log N) time.
        </li>
        <li>
          <strong>Narrow Phase:</strong> For each surviving candidate pair, the
          Gilbert-Johnson-Keerthi (GJK) algorithm computes the exact minimum distance between two
          convex shapes, determining whether a true collision has occurred.
        </li>
      </ul>
      <p>
        In a dynamic scene, objects move every frame, which means the BVH cannot be built once and
        reused. There are two update strategies. <strong>Full rebuild</strong> discards the old tree
        and constructs a new one from scratch each frame using the Karras (2012) Linear BVH (LBVH)
        algorithm, which maps object positions to Morton codes, sorts them via radix sort, and constructs
        a binary radix tree in parallel. <strong>Incremental refit</strong> keeps the existing tree
        topology and only updates bounding boxes bottom-up, which is faster but allows tree quality to
        degrade as objects move far from their original positions.
      </p>
      <p>
        Neither strategy dominates in all scenarios. This tradeoff between build cost and tree quality,
        and its downstream effect on traversal cost, is the central empirical question we want to answer.
      </p>

      <hr />

      <h2>The Challenge</h2>
      <p>
        By stripping out the physics response, we're left with a massive, pure geometry problem. We're
        implementing this sequentially, then with OpenMP, and finally in CUDA. The overarching headache
        is that what runs fast on a CPU usually tanks on a GPU. Our main bottlenecks are:
      </p>

      <h3>Broad Phase</h3>
      <ul>
        <li>
          <strong>Building Trees on a GPU:</strong> Traditional top-down BVH building is awful for
          GPUs. We're using the Karras LBVH algorithm, which cleverly turns tree-building into a
          massive parallel radix sort. But that means we are going to absolutely hammer our global
          memory bandwidth. On the OpenMP side, our main fight will be cache coherence during the
          bottom-up tree aggregation.
        </li>
        <li>
          <strong>The Rebuild vs. Refit Dilemma:</strong> Objects move and trees degrade. We need a
          super cheap heuristic on the fly, like measuring how much the bounding boxes have bloated
          to decide: do we do a quick refit, or bite the bullet and rebuild the whole tree? If
          computing that metric is too slow, the whole optimization is pointless.
        </li>
        <li>
          <strong>Load Imbalance:</strong> When two dense clusters of objects smash into each other,
          the BVH traversal spits out thousands of collision pairs in one spot, while the rest of the
          scene is empty. If we just use a basic static schedule, a few threads will do all the work
          while the rest of the processor just chills. We absolutely need to build a low-overhead
          dynamic work queue to steal and redistribute tasks.
        </li>
      </ul>

      <h3>Narrow Phase</h3>
      <ul>
        <li>
          <strong>GJK vs. Warps (Warp Divergence):</strong> Our Narrow Phase uses GJK, which is an
          unpredictable while loop. CPU branch predictors eat this up. On a GPU? Total nightmare. If
          one thread in a warp takes 20 iterations to converge and the other 31 finish in 3, they all
          just sit there stalled out. It's going to wreck our SIMT efficiency.
        </li>
        <li>
          <strong>Thrashing the Cache:</strong> Inside GJK, the Support function grabs vertices in
          completely random directions. If we keep our mesh data as a standard Array of Structures
          (AoS), our GPU memory accesses will be totally uncoalesced, and we'll hit the memory wall
          instantly. We have to pivot to a Structure of Arrays (SoA) layout.
        </li>
      </ul>

      <p>
        Through this project, we hope to deep dive into the hard parallel problems that come with
        dynamic geometry. Beyond correctness, we want to understand what actually bottlenecks
        performance in practice, and what's different when paralleling on CPU versus GPU.
      </p>

      <hr />

      <h2>Resources</h2>
      <p>
        <strong>Hardware &amp; Profiling:</strong> We will develop on GHC machines (multi-core CPU +
        RTX 2080) for our OpenGL visualization and baseline tests, then scale to PSC compute nodes for
        massive stress testing using <code>perf</code>, <code>nsys</code>, and <code>ncu</code>. We do
        not require any additional special machines.
      </p>
      <p>
        <strong>Code Base:</strong> We are building the core collision engine (dynamic BVH, GJK narrow
        phase, work queues) <strong>entirely from scratch in C++</strong>. We will use standard
        libraries (OpenGL/GLFW) for rendering and NVIDIA CUB/Thrust solely for the highly optimized
        parallel radix sort in our LBVH construction.
      </p>
      <p><strong>Key References:</strong></p>
      <ol>
        <li>
          Karras, T. (2012).{' '}
          <a href="https://diglib.eg.org/items/71edb707-2be1-4d84-8e25-9162cd9a59e9">
            Maximizing Parallelism in the Construction of BVHs, Octrees, and k-d Trees.
          </a>{' '}
          <em>Proc. High-Performance Graphics (HPG 2012)</em>, pp. 33–37. Eurographics Association.
        </li>
        <li>
          Lindemann, P. (2010).{' '}
          <a href="https://www.medien.ifi.lmu.de/lehre/ss10/ps/Ausarbeitung_Beispiel.pdf">
            The Gilbert-Johnson-Keerthi Distance Algorithm.
          </a>{' '}
          LMU Munich.
        </li>
        <li>
          Yazdani, A., &amp; Wachs, A. (2025).{' '}
          <a href="https://www.sciencedirect.com/science/article/pii/S001046552500270X">
            Performance optimization of GJK collision detection in discrete element simulations.
          </a>{' '}
          <em>Computer Physics Communications</em>, 316, 109768.
        </li>
      </ol>

      <hr />

      <h2>Goals &amp; Deliverables</h2>
      <ul>
        <li>Sequential Baseline: Correct LBVH build, BVH traversal, and GJK narrow phase.</li>
        <li>CPU parallel: OpenMP parallelization of three stages with work queue for traversal.</li>
        <li>
          GPU parallel: CUDA implementation of LBVH build (Morton sort, Karras tree construction,
          atomic refit) and parallel traversal.
        </li>
        <li>
          Adaptive rebuild/refit policy: a per-frame heuristic that measures tree quality (e.g.,
          overlap ratio or SAH cost) and switches strategies accordingly.
        </li>
        <li>
          Benchmarks across three dynamic scenarios (random walk, two-cluster collision, avalanche)
          on both CPU and GPU, with per-stage profiling and roofline analysis.
        </li>
        <li>
          Demo video: OpenGL real-time visualization showing moving objects, collision highlights,
          and BVH wireframes updating each frame.
        </li>
      </ul>

      <hr />

      <h2>Platform Choice</h2>
      <p>
        We chose C++ for the low-level memory control needed. We're using CUDA and OpenMP because
        their contrasting architectures perfectly highlight our bottlenecks. CUDA's massive parallelism
        dominates the broad phase, but its SIMT model will choke on GJK's divergent while loops.
        Conversely, OpenMP leverages CPU branch prediction to handle those loops effortlessly, giving
        us a great architectural comparison.
      </p>
      <p>
        We'll use GHC machines (CPU + RTX 2080) for development and OpenGL visualization, scaling to
        PSC nodes for extreme stress-testing. To rigorously analyze these architectural tradeoffs, we
        will rely on <code>perf</code> for CPU metrics, plus <code>nsys</code> and <code>ncu</code> to
        accurately diagnose GPU warp divergence and cache thrashing.
      </p>

      <hr />

      <h2>Schedule</h2>
      <table>
        <thead>
          <tr>
            <th style={{ width: '2.5rem' }}>Wk</th>
            <th style={{ width: '7rem' }}>Dates</th>
            <th>Goals</th>
            <th>Deliverables</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>1</td>
            <td>Mar 26 – Apr 1</td>
            <td>Sequential baseline: BVH build (LBVH), broad phase traversal, GJK narrow phase. Simple dynamic scene simulator (analytic motion).</td>
            <td>Correct sequential pipeline; ground truth for all later correctness checks.</td>
          </tr>
          <tr>
            <td>2</td>
            <td>Apr 2 – Apr 8</td>
            <td>CPU parallel: OpenMP LBVH rebuild, parallel traversal with work queue, parallel GJK. Initial refit implementation.</td>
            <td>CPU parallel pipeline running; first speedup numbers on GHC machines.</td>
          </tr>
          <tr>
            <td>3</td>
            <td>Apr 9 – Apr 15</td>
            <td>GPU parallel: CUDA LBVH rebuild (Morton sort + Karras tree construction + atomic bottom-up refit). BVH traversal + GJK CUDA kernel. <strong>Milestone.</strong></td>
            <td>GPU pipeline running on GHC RTX 2080; per-stage profiling with CUDA events.</td>
          </tr>
          <tr>
            <td>4</td>
            <td>Apr 16 – Apr 22</td>
            <td>Adaptive rebuild/refit policy: design quality metric (SAH cost or overlap ratio), implement switching heuristic. OpenGL visualization.</td>
            <td>Adaptive policy working; real-time OpenGL demo with BVH wireframes and collision highlighting.</td>
          </tr>
          <tr>
            <td>5</td>
            <td>Apr 23 – May 1</td>
            <td>Benchmarking all three scenarios (random walk, two clusters, avalanche) on CPU and GPU. Roofline analysis. Final report and poster.</td>
            <td>Final report, poster, and public GitHub with reproducible benchmarks.</td>
          </tr>
        </tbody>
      </table>

    </div>
  )
}
