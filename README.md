# Parallel-DCD-Engine

**Authors:** Bill Wu, ShuChen Lin
**Course:** CMU 15-418 / 15-618 Spring 2026

A high-performance parallel **Discrete Collision Detection** engine for large 3D scenes,
implemented from scratch in C++ with both **OpenMP** (multi-core CPU) and **CUDA** (GPU)
backends, plus a real-time **OpenGL** visualizer.

- **Website:** https://shuchenlin.github.io/Parallel-DCD-Engine/
- **Final Report:** https://shuchenlin.github.io/Parallel-DCD-Engine/#/final
- **Repository:** https://github.com/ShuChenLin/Parallel-DCD-Engine

## Overview

The engine processes thousands to millions of moving convex rigid bodies per frame using a
two-phase pipeline:

1. **Broad phase** — Linear BVH (Karras 2012) built from Morton-code-sorted AABBs, with
   periodic rebuild + atomic bottom-up refit, traversed via parallel dual traversal.
2. **Narrow phase** — Gilbert-Johnson-Keerthi (GJK) algorithm on surviving candidate pairs.

Three workload scenarios stress different parts of the pipeline:

| Scenario | Description | Stresses |
|---|---|---|
| `random_walk` | Sparse random motion | Build / refit |
| `two_cluster` | Two dense groups colliding head-on | Broad-phase + GJK |
| `avalanche`   | Bodies accumulate into a pile | Mixed |

## Headline Results

**OpenMP** — PSC Bridges-2, AMD EPYC 7742 (up to 128 threads), N = 1,000,000:

| Scenario      | T = 1     | T = 128   | Speedup |
|---------------|----------:|----------:|--------:|
| `random_walk` | 2706 ms   |    66 ms  | **41×** |
| `two_cluster` | 31897 ms  |   521 ms  | **61×** |
| `avalanche`   | 3755 ms   |    91 ms  | **41×** |

**CUDA** — NVIDIA RTX 2080 on GHC, N = 500,000:

| Scenario      | Sequential | CUDA     | Speedup |
|---------------|-----------:|---------:|--------:|
| `random_walk` |  534 ms    |  9.2 ms  | **58×** |
| `two_cluster` | 3838 ms    | 79.6 ms  | **48×** |
| `avalanche`   |  611 ms    |  9.5 ms  | **64×** |

## Building

```bash
cd source
make dcd_seq    # sequential baseline
make dcd_omp    # OpenMP (CPU)
make dcd_cuda   # CUDA (GPU)
make dcd_viz    # OpenGL visualizer
```

## Running

```bash
./dcd_omp  -n 500000 -f 30 -t 64 --scenario two_cluster
./dcd_cuda -n 500000 -f 30      --scenario avalanche
./dcd_viz                       # interactive demo (Space, 1/2/3 to switch scenarios)
```

## References

- Karras, T. (2012). *Maximizing Parallelism in the Construction of BVHs, Octrees, and k-d Trees.* HPG.
- Lindemann, P. (2010). *The Gilbert-Johnson-Keerthi Distance Algorithm.* LMU Munich.
- Yazdani &amp; Wachs (2025). *Performance optimization of GJK collision detection in discrete element simulations.* Comp. Phys. Comm.
