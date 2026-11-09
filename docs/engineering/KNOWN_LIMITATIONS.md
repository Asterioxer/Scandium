# Scandium Known Limitations & Constraints

This document catalogs known architectural limitations, unsupported configurations, and open performance engineering opportunities.

---

## 1. Graphics & Compute Backends
- **HLSL Compute Pipeline Status:** `BatchSimulationPipeline` currently provides structured buffer memory layouts, a 64-thread wavefront CPU emulator, and HLSL compute shader export. It is not yet connected to a live Vulkan or DirectX 12 hardware driver dispatch. True GPU execution requires linking an optional Vulkan SDK compute backend.

## 2. Navigation & Mega-Scale Grids
- **Dense Grid Scaling:** The current `AStar` and `JumpPointSearch` implementations use dense flat grid buffers (`width * height`). While fast due to contiguous memory, grids larger than 4,096 x 4,096 require excessive RAM (>64 MB per grid layer). Hierarchical Pathfinding (HPA*) is required for open-world environments exceeding 2,048 x 2,048.
- **Winding Corridors JPS Penalty:** On narrow 1-tile winding corridors, JPS performs slower than A* (2.01 ms vs 0.77 ms) because perpendicular lookaheads repeatedly hit dead-end wall cells. The engine must adaptively use A* when map density metrics indicate narrow corridors.

## 3. Concurrency & High Core Counts
- **Centralized Work Queue:** Although spin-wait contention has been eliminated, task enqueueing still synchronizes on a centralized `std::mutex`. On architectures with >32 logical cores or multi-socket NUMA nodes, a lockless work-stealing deque (Chase-Lev) will be necessary to eliminate enqueue latency.

## 4. Spatial Clustering Extremes
- **Adversarial Densities:** When all 25,000 entities are concentrated into a single spatial cell, candidate testing degrades to $O(N^2)$ in that bucket (producing 25,000 candidates per query). Dynamic bucket resizing and sub-cell octree refinement are planned to maintain throughput.
