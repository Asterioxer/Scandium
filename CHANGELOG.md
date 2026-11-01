# Changelog

## 0.6.0 — High-Performance Advancements: Linear BVH, 8-Way Navigation, SIMD Steering & Rollback Netcode

- **Linear Bounding Volume Hierarchy (LBVH):** Implemented `include/scandium/spatial/linear_bvh.hpp` using 63-bit Morton code spatial quantization (21 bits/axis across a $2,097,152^3$ coordinate space). Provides contiguous flat vector node storage, stack-based non-recursive $O(\log N)$ spatial bounding queries, and front-to-back raycasting (achieving 95,253 rays/sec and 1.75x speedup over naive raycasting with 0.7 ms tree build time).
- **8-Connected Diagonal Navigation with Octile Heuristic:** Added `GridConnectivity::EightWay` to `AStar`, supporting diagonal movement (NE, NW, SE, SW) with cost $\sqrt{2} pprox 1.414$ and admissible Octile distance heuristic $h = (\sqrt{2} - 1)\min(\Delta x, \Delta y) + \max(\Delta x, \Delta y)$. Enforces corner-cutting prevention rules. Reduces path costs by 29.3% and speeds up open-grid search by 13.0x (71 nodes explored vs 1,694).
- **SIMD Vectorized Steering Kernels (SoA Layout):** Added `include/scandium/ai/simd_steering.hpp` with `AgentSoA` Structure-of-Arrays layout and 4-wide SSE hardware acceleration (`_mm_loadu_ps`, branchless masked distance evaluation, horizontal reduction). Delivers 2.35x measured throughput increase (65.0 ms vs 152.9 ms) over scalar AoS separation.
- **Deterministic Client-Side Prediction & Rollback Netcode:** Added `include/scandium/network/rollback.hpp` featuring `StateRingBuffer<Capacity>` with $O(1)$ bitwise modular indexing and `RollbackController`. Supports zero-latency local input prediction, remote input speculation, authoritative divergence detection, and bit-exact state rewind and fast-forward re-simulation.
- **High-Throughput Batch / Compute Pipeline:** Added `include/scandium/simulation/compute_pipeline.hpp` with 16-byte aligned StructuredBuffer layouts (`GPUAgent`, `GPUSimulationParams`) matching DirectX 12 / Vulkan / WebGPU compute standards. Includes multi-threaded CPU wavefront emulator stepping 10,000 agents in lockstep and complete HLSL compute shader export.
- **Comprehensive Advancements Benchmark:** Added `benchmarks/advancements_benchmark.cpp` measuring LBVH raycasting speedup, 8-way diagonal path cost reduction, SIMD steering speedup, and batch pipeline throughput.

## 0.5.1 — Performance Engineering & Concurrency Hardening

- **Spatial Hash Optimization:** Implemented 16-byte cache-line aligned `Entry` packing with 64-bit `cell_key` coordinate bit-packing, reducing table memory by 13.5%. Integrated 64-bit SplitMix64 finalizer dispersion, cutting hash collisions by 38.8% and speeding up dense query traversals by 22-27%. Added `SpatialHashDiagnostics`.
- **Job System Lock-Free Synchronization:** Replaced stack-allocated mutexes and condition variables with lock-free atomic token synchronization (`std::memory_order_release` / `acquire`). Added adaptive microsecond spin-polling with `_mm_pause()`, eliminating the 8-worker regression and unlocking up to 3.25x scaling on asymmetric Intel Alder Lake architectures. Fixed thundering herd contention via targeted worker waking.
- **Comprehensive Navigation Benchmark Suite:** Benchmarked A* vs. JPS across 5 map topologies (Open Grid, Corridors, Dense Labyrinth, Random Obstacles 20%, Sparse Obstacles 5%), empirically proving JPS is 1.15-1.20x faster than A* in corridors and labyrinths, while A* is 1.31x faster in open space.
- **Statistical Benchmark Harness:** Added reusable `include/scandium/profiling/benchmark_harness.hpp` supporting monotonic timing, warmup isolation, optimizer barriers (`do_not_optimize`), and full statistical estimators (min, max, mean, stddev, p50, p90, p99, CSV/JSON export).
- **Regression Testing:** Added automated regression test suites for spatial hash broadphase equivalence against naive search, job system rapid-fire concurrency stress across 1..16 workers, and bit-exact multi-agent simulation parity.

## 0.5.0 — Renderer Flagship

- Added dependency-free software renderer.
- Added top-down arena frame export with team colors and health bars.
- Added rendered arena showcase executable.

## 0.4.0 — Flagship Gameplay and Multiplayer

- Added deterministic Scandium Arena combat simulation.
- Added team targeting, movement, attacks, damage, eliminations, and scoring.
- Added reliable acknowledgement-window networking primitive.
- Added deterministic tick-based lockstep input session.
- Added automated arena and networking tests.

## 0.3.0 — Flagship Runtime

- Added multithreaded job system and parallel agent simulation.
- Added runtime event bus and renderer-facing snapshots.
- Added deterministic replay commands and simulation state hashing.
- Added reliable acknowledgement-window networking primitive.
- Added Chrome trace-compatible profiling output.
- Added 10,000-agent simulation scale benchmark.
- Upgraded the default executable into a systems showcase.

## 0.2.0 — Simulation Runtime

- Added deterministic fixed-step simulation clock with bounded catch-up.
- Added simulation statistics and render interpolation state.
- Added dependency-free ASCII multi-agent simulation demo.
- Replaced placeholder JPS delegation with canonical horizontal-first JPS4.
- Added A* versus JPS navigation benchmark.
- Added quaternion normalization and rotation-aware transforms.
- Made object-pool release lookup constant-time on average.
- Added optional PostgreSQL persistence for players, scores, matches, and leaderboards.
- Added Docker Compose for the live-service stack.
- Expanded CI to test both memory and PostgreSQL API storage modes.
