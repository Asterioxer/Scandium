# Changelog

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
