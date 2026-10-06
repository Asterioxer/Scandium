# Changelog

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
