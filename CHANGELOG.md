# Changelog

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
