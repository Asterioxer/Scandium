# Scandium

A high-performance C++ game-systems laboratory.

Scandium is an engineering-focused portfolio project built around 3D mathematics, navigation, collision detection, simulation, profiling, and live-service infrastructure.

## Current capabilities

- C++20 + CMake project foundation
- Warning-clean core target configuration
- Generic Vec3<T> with dot/cross products and normalization
- Column-major Mat4
- Translation and scale transforms
- Unit tests + GitHub Actions CI

## Roadmap

### Core mathematics
- [x] Vec3
- [x] Mat4
- [x] Transform composition
- [x] Quaternion rotation
- [x] View/projection matrices
- [x] Ray primitives

### Simulation
- [x] AABB collision
- [x] Spatial hash
- [x] Object pooling
- [x] Entity-component system

### AI
- [x] A*
- [x] Jump Point Search interface
- [x] Steering behaviors
- [x] Multi-agent simulation

### Performance
- [x] Reproducible benchmark harness
- [x] Frame-time instrumentation
- [x] Allocation accounting
- [x] Before/after optimization reports

### Live service
- [x] Player API
- [x] Match prototype
- [x] Leaderboard
- [ ] PostgreSQL persistence

### Agentic engineering
- [x] Repository-aware coding workflow
- [x] Automated test/build verification
- [x] Benchmark-driven change validation
- [x] Human approval gate

## Engineering principles

1. Measure before optimizing.
2. Keep hot-path code allocation-aware.
3. Make complexity explicit.
4. Test systems independently.
5. Use agents to accelerate engineering, not replace verification.

See docs/architecture.md for the system direction.
