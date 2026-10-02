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
- [ ] Quaternion rotation
- [x] View/projection matrices
- [ ] Ray primitives

### Simulation
- [ ] AABB collision
- [ ] Spatial hash
- [ ] Object pooling
- [ ] Entity-component system

### AI
- [ ] A*
- [ ] Jump Point Search
- [ ] Steering behaviors
- [ ] Multi-agent simulation

### Performance
- [ ] Reproducible benchmark harness
- [ ] Frame-time instrumentation
- [ ] Allocation tracking
- [ ] Before/after optimization reports

### Live service
- [ ] Player API
- [ ] Matchmaking prototype
- [ ] Leaderboard
- [ ] PostgreSQL persistence

### Agentic engineering
- [ ] Repository-aware coding workflow
- [ ] Automated test/build verification
- [ ] Benchmark-driven change validation
- [ ] Human approval gate

## Engineering principles

1. Measure before optimizing.
2. Keep hot-path code allocation-aware.
3. Make complexity explicit.
4. Test systems independently.
5. Use agents to accelerate engineering, not replace verification.

See docs/architecture.md for the system direction.
