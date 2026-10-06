# Scandium

A high-performance C++ game-systems laboratory.

Scandium is an engineering-focused portfolio project built around 3D mathematics, navigation, collision detection, simulation, profiling, and live-service infrastructure.

## Current capabilities

- C++20 + CMake project foundation
- Deterministic simulation runtime with fixed-step ticking and catch-up limits
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
- [x] Deterministic fixed-step simulation clock

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

## Runtime demo

The default executable now runs a deterministic multi-agent simulation through a fixed-step clock. It reports simulation steps, agent states, spatial-hash proximity, and interpolation state before the math smoke test.

Build and run:

    cmake -S . -B build -DSCANDIUM_BUILD_TESTS=ON
    cmake --build build --config Release
    ctest --test-dir build -C Release --output-on-failure
    build\\Release\\scandium_demo.exe

See docs/architecture.md for the system direction.
