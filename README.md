# Scandium

A high-performance C++ game-systems laboratory evolving into a flagship real-time simulation platform.

> **Portfolio thesis:** build the systems underneath a large-scale game instead of building another small game demo.

Scandium is an engineering-focused C++ game-systems laboratory built around deterministic simulation, 3D mathematics, navigation, collision detection, profiling, and live-service infrastructure.

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
- [x] Jump Point Search (4-connected)
- [x] Steering behaviors
- [x] Multi-agent simulation
- [x] Deterministic fixed-step simulation clock
- [x] Multithreaded job system
- [x] Deterministic replay/input stream
- [x] Simulation state hashing
- [x] Renderer-facing interpolation snapshots
- [x] Event-driven runtime
- [x] Multithreaded job system
- [x] Deterministic replay/input stream
- [x] Simulation state hashing
- [x] Renderer-facing interpolation snapshots
- [x] Event-driven runtime

### Performance
- [x] Reproducible benchmark harness
- [x] Frame-time instrumentation
- [x] Allocation accounting
- [x] Before/after optimization reports
- [x] A* versus JPS4 navigation benchmark

### Live service
- [x] Player API
- [x] Match prototype
- [x] Reliable packet acknowledgement window
- [x] Reliable packet acknowledgement window
- [x] Leaderboard
- [x] PostgreSQL persistence (optional runtime backend)

### Agentic engineering
- [x] Repository-aware coding workflow
- [x] Automated test/build verification
- [x] Benchmark-driven change validation
- [x] Chrome trace export
- [x] 10,000-agent scale benchmark
- [x] Chrome trace export
- [x] 10,000-agent scale benchmark
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
    build\\Release\\scandium_ascii_demo.exe

### ASCII simulation demo

`scandium_ascii_demo` renders the fixed-step multi-agent simulation in a terminal using interpolated positions. It has no graphics dependency, so it remains portable across CI and local development.

### Live service with PostgreSQL

The API runs without a database for lightweight tests, or against PostgreSQL when `SCANDIUM_DATABASE_URL` is configured. To launch the full local service stack:

    docker compose -f services/game_api/docker-compose.yml up --build

The service exposes `/health`, player management, scores, leaderboards, and matches.

See docs/architecture.md for the system direction.

## Flagship architecture

Scandium is deliberately structured as engine technology rather than a single game:

```
                    Game / Demo Layer
                           |
                    Render Snapshot
                           |
                 +---------v---------+
                 |  Runtime Engine  |
                 +----+----+----+---+
                      |    |    |
                    ECS  Jobs  Events
                      |    |    |
                 Simulation / AI
                      |
             Physics / Navigation
                      |
                    Math
                      |
             Profiling / Replay
                      |
                 Live Services
```

### Performance philosophy

The engine separates simulation correctness from rendering. Fixed-step updates provide a stable simulation clock; interpolation produces render-ready state between ticks; job execution allows independent systems to scale across CPU cores; state hashes make deterministic regressions detectable.

### Flagship demo

The default executable reports worker count, simulation ticks, agent scale, AI state distribution, spatial-query results, interpolation state, deterministic state hash, replay command count, and profiling events.

Additional executables provide ASCII visualization and reproducible collision, navigation, and scale benchmarks.

## Engineering standard

A system is not considered complete merely because it compiles. Flagship features should have:

1. a public API with explicit invariants,
2. automated correctness tests,
3. a deterministic or reproducible benchmark where performance matters,
4. documentation describing trade-offs,
5. a demo path showing why the system exists.

Modern data-oriented game technology similarly emphasizes simulation scale, determinism, data layout, and multicore performance. citeturn0search0turn0search2


## Flagship architecture

Scandium is deliberately structured as engine technology rather than a single game.

### Performance philosophy

The engine separates simulation correctness from rendering. Fixed-step updates provide a stable simulation clock; interpolation produces render-ready state between ticks; job execution allows independent systems to scale across CPU cores; state hashes make deterministic regressions detectable.

### Flagship demo

The default executable reports worker count, simulation ticks, agent scale, AI state distribution, spatial-query results, interpolation state, deterministic state hash, replay command count, and profiling events.

Additional executables provide ASCII visualization and reproducible collision, navigation, and scale benchmarks.

## Engineering standard

A system is not considered complete merely because it compiles. Flagship features should have:

1. a public API with explicit invariants,
2. automated correctness tests,
3. a deterministic or reproducible benchmark where performance matters,
4. documentation describing trade-offs,
5. a demo path showing why the system exists.
