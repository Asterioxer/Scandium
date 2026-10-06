# Scandium — Engineering Evidence & Verification Record

**Project:** Scandium — C++ Real-Time Game Systems & Simulation Platform  
**Repository:** Asterioxer/Scandium  
**Evidence source:** Local Windows verification run captured on 2026-10-06  
**Environment:** Visual Studio Community 2026 Developer PowerShell v18.10.3, MSVC 19.51 toolchain, CMake, Release configuration

## 1. Executive evidence

Scandium was pulled from GitHub at commit `327a0e1`, configured successfully with CMake, built successfully in Release mode, and passed **12/12 automated tests** with zero failures.

This record is intentionally evidence-based: the claims below are limited to behavior directly demonstrated by the captured local run.

## 2. Build verification

Command:

```powershell
cmake -S . -B build -DSCANDIUM_BUILD_TESTS=ON
cmake --build build --config Release
```

Result:
- CMake configure: **PASS**
- Release compilation: **PASS**
- Core library and all configured demo/test targets compiled successfully.

The captured build produced targets covering math, physics, navigation, simulation, ECS/core systems, runtime, arena gameplay, networking, rendering, benchmarks, and demos.

## 3. Automated correctness verification

Command:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Result:

**100% tests passed — 0 tests failed out of 12.**

Verified test suites:
1. Vec3
2. Mat4 / transforms
3. AABB
4. Physics
5. A* / JPS navigation
6. Simulation
7. Core systems
8. Advanced systems
9. Runtime engine
10. Arena gameplay
11. Networking
12. Rendering

Total test execution time: **0.96 seconds**.

## 4. Flagship runtime evidence

The default Scandium flagship runtime completed a deterministic showcase with:

| Metric | Observed result |
|---|---:|
| Agents | **512** |
| Worker threads | **16** |
| Fixed timestep | **0.017 s** |
| Render frames | **120** |
| Simulation ticks | **120** |
| Seek agents | **432** |
| Idle agents | **80** |
| Nearby target-cell population | **170** |
| Interpolation alpha | **0.000** |
| State hash | **13505561316958863475** |
| Trace events | **120** |
| Replay commands | **1** |

This demonstrates that the runtime path is not merely compiling: the engine executes its fixed-step simulation, parallel worker system, spatial targeting, determinism hash, tracing, replay, and render-snapshot pipeline.

## 5. Simulation showcase evidence

The ASCII simulation demo successfully advanced a 32-agent simulation through multiple frames. The captured output shows agents converging toward the target and progressing from frame 0 through frame 29.

This provides a dependency-free demonstration path for the simulation layer without requiring a GPU or external rendering framework.

## 6. Engineering systems represented in the verified build

The verified commit contains the following major systems:

- C++20/CMake build infrastructure
- deterministic fixed-step simulation clock
- job system / parallel agent updates
- event bus
- lightweight ECS registry and object pooling
- vector/matrix/quaternion/transform math
- AABB and ray intersection physics primitives
- spatial hashing
- A* and JPS-style navigation
- steering AI
- arena combat simulation
- deterministic replay/input recording
- state hashing
- render snapshots/interpolation
- trace/profiling infrastructure
- reliability/ACK primitives
- tick-based lockstep networking contracts
- dependency-free software renderer
- FastAPI game-service backend
- PostgreSQL persistence layer
- automated C++ and API CI configuration
- benchmark targets for collision, navigation, and simulation scale

## 7. Performance evidence already captured

Earlier deterministic collision comparison results in the project verification history showed the spatial hash becoming increasingly advantageous as entity count increased.

Observed comparison:

| Entities | Naive (ms) | Spatial hash (ms) |
|---:|---:|---:|
| 100 | 0.0211 | 0.0697 |
| 250 | 0.2623 | 0.3826 |
| 500 | 0.5221 | 0.3962 |
| 1000 | 3.5818 | 1.8103 |
| 2000 | 15.5237 | 4.0284 |

Interpretation: the spatial hash has overhead at small scales, but the captured run shows a clear crossover and substantial benefit at larger entity counts.

## 8. Known limitation from this verification run

The C++ build/test/runtime verification is clean. The captured run did, however, expose a separate Docker Compose configuration problem in the game API service.

Command:

```powershell
docker compose -f services/game_api/docker-compose.yml up --build
```

Observed error:

```
unable to prepare context: path
"C:\Users\soham\Scandium\services\game_api\services\game_api"
not found
```

This is a **Compose build-context path issue**, not a C++ compilation or test failure. It should be fixed before claiming the PostgreSQL Docker stack is locally verified.

## 9. Reproducibility

Recommended local verification sequence:

```powershell
git pull
cmake -S . -B build -DSCANDIUM_BUILD_TESTS=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
build\Release\scandium_demo.exe
build\Release\scandium_ascii_demo.exe
```

The captured run successfully completed every step above.

## 10. Portfolio / interview evidence statement

A defensible interview summary is:

> Scandium is a C++ real-time game-systems platform I built around deterministic simulation and scalable runtime architecture. In my latest Windows verification run, the Release build passed 12/12 automated tests and the flagship runtime executed 120 simulation ticks over 512 agents using 16 workers, with replay recording, deterministic state hashing, tracing, spatial targeting, and render snapshots active.

## 11. Verification boundary

This document deliberately does **not** claim:
- production-ready multiplayer networking;
- production GPU rendering;
- production deployment of the PostgreSQL service;
- a pure mathematically complete JPS implementation;
- a production-grade archetype/SoA ECS;
- universal benchmark performance independent of hardware.

Those claims require additional evidence beyond this captured run.

---

**Status:** C++ core/runtime verification **GREEN**  
**Test status:** **12/12 PASS**  
**Release build:** **PASS**  
**Runtime showcase:** **PASS**  
**Docker Compose API stack:** **BLOCKED BY BUILD-CONTEXT PATH ERROR**
