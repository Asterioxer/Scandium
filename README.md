# Scandium

A high-performance C++20 deterministic multi-agent simulation and systems-engineering platform.

Scandium is an engineering-first simulation engine built for bit-exact deterministic execution, cache-friendly data structures, concurrent job scheduling, and high-throughput multi-agent AI.

---

## Key Architectural Capabilities

### 1. Deterministic Simulation Runtime
- **Fixed-Step Clock:** Simulation updates advance at a strict fixed delta ($dt = 1/60\text{ s} \approx 16.667\text{ ms}$), with max catch-up clamping to prevent spiral-of-death stalls.
- **Double-Buffered State Isolation:** `AgentWorld` maintains read buffers (`agents_`) and write buffers (`next_agents_`). Multi-threaded workers read state at tick $t$ and write to tick $t+1$, guaranteeing bit-exact deterministic state hashes across 1, 2, 4, 8, and 16 worker threads.
- **State Hashing & Diagnostics:** 64-bit FNV-1a state fingerprints verify execution invariance. The `diagnose_state_mismatch` tool isolates divergences down to agent ID and floating-point field level.
- **Serialization & Replay:** Stream and file persistence (`save_to_file`/`load_from_file`) supporting exact scenario replay and malformed input safety.

### 2. Generational Sparse-Set ECS
- **Generational Identifiers:** `Entity{id, generation}` tokens prevent ABA handle reuse bugs.
- **Contiguous Component Storage:** Arbitrary component types are stored in contiguous `ComponentPool<T>` sparse sets for optimal L1/L2 data prefetching.
- **Zero-Allocation Views:** `registry.view<Transform, Velocity, ...>()` iterates multi-component entities with zero runtime heap allocation.

### 3. Allocation-Free Spatial Hashing & Adaptive Broadphase
- **Contiguous Flat Bucket Arrays:** Replaced node-based hash maps with an indexed cell-head array and contiguous entry vector.
- **Sparse-Clear Optimization:** For small entity counts, `clear()` selectively resets only touched buckets rather than wiping all 8,192 words (32 KB), slashing clear overhead by 80x.
- **Adaptive Broadphase:** Automatically switches between a vectorizable Naive pairwise check ($N < 150$) and SpatialHash candidate queries ($N \ge 150$).
- **Zero-Allocation Queries:** `hash.query(pos, visitor)` and `hash.query_radius(center, radius, visitor)` stream neighbor candidates without dynamic allocations.

### 4. Navigation Engine: A* & JPS4
- **Run-Stamped Dense Flat Grid:** Eliminates hash map lookups by storing node costs in flat dense arrays stamped with a monotonic `run_id`, avoiding allocations during search.
- **JPS4 Jump Point Optimization:** Stack-allocated `DirectionSet` eliminates dynamic allocation during jump direction pruning.
- **Line-of-Sight Smoothing:** `AStar::smooth_path` applies Bresenham raycasting to eliminate staircase grid artifacts.
- **Search Profiling:** `find_path_with_metrics` outputs explored node counts, path cost, and duration in microseconds.

### 5. Multi-Agent Steering & Obstacle Avoidance
- **Reynolds Flocking Model:** Composable separation, alignment, and cohesion steering forces.
- **Corridor Waypoint Following:** Dynamic navigation grid integration; agents automatically advance along waypoints to goal destinations.
- **Dynamic Obstacle Avoidance:** Repulsive potential fields steer agents smoothly around dynamic circular obstacles.
- **Mutual Agent Separation:** Soft-repulsion forces prevent overcrowding in doorways and narrow corridors.

### 6. Concurrency & Adaptive Job Scheduling
- **Small Functor Optimization (SFO):** 64-byte inline aligned task storage eliminates per-job heap allocations.
- **Caller Participation:** Calling threads execute chunks while awaiting workers, minimizing kernel context switches for short tasks.
- **Adaptive Execution Thresholds:** Configurable `min_elements_per_task` (default 1,024) and `parallel_threshold` (default 2,000 agents) ensure small workloads execute sequentially on the calling thread, avoiding condition-variable wakeup penalties.
- **Deadlock Safeguards:** Thread-local worker detection prevents recursive parallel-for thread pool starvation.

### 7. Linear Bounding Volume Hierarchy (LBVH)
- **63-Bit Morton Codes:** Quantizes 3D coordinates into a $2,097,152^3$ virtual space using bit-expansion across $X, Y, Z$ axes for spatial ordering.
- **Hierarchical Flat Node Array:** Compact 32-byte cache-aligned node storage with non-recursive stack-based traversal.
- **Logarithmic Spatial & Raycast Queries:** Resolves closest ray hits and AABB queries in $O(\log N)$ time, achieving 95,253 rays/sec and 1.75x speedup over linear search.

### 8. 8-Connected Diagonal Navigation
- **Octile Distance Heuristic:** Uses admissible $h = (\sqrt{2} - 1)\min(\Delta x, \Delta y) + \max(\Delta x, \Delta y)$ heuristic, reducing path distance by 29.3% over cardinal Manhattan grids.
- **Corner-Cutting Prevention:** Enforces non-cutting rules ensuring agents cannot slip diagonally through adjacent blocked cardinal obstacles.
- **Search Acceleration:** Explores 71 nodes vs 1,694 nodes on open grids, speeding up path calculation by 13.0x.

### 9. SIMD Vectorized Steering & SoA Physics
- **Structure-of-Arrays (SoA):** Separate contiguous arrays for positions, velocities, radii, and max speeds for cache prefetching.
- **Hardware Acceleration:** 4-wide SSE vectorization (`_mm_loadu_ps`, branchless distance masking, horizontal reduction) yields 2.35x faster Reynolds separation (65.0 ms vs 152.9 ms).

### 10. Client-Side Prediction & Deterministic Rollback Netcode
- **State Ring Buffer:** High-performance fixed-capacity circular buffer with $O(1)$ bitwise modular indexing (`StateRingBuffer<Capacity>`).
- **Zero-Latency Prediction:** Executes local ticks immediately while speculatively extrapolating remote inputs.
- **Bit-Exact Rollback:** Automatically detects authoritative packet divergence, rewinding world state to the divergence tick and resimulating forward in zero visual latency.

### 11. Batch Simulation Pipeline & GPU Compute Shader Export
- **16-Byte Aligned StructuredBuffers:** `GPUAgent` and `GPUSimulationParams` structs match HLSL / GLSL / WebGPU standards.
- **Wavefront Execution:** CPU emulator processes threadgroups of 64 agents in parallel across cores.
- **DirectX/Vulkan Ready:** Exports production HLSL compute shader source (`BatchSimulationPipeline::get_hlsl_compute_shader()`).

---

## Visual Demonstration & Observability

Scandium provides two decoupled visualization frontends:

### 1. Standalone HTML5 Canvas Visualizer
- **Tracked Distributable File:** [`docs/scandium_simulation_viewer.html`](docs/scandium_simulation_viewer.html)  
  *Open directly in any modern web browser via `file://` with zero build or runtime prerequisites.*
- **Local Generator Target:** Running `.\build\scandium_visualizer_demo.exe` generates `scandium_simulation_viewer.html` in the current working directory.
- **Features:**
  - 60 FPS interactive Canvas playback with scrub bar, timeline stepping, and speed selector.
  - Proximity alert halos highlighting agents with separation $< 2.2\text{ m}$.
  - Route visibility toggle: focus on selected agent path or view all corridor routes.
  - Traversed trail breadcrumbs vs planned upcoming waypoints.
  - Interactive agent inspector displaying ID, position, velocity, and remaining waypoints.
  - Live performance HUD showing tick rate, clock budget utilization, and arrival completion percentage.

### 2. Terminal ASCII Dashboard
- Built and executed via `.\build\scandium_ascii_demo.exe`.
- Renders agents (`@`), doorways/walls (`#`), arrival points (`*`), and collision proximity markers (`!`) in real-time terminal output.

---

## Benchmark Methodology & Empirical Results

### Measurement Rigor
- **Clock:** High-resolution monotonic clock (`std::chrono::steady_clock`).
- **Warmup:** Minimum 2 warmup passes per workload before sampling.
- **Repeated Trials:** 5 measured trials reporting median runtimes (and p95).
- **Optimization Barriers:** Memory barriers (`do_not_optimize`) prevent compiler dead-code elimination.
- **Scale:** Workloads configured with sufficient operations ($50,000$ queries, 100 paths, 120 simulation ticks) to ensure execution durations exceed 5 ms, eliminating timer resolution quantization.

### Test Environment
- **Host CPU:** AMD/Intel x64 Architecture (16 Logical Processors)
- **Compiler:** Microsoft Visual C++ 19.51.36260 (x64) with C++20 Standard
- **Build Configuration:** CMake 4.3.1 / Ninja 1.13.2 / Release Mode (`/O2 /Ob2 /DNDEBUG`)

### Empirical Performance Summary

#### A. Spatial Hash vs Naive Broadphase
| Entity Count | Naive All-Pairs (ms) | Spatial Hash (ms) | Speedup vs Naive | Selected Strategy |
|:---:|:---:|:---:|:---:|:---:|
| **100** | **0.0158 ms** | 0.0497 ms | 0.32x (Naive faster) | Naive Direct |
| **250** | 0.1591 ms | **0.0733 ms** | **2.17x faster** | Spatial Hash |
| **500** | 0.5574 ms | **0.0870 ms** | **6.41x faster** | Spatial Hash |
| **1,000** | 1.4038 ms | **0.1683 ms** | **8.34x faster** | Spatial Hash |
| **2,000** | 5.9267 ms | **0.4476 ms** | **13.24x faster** | Spatial Hash |

#### B. Navigation Engine Across 5 Topologies (100 Paths per Topology)
| Topology | Grid Size | A* Median (ms) | JPS Median (ms) | Optimal Algorithm | Explored Nodes (A* / JPS) | Path Cost Match |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Corridors** | 96 x 96 | 0.8169 ms | **0.6827 ms** | **JPS (1.20x faster)** | 3,688 / 3,688 | **true** |
| **Dense Labyrinth** | 64 x 64 | 0.5942 ms | **0.5147 ms** | **JPS (1.15x faster)** | 3,048 / 3,048 | **true** |
| **Random Obstacles (20%)** | 128 x 128 | 1.4251 ms | **1.3837 ms** | **JPS (1.03x faster)** | 6,985 / 6,985 | **true** |
| **Open Grid** | 128 x 128 | **0.6861 ms** | 0.8984 ms | **A\* (1.31x faster)** | 3,864 / 3,864 | **true** |
| **Sparse Obstacles (5%)** | 128 x 128 | **0.7169 ms** | 0.7507 ms | **A\* (1.05x faster)** | 3,821 / 3,821 | **true** |

*In enclosed, corridor, and obstacle-dense maps, JPS significantly outperforms A* by pruning open-queue operations through ray-skipping. On open grids, A* with Manhattan distance heuristic avoids JPS's perpendicular column scans and achieves superior cache locality.*

#### C. Multi-Agent Simulation Scale (120 Frames, Adaptive Threshold = 2,000)
| Agents | 1 Worker (Seq) | 2 Workers | 4 Workers | 8 Workers | 16 Workers | Average Tick Duration |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **500** | 1.30 ms | 1.28 ms | 1.38 ms | 1.37 ms | **1.49 ms** | **12.4 µs** |
| **1,000** | 2.88 ms | 2.57 ms | 2.60 ms | 2.60 ms | **2.71 ms** | **22.6 µs** |
| **5,000** | 12.99 ms | 13.32 ms | 12.45 ms | 16.25 ms | **14.84 ms** | **123.7 µs** |
| **10,000** | 28.39 ms | 23.35 ms | 26.94 ms | 25.01 ms | **24.33 ms** | **202.8 µs** |
| **20,000** | 56.06 ms | 41.73 ms | **35.20 ms** | 40.13 ms | **35.35 ms** | **294.6 µs (0.29 ms)** |

---

## Clean Build & Reproduction Instructions

### Prerequisites
- C++20 compliant compiler (MSVC 19.30+, GCC 11+, or Clang 13+)
- CMake 3.20+
- Ninja build tool (recommended) or Visual Studio 2022+

### Step-by-Step Build
```powershell
# 1. Open the Developer PowerShell for Visual Studio (x64)
& "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\Launch-VsDevShell.ps1" -Arch amd64

# 2. Configure Release build with tests and benchmarks enabled
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSCANDIUM_BUILD_TESTS=ON -DSCANDIUM_BUILD_BENCHMARKS=ON

# 3. Compile all targets
cmake --build build

# 4. Run the full automated CTest suite (15/15 tests)
ctest --test-dir build --output-on-failure

# 5. Execute benchmarks
.\build\scandium_collision_comparison.exe
.\build\scandium_spatial_hash_benchmark.exe
.\build\scandium_navigation_comparison.exe
.\build\scandium_job_system_benchmark.exe
.\build\scandium_simulation_scale.exe

# 6. Generate and view interactive demonstration
.\build\scandium_visualizer_demo.exe
# Open docs\scandium_simulation_viewer.html in any browser
```

---

## Future Advancements & Roadmap

The following architectural milestones represent the next phase of evolution for Scandium:

1. **SIMD Vectorized Steering & Spatial Queries:**
   - Implement AVX2 / AVX-512 and ARM NEON vectorized kernels for Reynolds flocking and agent distance calculations, processing 8-16 agent interactions per SIMD vector register.
2. **8-Connected Diagonal Navigation & Hierarchical Pathfinding (HPA*):**
   - Extend the navigation grid to 8-connected diagonal movement with Octile heuristics and true diagonal jump-point pruning.
   - Introduce Hierarchical Pathfinding (HPA*) with coarse cluster abstract graphs for maps larger than 1,024×1,024.
3. **Dynamic Multi-Scale Bounding Volume Hierarchy (BVH):**
   - Supplement the uniform spatial hash with a Linear BVH (LBVH) / Morton-code radix-sorted octree to efficiently support entities of widely varying physical radii.
4. **Deterministic Client-Side Prediction & Rollback Network Transport:**
   - Enhance the lockstep network protocol with speculative client prediction and local state ring buffers for lag-free multi-agent multiplayer interactions.
5. **GPU Compute Shader Simulation Backend (Vulkan / WebGPU):**
   - Provide an optional compute shader simulation path capable of simulating 100,000+ boids at 60 Hz with zero CPU readback.

---

## License
MIT License. Copyright (c) 2026 Soham Mukherjee.
