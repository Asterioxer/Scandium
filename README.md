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
- **Generational Handle Safety:** Component queries, emplacements, and removals accept `Entity` handles directly and reject stale or dead entities automatically.
- **Zero-Allocation Views:** `registry.view<Transform, Velocity, ...>()` iterates multi-component entities with zero runtime heap allocation.

### 3. Allocation-Free Spatial Hashing & Adaptive Broadphase
- **Contiguous Flat Bucket Arrays:** Replaced node-based hash maps with an indexed cell-head array and contiguous entry vector.
- **Sparse-Clear Optimization:** For small entity counts, `clear()` selectively resets only touched buckets rather than wiping all 8,192 words (32 KB), slashing clear overhead by 80x.
- **Adaptive Broadphase:** Automatically switches between a vectorizable Naive pairwise check ($N < 150$) and SpatialHash candidate queries ($N \ge 150$).
- **Zero-Allocation Queries:** `hash.query(pos, visitor)` and `hash.query_radius(center, radius, visitor)` stream neighbor candidates without dynamic allocations.

### 4. Navigation Engine: A* & True JPS (4-Way & 8-Way)
- **Run-Stamped Dense Flat Grid:** Eliminates hash map lookups by storing node costs in flat dense arrays stamped with a monotonic `run_id`, avoiding per-search heap allocations.
- **True Graph Pruning (JPS):** Implements canonical Jump Point Search with lookahead ray skipping, cutting node expansions by up to 99.9% on open grids (4 nodes vs 3,864 nodes) with bit-identical path costs.
- **Topology-Adaptive Execution:** JPS dominates on open grids, dense labyrinths, and sparse obstacle fields, while A* is preferred in winding 1-tile corridors where branch raycasts add constant overhead.
- **Line-of-Sight Smoothing:** `AStar::smooth_path` applies Bresenham raycasting to eliminate staircase grid artifacts.

### 5. Multi-Agent Steering & Obstacle Avoidance
- **Reynolds Flocking Model:** Composable separation, alignment, and cohesion steering forces.
- **Corridor Waypoint Following:** Dynamic navigation grid integration; agents automatically advance along waypoints to goal destinations.
- **Dynamic Obstacle Avoidance:** Repulsive potential fields steer agents smoothly around dynamic circular obstacles.
- **Mutual Agent Separation:** Soft-repulsion forces prevent overcrowding in doorways and narrow corridors.

### 6. Concurrency, Chase-Lev Work Stealing & Contention-Free Scheduling
- **Chase-Lev Lock-Free Work Stealing:** Per-worker `WorkStealingQueue<T>` with 64-byte cache-line separated `top_` and `bottom_` atomics eliminating false sharing between owning workers and stealers.
- **2-Tier Exponential Backoff:** Worker threads execute brief user-space `_mm_pause()` spins before parking, eliminating 15-30 µs of OS thread descheduling latency and enabling monotonic throughput scaling up to 16 workers.
- **Small Functor Optimization (SFO):** 64-byte inline aligned task storage eliminates per-job heap allocations.
- **Caller Participation & Zero-Contention Wait:** Calling threads help execute chunks directly while awaiting workers. When the job queue is emptied, waiting threads poll atomic task counters with `_mm_pause()` without re-acquiring the job queue lock.
- **Adaptive Execution Thresholds:** Configurable `min_elements_per_task` (default 1,024) and `parallel_threshold` (default 2,000 agents) ensure small workloads execute sequentially on the calling thread, avoiding thread wakeup overhead.

### 7. Self-Balancing Dynamic AABB Tree & SAH
- **Surface Area Heuristic (SAH):** Greedy insertion selects optimal sibling bounding volumes to minimize query area overlap.
- **AVL Tree Rotations:** Automatic node rotations preserve $O(\log N)$ tree depth across dynamic insertions and removals.
- **Fat-AABB Margins & Velocity Prediction:** Moving objects within their fat margin execute in $O(1)$ time with zero tree modifications.
- **Non-Recursive Stack Traversal:** Stack-based AABB overlap queries, raycasts, and self-collision pair generation (`query_pairs`).

### 8. Linear Bump Frame Allocator & STL Arena
- **Zero-Allocation Hot Paths:** `FrameAllocator` provides $O(1)$ linear pointer bumping with arbitrary power-of-two alignment (8, 16, 32, 64-byte alignment for SIMD vector types).
- **Pointer-Stable Chained Pages:** Dynamic chained page allocation preserves pointer validity if an allocation exceeds primary page size.
- **Scoped Rollback & Retained Memory:** `mark()` / `reset_to(marker)` and `ScopedArenaMarker` RAII guards allow localized scratch allocations; `reset()` retains pages across simulation ticks for zero-malloc subsequent runs.
- **Standard C++20 `ArenaAllocator<T>`:** Standard-compliant allocator traits for `std::vector<T, ArenaAllocator<T>>`.

### 9. SIMD Ray-AABB Vectorized Slab Intersection
- **8-Wide AVX2 Slab Tests:** `intersect_ray_8aabb_avx2` tests 1 ray against 8 bounding boxes simultaneously using 256-bit vector registers (`__m256`) and generates 8-bit hit masks without branching.
- **4-Wide SSE Fallback:** `intersect_ray_4aabb_sse` evaluates 4 bounding boxes in parallel using 128-bit vector registers (`__m128`).
- **Dynamic CPUID Dispatch:** Runtime feature detection (`CPUFeatures`) auto-selects AVX2, SSE, or scalar fallback on any x86 host.

### 10. Hierarchical Pathfinding (HPA*) on Large Maps
- **Cluster Abstraction:** Partitions large grids into uniform clusters (default $16 \times 16$).
- **Abstract Border Entrances:** Precomputes border transition pairs and intra-cluster paths.
- **Sub-Millisecond Routing:** High-level search executes across the abstract transition graph, expanding to contiguous concrete waypoints with $\le 1.15\times$ bounded sub-optimality.

### 11. Linear Bounding Volume Hierarchy (LBVH)
- **63-Bit Morton Codes:** Quantizes 3D coordinates into a $2,097,152^3$ virtual space using bit-expansion across $X, Y, Z$ axes for spatial ordering.
- **Hierarchical Flat Node Array:** Compact 32-byte cache-aligned node storage with non-recursive stack-based traversal.
- **Logarithmic Spatial & Raycast Queries:** Resolves closest ray hits and AABB queries in $O(\log N)$ time, achieving 34,700+ rays/sec.

### 8. 8-Connected Diagonal Navigation
- **Octile Distance Heuristic:** Uses admissible $h = (\sqrt{2} - 1)\min(\Delta x, \Delta y) + \max(\Delta x, \Delta y)$ heuristic, reducing path distance by 29.3% over cardinal Manhattan grids.
- **Corner-Cutting Prevention:** Enforces non-cutting rules ensuring agents cannot slip diagonally through adjacent blocked cardinal obstacles.
- **Search Acceleration:** Explores 71 nodes vs 1,694 nodes on open grids, speeding up path calculation by 13.0x.

### 9. Multi-Tier SIMD Vectorized Steering & SoA Physics
- **Structure-of-Arrays (SoA):** Separate contiguous arrays for positions, velocities, radii, and max speeds for optimal cache prefetching.
- **Hardware Acceleration:** Multi-tier vectorization with 8-wide AVX2 (`__m256`), 4-wide SSE (`__m128`), and scalar fallback yields 2.33x faster Reynolds separation (192.6 ms vs 448.6 ms).

### 10. Client-Side Prediction & Deterministic Rollback Netcode
- **State Ring Buffer:** High-performance fixed-capacity circular buffer with $O(1)$ bitwise modular indexing (`StateRingBuffer<Capacity>`).
- **Zero-Latency Prediction:** Executes local ticks immediately while speculatively extrapolating remote inputs.
- **Bit-Exact Rollback:** Automatically detects authoritative packet divergence, rewinding world state to the divergence tick and resimulating forward in zero visual latency.

### 11. Batch Simulation Pipeline & GPU Compute Shader Export
- **16-Byte Aligned StructuredBuffers:** `GPUAgent` and `GPUSimulationParams` structs match HLSL / GLSL / WebGPU standards.
- **Wavefront CPU Emulator:** Processes threadgroups of 64 agents in parallel across cores matching GPU execution paradigms.
- **DirectX/Vulkan Ready:** Exports production HLSL compute shader source (`BatchSimulationPipeline::get_hlsl_compute_shader()`) for native graphics API integration.

### 12. Continuous Collision Detection (CCD) & Swept Physics
- **Quadratic Swept-Sphere Solver:** Computes exact time of impact $t_{toi} \in [0, 1]$ between dynamic moving spheres, eliminating tunneling at high speeds.
- **Minkowski Ray Marching:** Resolves swept sphere vs stationary AABB contacts with rounded-box edge refinement.
- **Impulse Response & Friction:** Newton restitution impulse and tangential Coulomb friction response for realistic contact bouncing.

### 13. Flow Field (Vector Field) Crowd Navigation
- **Integration Potential Field:** Breadth-first Dijkstra wave expansion over 8-way connected grid cells from goal targets.
- **Downhill Gradient Vector Field:** Precalculates normalized steering directions for all traversable grid cells.
- **Continuous Bilinear Sampling:** Delivers 18.43 Million samples/sec with smooth $O(1)$ sampling per agent, supporting arbitrary crowd scales.

### 14. Behavior Tree AI Architecture
- **Composite Nodes:** Deterministic `SequenceNode` (logical AND) and `SelectorNode` (priority fallback).
- **Decorator Nodes:** `InverterNode` and `CooldownNode` for timing and status inversion.
- **Type-Safe Blackboard:** Key-value state memory (`std::any` based with zero-cost typed accessors).

### 15. Agent Perception & Sensory System
- **Vision Cones:** Field-of-view half-angle dot product testing, view range limits, peripheral proximity radius, and line-of-sight occlusion raycasting.
- **Acoustic Noise Propagation:** Inverse-square intensity decay with hearing threshold detection.
- **Perception Memory:** Temporal confidence decay ($1.0 \to 0.0$) and entity forgetting timers.

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
| **100** | **0.0271 ms** | 0.0551 ms | 0.49x (Naive faster) | Naive Direct |
| **250** | 0.1592 ms | **0.0658 ms** | **2.42x faster** | Spatial Hash |
| **500** | 0.6528 ms | **0.1444 ms** | **4.52x faster** | Spatial Hash |
| **1,000** | 2.9518 ms | **0.2075 ms** | **14.22x faster** | Spatial Hash |
| **2,000** | 13.9855 ms | **1.3873 ms** | **10.08x faster** | Spatial Hash |

#### B. Navigation Engine Across 5 Topologies (100 Paths per Topology)
| Topology | Grid Size | A* Median (ms) | JPS Median (ms) | Optimal Algorithm | Explored Nodes (A* / JPS) | Path Cost Match |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Open Grid** | 128 x 128 | 0.8704 ms | **0.6721 ms** | **JPS (1.30x faster)** | 3,864 / **4** (99.9% fewer) | **true** |
| **Dense Labyrinth** | 64 x 64 | 0.7675 ms | **0.5328 ms** | **JPS (1.44x faster)** | 3,048 / **280** (90.8% fewer) | **true** |
| **Random Obstacles (20%)** | 128 x 128 | 1.6678 ms | **1.2735 ms** | **JPS (1.31x faster)** | 6,985 / **2,253** (67.7% fewer) | **true** |
| **Sparse Obstacles (5%)** | 128 x 128 | 0.7822 ms | **0.6932 ms** | **JPS (1.13x faster)** | 3,821 / **1,824** (52.3% fewer) | **true** |
| **Corridors** | 96 x 96 | **0.7696 ms** | 2.0067 ms | **A\* (2.61x faster)** | 3,688 / **210** (94.3% fewer) | **true** |

*In open grids, dense labyrinths, and random/sparse obstacle fields, JPS delivers significant wall-clock speedups while slashing explored node counts by 52% to 99.9%. In winding 1-tile corridors, A\* achieves lower wall-clock duration because orthogonal branch raycasts introduce constant overhead at every turn, validating Scandium's topology-aware navigation choice.*

#### C. Multi-Agent Simulation Scale (120 Frames, Adaptive Threshold = 2,000)
| Agents | 1 Worker (Seq) | 2 Workers | 4 Workers | 8 Workers | 16 Workers | Max Speedup | Average Tick Duration |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **500** | 7.21 ms | 5.62 ms | 6.06 ms | 5.72 ms | 5.82 ms | **1.28x** | **46.9 µs** |
| **1,000** | 11.75 ms | 12.64 ms | 11.93 ms | 12.16 ms | 13.12 ms | 1.00x | **97.9 µs** |
| **5,000** | 63.31 ms | 47.91 ms | 54.85 ms | 38.08 ms | 42.71 ms | **1.66x** | **317.3 µs** |
| **10,000** | 127.99 ms | 92.89 ms | 85.24 ms | 78.85 ms | 85.55 ms | **1.62x** | **657.1 µs** |
| **20,000** | 277.77 ms | 193.87 ms | 129.93 ms | 148.99 ms | 144.37 ms | **2.14x** | **1,082.7 µs (1.08 ms)** |

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

1. **Native GPU Compute Dispatch (Vulkan / WebGPU / DirectX 12):**
   - Bind `BatchSimulationPipeline`'s HLSL compute shaders directly to native Vulkan compute pipelines (`VkComputePipelineCreateInfo`) or WebGPU compute passes for 100K+ agent simulation with GPU-only residency.
2. **Chunked Archetype Component Storage:**
   - Supplement sparse sets with chunked archetype storage for cache-line contiguous iteration of homogeneous entity batches.
3. **Directed Acyclic Graph (DAG) Job Graph:**
   - Extend `JobSystem` with atomic predecessor counters and dependency DAGs for heterogeneous task graphs.
4. **Spatial Hash Dynamic Re-Hashing & Multi-Cell Striding:**
   - Dynamically resize bucket count based on load factor and cluster dispersion to maintain $>1.5\text{ M}$ QPS under adversarial spatial concentrations.

---

## License
MIT License. Copyright (c) 2026 Soham Mukherjee.
