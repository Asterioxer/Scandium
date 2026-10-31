# Scandium Architecture & Systems Design

Scandium is a high-performance C++20 simulation and systems-engineering engine engineered for deterministic multi-agent worlds, spatial hashing, navigation, and concurrent simulation scaling.

## Architectural Overview

```
                      +---------------------------------------+
                      |         Application / Visualizer      |
                      |  (HTML5 Interactive Canvas / PPM)     |
                      +-------------------+-------------------+
                                          |
                                 RenderSnapshot
                                          |
                      +-------------------v-------------------+
                      |            Runtime Engine             |
                      | (Fixed-Step Clock, Events, Replays)   |
                      +---------+---------+---------+---------+
                                |         |         |
                      +---------+   +-----+-----+   +---------+
                      | Generational|   | Job System|   |  Spatial|
                      | Sparse ECS  |   | (Caller-  |   |  Hashing|
                      | Registry    |   |  Assisted)|   |  (Flat) |
                      +---------+   +-----+-----+   +---------+
                                |         |         |
                      +---------v---------v---------v---------+
                      |         AgentWorld Simulation         |
                      |  (Double-Buffered Deterministic Step) |
                      +---------+-------------------+---------+
                                |                   |
                      +---------v---------+   +-----v---------+
                      |  Navigation (A*   |   |   Steering &  |
                      |  & JPS4 Jumping)  |   |   Avoidance   |
                      +-------------------+   +---------------+
```

## Core Systems & Implementation Details

### 1. Deterministic Simulation Clock & Double Buffering
- **Fixed-Step Clock:** Simulation updates occur at strict fixed intervals (default `60 Hz` / `16.667 ms`), accumulating elapsed wall time and bounded by a maximum catch-up limit to eliminate spiral-of-death stalls.
- **Double-Buffered State Isolation:** `AgentWorld` maintains read buffers (`agents_`) and write buffers (`next_agents_`). When systems run concurrently across threads, agents read positions at tick $t$ and write new states to tick $t+1$. This guarantees bit-exact, thread-count-independent determinism whether executing on 1, 4, or 16 CPU cores.
- **State Hashing & Diagnostics:** FNV-1a-based 64-bit state fingerprints allow instant regression detection. `diagnose_state_mismatch` provides fine-grained delta reporting, pinpointing exact agent IDs and floating-point divergences.
- **Replay Persistence:** Replay streams support full text and stream serialization (`save`/`load`), allowing simulation traces to be recorded, persisted, and verified against golden baselines.

### 2. Generational Sparse-Set ECS
- **Entity Lifecycle:** Entities are tracked via generational identifiers (`EntityId` and `Entity{id, generation}`) to protect against use-after-free and ABA bugs.
- **Dense Component Storage:** Arbitrary components are stored in contiguous `ComponentPool<T>` sparse sets. Dense component arrays preserve L1/L2 cache locality during updates.
- **Zero-Allocation Views:** `registry.view<Transform, Velocity, ...>()` performs multi-component queries without dynamic allocations.

### 3. Allocation-Free Flat Spatial Hash & Adaptive Broadphase
- **Flat Bucket Chaining:** Replaced node-based `std::unordered_map` with an indexed cell-head array and a contiguous entry vector.
- **Sparse-Clear Optimization:** For small workloads where entity count $N \ll \text{bucket\_count}$, `clear()` selectively resets only touched buckets rather than wiping all 8,192 words (32 KB), reducing memory clearing overhead by up to 80x.
- **Adaptive Broadphase Strategy:** `AdaptiveBroadphase` supports `Auto`, `Naive`, and `SpatialHash` modes. At small entity counts ($N < 150$), naive contiguous pairwise checks run directly in L1 cache, avoiding hash setup overhead. Beyond the crossover threshold, spatial hash candidate queries dominate.
- **Visitor Query API:** `hash.query(pos, callback)` and `hash.query_radius(center, radius, callback)` deliver zero-allocation spatial queries directly into callers.

### 4. Navigation Engine: A* & JPS4
- **Flat Run-Stamped Grid Search:** Grid searches eliminate hash map overhead via dense flat arrays and generation run-stamps (`run_id`).
- **Stack-Allocated JPS Directions:** Pruned directions return a fixed-size `DirectionSet` directly on the stack, eliminating dynamic heap allocations in hot expansion loops.
- **Topology & Algorithmic Analysis:** In 4-connected cardinal grids, JPS requires perpendicular raycast scans (`jump_vertical_check`) at each horizontal step, incurring higher search setup and scanning cost than cache-friendly A* with Manhattan heuristic. Both algorithms guarantee identical path lengths and costs.
- **Line-of-Sight Smoothing:** `smooth_path` applies Bresenham raycasting to eliminate Manhattan staircase artifacts, producing natural paths.
- **Metrics Telemetry:** `find_path_with_metrics` reports explored nodes, path cost, and execution duration in microseconds.

### 5. Caller-Assisted Concurrency & Adaptive Job System
- **Inline Functor Storage (SFO):** Small Functor Optimization (64 bytes) stores task closures inline, eliminating heap allocation on task enqueue.
- **Adaptive Execution Threshold:** `JobSystem` introduces configurable `min_elements_per_task` (default 1024). Micro-workloads below this threshold execute sequentially on the calling thread, eliminating kernel context switches and condition variable wakeup latency.
- **Adaptive Agent Parallelization:** `AgentWorldConfig` provides `parallel_threshold` (default 2,000 agents) and `min_agents_per_chunk` (default 512). Small agent counts update sequentially to avoid multi-thread dispatch overhead, while larger agent populations scale across worker pools.
- **Caller Participation:** The calling thread does not sleep; it directly executes parallel chunks, eliminating context-switch overhead for short tasks.
- **Exception Resilience:** Worker threads intercept exceptions cleanly, preserving worker thread lifetime and preventing process aborts.

### 6. Observability & Interactive Visualisation
- **Decoupled Architecture:** Simulation engine is strictly decoupled from presentation.
- **Interactive HTML5 Visualizer:** `scandium_visualizer_demo` exports an interactive 60 FPS HTML5 Canvas viewer (`scandium_simulation_viewer.html`) featuring:
  - Configurable minimum separation proximity alerts (dashed red halo & connector lines).
  - Focused route visibility (toggle selected agent path vs all paths).
  - Traversed trail breadcrumbs vs planned upcoming waypoints.
  - Interactive agent inspector with live velocity, position, and remaining waypoints.
  - Simulation completion summary with percentage progress and state telemetry.
- **Scenario Arrival Kinematics:** Demonstrates both long-range corridor navigation and short-range destination arrivals (`AgentState::Arrived`).
- **ASCII Simulation Dashboard:** Terminal-based visualizer with dynamic obstacle markers, collision indicators, and state telemetry.
