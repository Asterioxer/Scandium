# Scandium Engineering Log

This log records every significant engineering implementation, root-cause investigation, and architectural advancement.

---

## [2026-11-09] - Jump Point Search (JPS) True Graph Pruning & Fix of Silent A* Fallback

- **Subsystem:** Navigation (`navigation/jps.hpp`)
- **Problem:** JPS benchmark reported identical explored node counts as A* across all 5 topologies. Initial profiling revealed that `find_path_with_metrics` was exhausting and hitting a hidden fallback: `if (!found) return AStar::find_path_with_metrics(...)`.
- **Root Cause:**
  1. Directional asymmetry: horizontal scanning invoked `jump_vertical_check`, but vertical scanning had no orthogonal lookahead.
  2. Direction pruning loss: when an orthogonal scan triggered a jump point, the expanded node was restricted solely to the original axis in `pruned_directions`, dropping the orthogonal trigger direction.
- **Solution:**
  1. Implemented bidirectional orthogonal lookaheads (`jump_4way_orthogonal`).
  2. Enhanced neighbor expansion to explore both axis continuation and orthogonal branches.
  3. Completely removed the silent A* fallback.
  4. Added canonical 8-way diagonal JPS with corner-cutting prevention.
- **Validation:**
  - OpenGrid 128x128: Explored nodes dropped by 99.9% (4 nodes vs 3,864 nodes).
  - Path costs matched A* bit-for-bit across all 5 topologies.
  - Wall-clock speedups observed in 4 out of 5 topologies (up to 1.44x faster).

---

## [2026-11-09] - Concurrency Optimization: Spin-Wait Lock Contention Elimination

- **Subsystem:** Core Concurrency (`core/job_system.hpp`)
- **Problem:** When scaling to 8 and 16 worker threads, job completion latency regressed on medium/bulk workloads.
- **Root Cause:** In `parallel_for_chunked`, the calling thread entered a busy spin loop acquiring `mutex_` repeatedly even after all tasks were already popped by workers, causing severe cache line bouncing and lock contention.
- **Solution:** Added `std::atomic<std::size_t> unpopped_jobs_{0}`. When `unpopped_jobs_ == 0`, waiting threads bypass mutex acquisition entirely and spin on atomic task completion flags using `_mm_pause()`.
- **Validation:** 100,000-task speedup at 16 workers increased from 1.29x to 2.78x (0.0416 ms vs 0.1155 ms).

---

## [2026-11-09] - Generational ECS Safety & Wraparound Protection

- **Subsystem:** ECS (`ecs/registry.hpp`, `ecs/entity.hpp`)
- **Problem:** Potential ABA generation wraparound if generation counter overflows to 0 (which is reserved for invalid handle tokens). Stale `Entity` handles lacked direct overloads for `emplace`, `get`, `has`, `remove`.
- **Solution:**
  1. Added zero-wraparound guard: `if (entities_[id].generation == 0) entities_[id].generation = 1;`.
  2. Implemented `Entity` generational handle overloads that validate `alive(entity)` before delegating to the sparse-set pools.
- **Validation:** Added tests in `core_tests.cpp` validating 1,000-cycle entity recycling, stale handle rejection, and dense-array consistency.

---

## [2026-11-09] - Multi-Tier SIMD Steering Dispatch (AVX2 + SSE + Scalar)

- **Subsystem:** AI & Simulation (`ai/simd_steering.hpp`)
- **Problem:** SIMD steering only supported 4-wide SSE1 instructions, underutilizing 256-bit AVX2 hardware.
- **Solution:** Added 8-wide AVX2 vector paths (`__m256`) with AVX2 horizontal sum, branchless distance comparison masks, and automatic fallback to 4-wide SSE and scalar remainder loops.
- **Validation:** Achieved 2.33x speedup on SoA agent separation (192.6 ms vs 448.6 ms) in `advancements_benchmark.cpp`.

---

## [2026-11-09] - CPUID Runtime Feature Detection & Dynamic SIMD Dispatch

- **Subsystem:** Core & AI (`core/cpu_features.hpp`, `ai/simd_steering.hpp`)
- **Problem:** SIMD kernels relied on static compile-time flags (`#if defined(__AVX2__)`). Binaries built with standard x64 flags were unable to utilize 256-bit AVX2 vector registers, while binaries compiled with `/arch:AVX2` risked illegal instruction crashes on CPUs lacking AVX2.
- **Solution:**
  1. Created `core/cpu_features.hpp` using cross-compiler CPUID detection (`__cpuidex` on MSVC, `__cpuid_count` on GCC/Clang) identifying SSE, SSE2, SSE3, SSSE3, SSE4.1, SSE4.2, AVX, AVX2, FMA, AVX-512, and ARM NEON.
  2. Implemented dynamic kernel dispatch in `ai::compute_simd_separation_soa` with `SIMDInstructionSet::Auto` selecting AVX2 or SSE at runtime safely.
  3. Added differential validation in `multi_agent_tests.cpp` confirming $< 10^{-3}$ numerical equivalence across Scalar, SSE, AVX2, and Auto backends.
- **Validation:** Detected host CPU `12th Gen Intel(R) Core(TM) i5-12500H` (AVX2=1, SSE4.1=1), all differential tests passed.

---

## [2026-11-09] - Hierarchical Pathfinding (HPA*) on Large Maps

- **Subsystem:** Navigation (`navigation/hpa.hpp`)
- **Problem:** On large grid environments, standard flat A* searches tens of thousands of individual cells, causing routing latency to scale with $O(N \log N)$ over the full map area.
- **Solution:**
  1. Implemented Hierarchical Pathfinding A* (HPA*) according to the Botea et al. formulation.
  2. Partitions grid into uniform $C \times C$ clusters (default $16 \times 16$).
  3. Precomputes entrance pairs along horizontal and vertical cluster borders.
  4. Precomputes intra-cluster edges via bounded local A* sweeps.
  5. Dynamically links start and goal to cluster entrances and performs high-level search across the abstract transition graph, expanding into contiguous concrete grid waypoints.
- **Validation:** Added tests in `astar_tests.cpp` validating abstract graph construction, doorway traversal, contiguous step verification, and $\le 1.15\times$ bounded sub-optimality on 64x64 maps.

---

## [2026-11-09] - Linear Bump Frame Allocator & STL-Compliant ArenaAllocator

- **Subsystem:** Memory Architecture (`core/frame_allocator.hpp`)
- **Problem:** Frequent dynamic allocations (`malloc`/`free`) in simulation ticks cause memory fragmentation, allocator lock contention across threads, and cache misses.
- **Solution:**
  1. Implemented `FrameAllocator` supporting $O(1)$ pointer bumping with arbitrary power-of-two alignment (8, 16, 32, 64-byte alignment for SIMD vector types).
  2. Implemented dynamic page-chaining for pointer stability: if allocations exceed the current page, new pages are allocated without reallocating existing data.
  3. Added `mark()` and `reset_to(marker)` for nested scratch unwinding, as well as `ScopedArenaMarker` RAII guard.
  4. Added `reset()` which retains pre-allocated page buffers across ticks, guaranteeing zero-malloc execution in subsequent simulation ticks.
  5. Implemented C++20 standard conforming `ArenaAllocator<T>` with allocator traits and rebind support for use with `std::vector<T, ArenaAllocator<T>>`.
- **Validation:** Added comprehensive test suite in `tests/core_tests.cpp` verifying alignment, marker rollback, chained expansion, page retention, and STL vector operations.

---

## [2026-11-09] - Chase-Lev Lock-Free Work-Stealing Queue & 2-Tier Worker Exponential Backoff

- **Subsystem:** Concurrency & Job System (`core/work_stealing_queue.hpp`, `core/job_system.hpp`)
- **Problem:** Benchmark revealed severe throughput regression when scaling worker counts (Medium workload regressed to 0.66x at 16 workers; LargeBulk dropped from 3.08x at 8 workers to 2.07x at 16 workers).
- **Root Cause:**
  1. Thread descheduling latency: worker threads parked immediately on `condition_variable::wait()`. Waking threads via OS scheduler introduced 15-30 µs latency, causing callers to complete work before workers even woke up.
  2. Full-spin contention: naive busy-spinning saturated SMT hyperthreads and ALU resources on shared physical cores.
- **Solution:**
  1. Implemented `WorkStealingQueue<T>` with Chase-Lev lock-free algorithm featuring 64-byte cache-line separated `top_` and `bottom_` atomics to eliminate false sharing.
  2. Implemented 2-tier exponential backoff in worker loop: first 32 iterations execute `_mm_pause()` in user space to catch immediate back-to-back tasks with zero OS context switches, next 32 iterations yield the thread, and only then does the worker sleep on the condition variable.
- **Validation:**
  - Medium workload at 16 workers increased from 0.66x regression to **2.23x speedup** (0.0076 ms vs 0.0262 ms — **3.38x throughput improvement**).
  - LargeBulk workload at 16 workers achieved **5.61x speedup** (0.0752 ms vs 0.4218 ms — **2.50x speedup** compared to previous 16-worker performance).
  - All 15 CTest suites passed in 1.62s.

---

## [2026-11-09] - Dynamic AABB Tree with Surface Area Heuristic (SAH) & Fat Bounding Boxes

- **Subsystem:** Spatial Indexing (`spatial/dynamic_aabb_tree.hpp`)
- **Problem:** `LinearBVH` requires an $O(N \log N)$ rebuild from scratch whenever objects move, and `SpatialHash` struggles with non-uniform primitive scales or sparse clusters.
- **Solution:**
  1. Implemented self-balancing binary bounding box tree (`DynamicAABBTree`).
  2. Applied Surface Area Heuristic (SAH) to greedily choose optimal insertion siblings.
  3. Implemented AVL-style tree rotations to preserve $O(\log N)$ tree depth across dynamic insertions and removals.
  4. Implemented Fat-AABB margins and velocity prediction: moving entities within their fat boundary require ZERO tree reinsertions ($O(1)$ fast path).
  5. Implemented non-recursive stack-based AABB overlap queries, raycasts, and self-collision pair generation (`query_pairs`).
  6. Memory pooling with free-list recycling to eliminate runtime allocations during updates.
- **Validation:** Added unit tests in `tests/aabb_tests.cpp` validating SAH insertion, removal, fat-AABB fast path, raycasts, broadphase pair overlaps, and bounded tree height ($\le 20$ for 500 entities).

---

## [2026-11-09] - Simulated Network Adversity Test Harness & Rollback Hardening

- **Subsystem:** Networking & Rollback (`network/reliability.hpp`, `network/rollback.hpp`)
- **Problem:** Rollback and client-side prediction were only validated under clean synchronous conditions, leaving packet drop, out-of-order arrival, and jitter untested.
- **Solution:**
  1. Built simulated network adversity test harness with 20% packet drop rate, 1-4 tick latency jitter, and out-of-order packet delivery.
  2. Verified `AckWindow` sequence number tracking under reordering, duplicate packets, and large sequence forward jumps (> 32).
  3. Integrated client-server test with authoritative server step and speculative client rollback controller.
- **Validation:** Over 60 ticks of simulated network adversity with 50 rollbacks executed (up to 5 frames rewound per rollback), the client simulation achieved bit-exact convergence with the authoritative server state.

---

## [2026-11-09] - SIMD Ray-AABB 8-Box Vectorized Intersection Tests

- **Subsystem:** Physics & Spatial Acceleration (`physics/simd_ray_intersection.hpp`)
- **Problem:** Testing single rays against numerous bounding boxes in perception cones, LiDAR sweeps, or BVH traversals executes repetitive scalar branch checks.
- **Solution:**
  1. Implemented Structure of Arrays (SoA) layout `AABB8` and `AABB4` for 8-wide and 4-wide bounding box processing.
  2. Implemented 8-wide AVX2 (`__m256`) vectorized slab intersection: broadcasts ray origin and inverse direction, computing all 6 slab planes simultaneously with `_mm256_min_ps` and `_mm256_max_ps`, generating an 8-bit hit mask via `_mm256_movemask_ps` with zero branching.
  3. Implemented 4-wide SSE (`__m128`) fallback and portable scalar fallback.
  4. Auto-dispatched dynamically via runtime CPUID feature detection (`CPUFeatures::has_avx2()`).
- **Validation:** Added differential tests in `tests/physics_tests.cpp` validating exact hit mask agreement and $< 10^{-4}$ hit distance equivalence across scalar, SSE, and AVX2 kernels.
