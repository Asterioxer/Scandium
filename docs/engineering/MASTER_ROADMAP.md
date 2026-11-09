# Scandium Engineering Master Roadmap

## Vision
Transform Scandium into an exceptionally reliable, bit-exact deterministic, scalable, modular C++20 simulation and systems-engineering platform capable of orchestrating multi-million agent environments across heterogeneous CPU/GPU targets.

---

## Autonomous Milestone Tracking

### Milestone 1: Navigation Subsystem Revolution & Graph Pruning
- [x] **Audit & Fix JPS (Jump Point Search):** Resolved asymmetric lookahead bug and open-set direction pruning loss. Removed silent fallback to A*.
- [x] **Differential Path Validation:** Verified bit-exact path costs across 5 standard topologies (OpenGrid, Corridors, DenseLabyrinth, Random20%, Sparse5%).
- [x] **8-Way JPS & Corner-Cutting Prevention:** Canonical 8-way diagonal jump search with Octile heuristics.
- [x] **Hierarchical Pathfinding (HPA*):** Implement coarse-grain cluster abstraction for maps exceeding 1,024 x 1,024 to enable sub-millisecond long-distance routing.

### Milestone 2: Concurrency & Lock-Free Scheduling
- [x] **Zero-Contention Spin-Wait:** Introduced atomic queue tracking `unpopped_jobs_` and `_mm_pause()` in worker synchronization.
- [x] **Scaling Verification:** Achieved 5.61x speedup on 16 threads for bulk 100K task workloads.
- [x] **Chase-Lev Work-Stealing Deque:** Implement per-worker lock-free double-ended queues for work-stealing scheduling with minimal CAS overhead.
- [x] **2-Tier Exponential Backoff:** Eliminated OS thread descheduling latency, achieving 2.23x speedup on Medium (5,000 tasks) workloads at 16 workers.
- [ ] **Task Graph Dependencies:** Directed acyclic graph (DAG) job execution with atomic predecessor counters.

### Milestone 3: Memory Systems & Frame Allocator
- [x] **Generational ECS Safety:** Stale handle rejection and wraparound guards (`generation == 0` reserved).
- [x] **Linear Frame Allocator:** Monotonic bump allocator for transient per-tick simulation scratch memory, eliminating system malloc in inner loops.
- [x] **STL Arena Allocator:** Standard-compliant `ArenaAllocator<T>` for vector and container allocations backed by bump frames.
- [ ] **Chunked Archetype Storage:** Evaluate archetype chunk storage for homogeneous batch iteration versus sparse-set overhead.

### Milestone 4: Spatial Intelligence & Dynamic Trees
- [x] **Spatial Hash Sparse Clear:** 80x reduction in clear overhead for sub-thousand entity counts.
- [x] **Linear BVH (Morton Codes):** 63-bit Z-order curve spatial indexing with non-recursive stack traversal.
- [x] **Dynamic AABB Tree:** Self-balancing bounding box tree with Surface Area Heuristic (SAH), AVL rotations, and fat-AABB margin caching for moving geometry.
- [ ] **Dynamic Resizing Spatial Hash:** Load-factor adaptive bucket resizing to preserve >1.5M QPS under adversarial concentration.

### Milestone 5: CPU SIMD Acceleration & Hardware Dispatch
- [x] **Multi-Tier SIMD Steering:** 8-wide AVX2 (`__m256`) and 4-wide SSE (`__m128`) with portable scalar fallback.
- [x] **Runtime CPUID Capability Detection:** Dynamic CPU feature dispatch without requiring explicit compiler flags (`/arch:AVX2`).
- [x] **SIMD Ray-AABB Intersection:** 8-box vectorized AVX2 (`__m256`) slab intersection test with 4-wide SSE fallback and CPUID auto-dispatch.

### Milestone 6: Native GPU Compute Backend
- [x] **HLSL Compute Shader Generation:** Production compute shader string export and 16-byte aligned structured buffers.
- [x] **Wavefront CPU Emulator:** 64-thread workgroup emulation.
- [ ] **Vulkan Compute Backend (Headless):** Live Vulkan compute pipeline creation (`VkComputePipelineCreateInfo`), device buffer allocation, descriptor sets, and command buffer dispatch for GPU-resident multi-agent simulation.

### Milestone 7: Networking & Rollback Hardening
- [x] **Circular State Ring Buffer:** $O(1)$ bitwise indexed state history.
- [x] **Input Prediction & Resimulation:** Authoritative packet mismatch detection and forward rollback.
- [x] **Simulated Network Adversity Harness:** Packet loss (20%), out-of-order delivery, jitter, and automated resimulation verification with bit-exact convergence.

---

## Active Execution Objective
**Objective 7:** Implement SIMD Ray-AABB 4-Ray / 8-Box Vector Intersection Tests (`include/scandium/physics/simd_ray_intersection.hpp`) to accelerate spatial queries, raycasts, and perception field-of-view sweeps across dense agent clusters.
