# Scandium Architecture Specification

## 1. Architectural Philosophy
Scandium is designed around four inviolable principles:
1. **Bit-Exact Determinism:** Given identical initial state and command stream, identical outputs are produced across thread configurations and executions.
2. **Data-Oriented Cache Locality:** Prefer contiguous memory buffers, structure-of-arrays (SoA) layouts, and monotonic stamps over linked lists and hash maps.
3. **Zero-Allocation Hot Paths:** Memory for ticks, queries, searches, and jobs is pre-reserved or thread-locally reused.
4. **Hardware-Aware Execution:** Adaptive thresholds switch algorithms based on dataset size and instruction capabilities.

---

## 2. Module Decomposition & Boundaries

```
+-------------------------------------------------------------------------+
|                        Scandium Simulation Engine                       |
+-------------------------------------------------------------------------+
       |                                                    |
       v                                                    v
[ Runtime & Events ]                               [ Simulation Layer ]
 - Engine Orchestrator                              - AgentWorld (Double Buffered)
 - Fixed-Step Clock (dt=16.67ms)                    - Determinism & FNV-1a Hashing
 - EventBus                                         - Replay Serialization
       |                                                    |
       +--------------------+-------------------------------+
                            |
                            v
       +-----------------------------------------+
       |             Systems & AI               |
       |  - Reynolds Flocking & Steering         |
       |  - Continuous Collision Detection (CCD) |
       |  - Behavior Trees & Blackboard          |
       |  - Sensory Perception (Vision/Sound)    |
       +-----------------------------------------+
                            |
       +--------------------+--------------------+
       |                                         |
       v                                         v
[ Spatial & Navigation ]                  [ Hardware & Concurrency ]
 - Flat-Bucket Spatial Hash                - Contention-Free Job System
 - Linear BVH (63-bit Morton Octree)       - Multi-Tier SIMD (AVX2/SSE/Scalar)
 - A* (4-Way & 8-Way Diagonal)             - State Ring Buffer Rollback Netcode
 - Canonical Jump Point Search (JPS)       - Batch GPU StructuredBuffer Pipeline
 - Dijkstra Flow Field Vector Maps         - Generational Sparse-Set ECS
```

---

## 3. Data Flow & Execution Model

### The Fixed-Step Simulation Tick
1. **Clock Accumulator:** Time accumulates via `Clock::tick()`. When $\text{accumulator} \ge dt$, a simulation frame is executed.
2. **Buffer Ping-Pong:** Workers read exclusively from `agents_` (tick $t$) and write outputs to `next_agents_` (tick $t+1$).
3. **Spatial Acceleration Build:** Spatial hash buckets are populated using contiguous entries with the SplitMix64 hash function.
4. **Concurrent Agent Update:** The job system chunks agent iterations across available worker threads (`parallel_for_chunked`).
5. **State Swap & Hash Verification:** Read/write buffers are swapped. 64-bit FNV-1a hash is computed and emitted to `SimulationTickEvent`.
