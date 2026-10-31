# Scandium Empirical Performance & Benchmark Report

This document records the empirical measurements, optimization methodologies, and scaling characteristics of the Scandium simulation engine.

## Test Environment
- **Host CPU:** AMD/Intel x64 Architecture (16 Logical Processors)
- **Compiler:** Microsoft Visual C++ 19.51.36260 (x64) with C++20 Standard
- **Build Configuration:** CMake 4.3.1 / Ninja 1.13.2 / Release Mode (`/O2 /Ob2 /DNDEBUG`)
- **Simulation Clock:** Strict 60 Hz Fixed-Step (`16.667 ms`)
- **Measurement Methodology:** High-resolution monotonic clock (`std::chrono::steady_clock`), minimum 2 warmup passes, 5 repeated trials reporting median durations, compiler dead-code elimination barriers (`do_not_optimize`).

---

## 1. Spatial Hash Collision vs. Naive Broadphase

Comparison of broadphase collision candidate detection across increasing entity counts.

| Entity Count | Naive Baseline (ms) | Spatial Hash (ms) | Speedup vs Naive | Selected Strategy |
|---|---|---|---|---|
| **100** | **0.0158 ms** | 0.0497 ms | 0.32x (Naive faster) | Naive Direct |
| **250** | 0.1591 ms | **0.0733 ms** | **2.17x faster** | Spatial Hash |
| **500** | 0.5574 ms | **0.0870 ms** | **6.41x faster** | Spatial Hash |
| **1,000** | 1.4038 ms | **0.1683 ms** | **8.34x faster** | Spatial Hash |
| **2,000** | 5.9267 ms | **0.4476 ms** | **13.24x faster** | Spatial Hash |

### Small-$N$ Crossover & Adaptive Broadphase
- **Small-$N$ Crossover (Threshold $\approx 150$):** Below 150 entities, all-pairs distance checks operate directly within L1 cache with zero hash setup or clearing overhead.
- **Sparse Clear Optimization:** For small entity counts, `SpatialHash::clear()` selectively resets only touched bucket indices, reducing memory write overhead by 80x compared to wiping the entire 8,192-entry table.
- **AdaptiveBroadphase:** Automatically selects Naive broadphase when $N < 150$ and SpatialHash when $N \ge 150$.

---

## 2. Rigorous Spatial Hash Throughput (50,000 Queries)

| Entities | Queries | Insert Median (ms) | Query Median (ms) | Total Workload (ms) | Throughput (k QPS) |
|---|---|---|---|---|---|
| **1,000** | 50,000 | 0.0147 ms | 9.2784 ms | 9.2931 ms | **5,388.9 kQPS** |
| **5,000** | 50,000 | 0.1621 ms | 11.0990 ms | 11.2611 ms | **4,504.9 kQPS** |
| **10,000** | 50,000 | 0.3164 ms | 14.8771 ms | 15.1935 ms | **3,360.9 kQPS** |
| **25,000** | 50,000 | 0.7959 ms | 30.1081 ms | 30.9040 ms | **1,660.7 kQPS** |

---

## 3. Navigation Engine Rigorous Comparison (100 Paths, Median of 5 Trials)

| Topology | Grid Size | A* Median (ms) | JPS Median (ms) | Explored Nodes (A* / JPS) | Path Length | Costs Match |
|---|---|---|---|---|---|---|
| **Corridor Grid** | 96 x 96 | **19.50 ms** | 30.35 ms | 3,688 / 3,688 | 187 | **true** |
| **Open Grid** | 128 x 128 | **22.71 ms** | 24.32 ms | 3,864 / 3,864 | 247 | **true** |
| **Dense Labyrinth** | 64 x 64 | **16.91 ms** | 17.20 ms | 3,048 / 3,048 | 123 | **true** |

### JPS vs. A* Findings & Analysis
- **Pruning & Correctness:** Path costs and lengths match identically across all topologies.
- **4-Connected Scanning Overhead:** In 4-connected (cardinal) grids without diagonal jumps, JPS must perform orthogonal raycasts (`jump_vertical_check`) at each horizontal step.
- **Cache Locality:** A* with dense flat arrays and Manhattan heuristic achieves superior instruction-cache locality on cardinal grids, expanding directly towards the goal without recursive ray scans.
- **Optimization Impact:** Replacing heap `std::vector` in `pruned_directions` with stack-allocated `DirectionSet` reduced JPS runtime by 43% (from 53.7 ms down to 30.3 ms on corridors).

---

## 4. Job System Throughput & Adaptive Execution

| Workload Type | Size | Workers | Adaptive Threshold | Median Time (ms) | Speedup vs 1 Worker |
|---|---|---|---|---|---|
| **SmallMicro** | 500 | 1 | 1,024 | 0.0009 ms | 1.00x |
| **SmallMicro** | 500 | 16 | 1,024 | **0.0010 ms** | **0.94x (sequential fallback)** |
| **Medium** | 5,000 | 1 | 1,024 | 0.0107 ms | 1.00x |
| **Medium** | 5,000 | 4 | 1,024 | **0.0012 ms** | **9.26x speedup** |
| **Medium** | 5,000 | 8 | 1,024 | **0.0011 ms** | **9.38x speedup** |
| **LargeBulk** | 100,000 | 1 | 1,024 | 0.0670 ms | 1.00x |
| **LargeBulk** | 100,000 | 4 | 1,024 | **0.0401 ms** | **1.67x speedup** |
| **LargeBulk** | 100,000 | 8 | 1,024 | **0.0389 ms** | **1.72x speedup** |
| **LargeBulk** | 100,000 | 16 | 1,024 | **0.0408 ms** | **1.64x speedup** |

*Micro-workloads ($N \le 1,024$) bypass thread pool dispatch, eliminating the previous 0.048 ms thread-wake penalty.*

---

## 5. Multi-Agent Simulation Scale (120 Frames, Adaptive Threshold = 2,000)

| Agents | 1 Worker (Seq) | 2 Workers | 4 Workers | 8 Workers | 16 Workers | Speedup (vs Seq) | Average Tick |
|---|---|---|---|---|---|---|---|
| **500** | 1.30 ms | 1.28 ms | 1.38 ms | 1.37 ms | **1.49 ms** | 0.87x | **12.4 µs** |
| **1,000** | 2.88 ms | 2.57 ms | 2.60 ms | 2.60 ms | **2.71 ms** | 1.06x | **22.6 µs** |
| **5,000** | 12.99 ms | 13.32 ms | 12.45 ms | 16.25 ms | **14.84 ms** | 1.04x | **123.7 µs** |
| **10,000** | 28.39 ms | 23.35 ms | 26.94 ms | 25.01 ms | **24.33 ms** | **1.17x** | **202.8 µs** |
| **20,000** | 56.06 ms | 41.73 ms | **35.20 ms** | 40.13 ms | **35.35 ms** | **1.59x** | **293.3 µs (0.29 ms)** |

*With the adaptive parallel threshold (2,000 agents), low agent counts avoid thread pool dispatch thrashing (taking 1.49 ms instead of the earlier 8.22 ms penalty), while 20,000 agents simulate 120 frames in 35.2 ms (0.29 ms per tick).*
