# Scandium Benchmark History & Telemetry Matrix

## Measurement Environment
- **Host CPU:** AMD/Intel x64 Architecture (16 Logical Processors)
- **Compiler:** Microsoft Visual C++ 19.51.36260 (x64) with `/std:c++20`
- **Build Configuration:** CMake 4.3.1 / Ninja 1.13.2 / Release (`/O2 /Ob2 /DNDEBUG`)
- **Measurement Method:** Warmup (2 trials), Measured (5 trials), Reporting median and percentiles with `do_not_optimize` memory barriers.

---

## 1. Spatial Broadphase: Naive vs Spatial Hash
| Entity Count | Naive All-Pairs (ms) | Spatial Hash (ms) | Speedup vs Naive | Selected Strategy |
|:---:|:---:|:---:|:---:|:---:|
| 100 | 0.0271 ms | 0.0551 ms | 0.49x (Naive faster) | Naive Direct |
| 250 | 0.1592 ms | 0.0658 ms | **2.42x faster** | Spatial Hash |
| 500 | 0.6528 ms | 0.1444 ms | **4.52x faster** | Spatial Hash |
| 1,000 | 2.9518 ms | 0.2075 ms | **14.22x faster** | Spatial Hash |
| 2,000 | 13.9855 ms | 1.3873 ms | **10.08x faster** | Spatial Hash |

*Crossover Threshold:* $N = 150$ entities.

---

## 2. Spatial Hash Query Throughput (50,000 Queries)
| Entity Population | Insert Median (ms) | Query Median (ms) | Total (ms) | Throughput (K QPS) |
|:---:|:---:|:---:|:---:|:---:|
| 1,000 | 0.0127 ms | 10.8673 ms | 10.8800 ms | **4,601.0 K QPS** |
| 5,000 | 0.1179 ms | 12.2424 ms | 12.3603 ms | **4,084.2 K QPS** |
| 10,000 | 0.2243 ms | 15.6945 ms | 15.9188 ms | **3,185.8 K QPS** |
| 25,000 | 0.4635 ms | 28.9912 ms | 29.4547 ms | **1,724.7 K QPS** |

---

## 3. Navigation: A* vs Jump Point Search (100 Repetitions per Topology)
| Topology | Grid Size | A* (ms) | JPS (ms) | Optimal Solver | Explored Nodes (A* / JPS) | Path Cost Match |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| Open Grid | 128 x 128 | 0.8704 ms | **0.6721 ms** | **JPS (1.30x faster)** | 3,864 / **4** (-99.9%) | **true (246.0)** |
| Dense Labyrinth | 64 x 64 | 0.7675 ms | **0.5328 ms** | **JPS (1.44x faster)** | 3,048 / **280** (-90.8%) | **true (122.0)** |
| Random Obstacles (20%) | 128 x 128 | 1.6678 ms | **1.2735 ms** | **JPS (1.31x faster)** | 6,985 / **2,253** (-67.7%) | **true (246.0)** |
| Sparse Obstacles (5%) | 128 x 128 | 0.7822 ms | **0.6932 ms** | **JPS (1.13x faster)** | 3,821 / **1,824** (-52.3%) | **true (246.0)** |
| Corridors | 96 x 96 | **0.7696 ms** | 2.0067 ms | **A\* (2.61x faster)** | 3,688 / **210** (-94.3%) | **true (186.0)** |

---

## 4. Concurrency: Job System Throughput & Adaptive Scaling
### LargeBulk Workload (100,000 Tasks)
| Worker Count | Median Execution (ms) | Speedup vs 1 Worker |
|:---:|:---:|:---:|
| 1 Worker | 0.4218 ms | 1.00x |
| 2 Workers | 0.2710 ms | 1.56x |
| 4 Workers | 0.1306 ms | 3.23x |
| 8 Workers | 0.0890 ms | 4.74x |
| 16 Workers | **0.0752 ms** | **5.61x** |

### Medium Workload (5,000 Tasks)
| Worker Count | Median Execution (ms) | Speedup vs 1 Worker |
|:---:|:---:|:---:|
| 1 Worker | 0.0169 ms | 1.00x |
| 2 Workers | 0.0112 ms | 1.51x |
| 4 Workers | 0.0088 ms | 1.91x |
| 8 Workers | 0.0077 ms | 2.18x |
| 16 Workers | **0.0076 ms** | **2.23x** |

### SmallMicro Workload (500 Tasks)
Adaptive threshold retains zero-overhead execution: 0.0017 ms (1.00x) across all worker counts.

---

## 5. Multi-Agent Simulation Scaling (120 Frames)
| Agent Population | 1 Worker | 2 Workers | 4 Workers | 8 Workers | 16 Workers | Max Speedup | Average Tick Duration |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| 500 | 7.21 ms | 5.62 ms | 6.06 ms | 5.72 ms | 5.82 ms | 1.28x | 46.9 µs |
| 1,000 | 11.75 ms | 12.64 ms | 11.93 ms | 12.16 ms | 13.12 ms | 1.00x | 97.9 µs |
| 5,000 | 63.31 ms | 47.91 ms | 54.85 ms | 38.08 ms | 42.71 ms | 1.66x | 317.3 µs |
| 10,000 | 127.99 ms | 92.89 ms | 85.24 ms | 78.85 ms | 85.55 ms | 1.62x | 657.1 µs |
| 20,000 | 277.77 ms | 193.87 ms | 129.93 ms | 148.99 ms | 144.37 ms | **2.14x** | **1,082.7 µs (1.08 ms)** |
