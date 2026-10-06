# Performance Methodology

Scandium treats performance as an engineering claim that must be reproducible.

## Measurement rules

For every optimization:

1. Establish a baseline.
2. Use deterministic input.
3. Run enough iterations to reduce measurement noise.
4. Report median and percentile latency where practical.
5. Explain the algorithmic or data-layout change.
6. Keep correctness tests separate from performance tests.

## Initial targets

The first performance experiment will compare naive pairwise AABB collision detection against a spatial hash.

Naive broad phase:

- Candidate checks: O(n²)

Spatial hash target:

- Candidate generation should approach O(n + k), where k is the number of nearby candidates.

The benchmark will vary entity count and report collision-check time so the optimization is visible rather than asserted.

## Multicore simulation scaling

The flagship benchmark compares identical 10,000-agent workloads with one worker and the automatically sized worker pool:

    cmake --build build --config Release
    build/Release/scandium_simulation_scale

The output reports total simulation time for 120 fixed-step frames and the resulting scaling ratio. Record the result on the target machine before making performance claims; the benchmark is intentionally workload- and hardware-dependent.

## Trace profiling

Use `scandium::profiling::TraceCollector` and `TraceScope` around systems that need investigation. `to_chrome_json()` produces Chrome trace-compatible JSON that can be loaded into a trace viewer.

## Performance invariants

- Fixed-step simulation bounds simulation work per frame.
- Spatial hashing is measured against a naive collision baseline.
- Job-system parallelism is only used for independent agent updates.
- Rendering is separated from simulation through interpolated snapshots.
- Benchmarks are deterministic in workload generation even when wall-clock measurements vary by hardware.
