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
