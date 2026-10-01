# Contributing to Scandium

## Development loop

1. Make one focused change.
2. Build with CMake.
3. Run the test suite.
4. Benchmark before/after when changing performance-sensitive code.
5. Document non-obvious engineering decisions.

## Code standards

- C++20
- Warnings enabled
- Prefer value semantics for small math types.
- Avoid premature abstraction.
- Keep hot-path operations allocation-free where practical.
- Every new subsystem should have tests.

## Performance rule

Do not claim an optimization without a reproducible measurement.
