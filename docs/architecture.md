# Scandium Architecture

Scandium is being built as a small, measurable game-systems laboratory rather than a content-heavy game.

## Design goals

1. **Data structures first** — core systems should expose clear complexity characteristics.
2. **Measurement first** — performance claims must be backed by benchmarks or profiling data.
3. **Small interfaces** — systems should remain independently testable.
4. **Modern C++** — use C++20 features where they improve correctness or expressiveness.
5. **Agent-friendly development** — repository structure, tests, and benchmarks should make automated code changes verifiable.

## Planned layers

```
Application
    |
Gameplay / Simulation
    |
AI / Navigation ---- Physics / Collision
    |                    |
    +-------- Math ------+
             |
        Core utilities
```

The first implementation is the math layer. Navigation, collision, simulation, profiling, and live-service components will build on these primitives.
