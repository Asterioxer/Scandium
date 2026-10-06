# Scandium Architecture

Scandium is a small, measurable game-systems laboratory evolving toward a deterministic simulation runtime and live-service prototype.

## Runtime layers

```
Application / Demo
       |
Fixed-Step Runtime
       |
Gameplay / Simulation
       |
+------+----------------+
|                       |
AI / Navigation     Physics / Collision
|                       |
+-----------+-----------+
            |
          Math
            |
      Core / ECS / Pool
```

The fixed-step runtime separates simulation time from render time. A frame may execute zero or multiple fixed simulation ticks, bounded by a catch-up limit. The resulting interpolation alpha is reserved for future render interpolation.

## Core systems

- **Math:** Vec3, Mat4, transforms, quaternions, projection, rays.
- **Physics:** AABB and ray intersection primitives.
- **Spatial:** grid-cell spatial hashing for neighborhood queries.
- **Navigation:** A* plus a 4-connected Jump Point Search implementation.
- **AI:** steering behaviors and multi-agent simulation.
- **Core:** object pooling with constant-time average release lookup.
- **ECS:** entity lifecycle and transform storage.
- **Profiling:** frame timing, allocation accounting, deterministic collision benchmarks.
- **Runtime:** fixed-step simulation clock and simulation statistics.

## Live service

The FastAPI service supports players, scores, matches, and leaderboards.

Storage modes:

1. **Memory mode** — default for tests and zero-setup development.
2. **PostgreSQL mode** — enabled with `SCANDIUM_DATABASE_URL`.

The PostgreSQL Docker stack is defined in `services/game_api/docker-compose.yml`.

## Engineering contract

Every performance or correctness claim should be backed by one of:

- an automated test,
- a reproducible benchmark,
- or an explicit runtime invariant.

The project intentionally keeps rendering separate from simulation so a future renderer can consume state without becoming part of the simulation's correctness boundary.


## Flagship runtime

The runtime facade is intentionally thin: it coordinates systems without owning their implementation details.

- JobSystem executes independent CPU work.
- EventBus provides deterministic synchronous system notifications.
- SimulationClock defines fixed-step time.
- Replay stores input commands by simulation tick.
- State hashing produces a compact regression fingerprint.
- RenderSnapshot exposes interpolated state to any renderer.
- AckWindow provides the sequence/ack primitive needed by a future authoritative transport.
- TraceCollector exports Chrome trace-compatible timing data.

The architecture is renderer-agnostic and transport-agnostic while exposing the contracts needed by a real multiplayer game.


## Multiplayer contract

The networking layer is intentionally split into transport and simulation contracts. `AckWindow` tracks packet sequencing and acknowledgement state; `LockstepSession` buffers per-peer input by simulation tick and exposes a deterministic ready/consume boundary. Socket I/O is deliberately outside the core library, so transport can later be UDP, a relay, or another platform service without changing simulation code.

## Gameplay layer

`game::Arena` demonstrates how engine primitives become an actual game loop: deterministic target selection, movement, attack cooldowns, damage events, elimination events, team scoring, and bounded arena movement. The arena is independently testable and can later be driven by the runtime clock or a network lockstep session.
