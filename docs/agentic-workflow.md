# Agentic Engineering Workflow

Scandium treats coding agents as engineering accelerators with verification gates.

Task loop: Issue -> repository inspection -> plan -> implementation -> build -> unit tests -> benchmark when performance-sensitive -> diff review -> human approval.

Rules:
1. Read relevant architecture and tests before editing.
2. Make the smallest coherent change.
3. Never remove a failing test to make a task green.
4. Add tests for behavior changes.
5. Run build and tests after implementation.
6. Establish a baseline before optimization.
7. Report assumptions and unresolved risks.
8. Do not merge solely because an agent reports success.

Example task: replace naive broad-phase collision checks with a spatial hash. The agent should inspect physics/spatial, add a deterministic benchmark, implement the broad phase, preserve correctness tests, compare results, explain complexity and memory trade-offs, and produce a reviewable diff.
