# Scandium Live Service

Small REST service representing the backend side of a live-service game.

Endpoints: GET /health, POST /players, GET /players/{id}, POST /players/{id}/score, GET /leaderboard, POST /matches, GET /matches/{id}.

Run locally with `uvicorn app.main:app --reload` from this directory. Run tests with `pytest`.

The first implementation is intentionally in-memory; PostgreSQL persistence is a later hardening milestone.
