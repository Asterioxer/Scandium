# Scandium Live Service

Small REST service representing the backend side of a live-service game.

Endpoints: GET /health, POST /players, GET /players/{id}, POST /players/{id}/score, GET /leaderboard, POST /matches, GET /matches/{id}.

Run locally with `uvicorn app.main:app --reload` from this directory. Run tests with `pytest`.

The first implementation is intentionally in-memory; PostgreSQL persistence is a later hardening milestone.


## PostgreSQL mode

Set `SCANDIUM_DATABASE_URL` to enable persistent storage:

    SCANDIUM_DATABASE_URL=postgresql://scandium:scandium@localhost:5432/scandium

For a complete local stack:

    docker compose -f docker-compose.yml up --build

Without the variable, the service uses in-memory storage, which keeps unit tests and lightweight local development dependency-free.
