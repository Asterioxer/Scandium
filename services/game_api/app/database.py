import os
from contextlib import contextmanager
from typing import Iterator

try:
    import psycopg
except ImportError:  # pragma: no cover
    psycopg = None


class Database:
    def __init__(self, url: str | None = None):
        self.url = url or os.getenv("SCANDIUM_DATABASE_URL")
        self.enabled = bool(self.url)

    @contextmanager
    def connection(self) -> Iterator:
        if not self.enabled or psycopg is None:
            yield None
            return
        with psycopg.connect(self.url) as connection:
            yield connection

    def initialize(self) -> None:
        if not self.enabled:
            return
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    """
                    CREATE TABLE IF NOT EXISTS players (
                        id BIGSERIAL PRIMARY KEY,
                        name VARCHAR(32) NOT NULL
                    );

                    CREATE TABLE IF NOT EXISTS scores (
                        player_id BIGINT PRIMARY KEY REFERENCES players(id) ON DELETE CASCADE,
                        score BIGINT NOT NULL DEFAULT 0
                    );

                    CREATE TABLE IF NOT EXISTS matches (
                        id BIGSERIAL PRIMARY KEY,
                        status VARCHAR(32) NOT NULL DEFAULT 'created'
                    );

                    CREATE TABLE IF NOT EXISTS match_players (
                        match_id BIGINT REFERENCES matches(id) ON DELETE CASCADE,
                        player_id BIGINT REFERENCES players(id) ON DELETE CASCADE,
                        PRIMARY KEY (match_id, player_id)
                    );
                    """
                )

    def create_player(self, name: str) -> dict | None:
        if not self.enabled:
            return None
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    "INSERT INTO players (name) VALUES (%s) RETURNING id, name",
                    (name,),
                )
                player_id, player_name = cursor.fetchone()
                cursor.execute(
                    "INSERT INTO scores (player_id, score) VALUES (%s, 0)",
                    (player_id,),
                )
                return {"id": player_id, "name": player_name}

    def get_player(self, player_id: int) -> dict | None:
        if not self.enabled:
            return None
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    "SELECT id, name FROM players WHERE id = %s",
                    (player_id,),
                )
                row = cursor.fetchone()
                return None if row is None else {"id": row[0], "name": row[1]}

    def add_score(self, player_id: int, score: int) -> int | None:
        if not self.enabled:
            return None
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    """
                    UPDATE scores
                    SET score = score + %s
                    WHERE player_id = %s
                    RETURNING score
                    """,
                    (score, player_id),
                )
                row = cursor.fetchone()
                return None if row is None else row[0]

    def leaderboard(self) -> list[dict[str, int]] | None:
        if not self.enabled:
            return None
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    """
                    SELECT player_id, score
                    FROM scores
                    ORDER BY score DESC, player_id ASC
                    """
                )
                return [
                    {"rank": rank, "player_id": row[0], "score": row[1]}
                    for rank, row in enumerate(cursor.fetchall(), 1)
                ]

    def create_match(self, player_ids: list[int]) -> int | None:
        if not self.enabled:
            return None
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    "INSERT INTO matches DEFAULT VALUES RETURNING id"
                )
                match_id = cursor.fetchone()[0]
                cursor.executemany(
                    "INSERT INTO match_players (match_id, player_id) VALUES (%s, %s)",
                    [(match_id, player_id) for player_id in player_ids],
                )
                return match_id

    def get_match(self, match_id: int) -> dict | None:
        if not self.enabled:
            return None
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    """
                    SELECT m.id, m.status, mp.player_id
                    FROM matches m
                    LEFT JOIN match_players mp ON mp.match_id = m.id
                    WHERE m.id = %s
                    ORDER BY mp.player_id
                    """,
                    (match_id,),
                )
                rows = cursor.fetchall()
                if not rows:
                    return None
                return {
                    "id": rows[0][0],
                    "status": rows[0][1],
                    "player_ids": [row[2] for row in rows if row[2] is not None],
                }

    def players_exist(self, player_ids: list[int]) -> bool:
        if not self.enabled:
            return True
        with self.connection() as connection:
            with connection.cursor() as cursor:
                cursor.execute(
                    "SELECT COUNT(*) FROM players WHERE id = ANY(%s)",
                    (player_ids,),
                )
                return cursor.fetchone()[0] == len(set(player_ids))
