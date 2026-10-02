from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

app = FastAPI(title="Scandium Live Service API", version="0.1.0")

players: dict[int, "Player"] = {}
leaderboard: dict[int, int] = {}
matches: dict[int, "Match"] = {}

class PlayerCreate(BaseModel):
    name: str = Field(min_length=1, max_length=32)

class Player(PlayerCreate):
    id: int

class ScoreUpdate(BaseModel):
    score: int = Field(ge=0)

class MatchCreate(BaseModel):
    player_ids: list[int] = Field(min_length=1, max_length=8)

class Match(BaseModel):
    id: int
    player_ids: list[int]
    status: str = "created"

@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok", "service": "scandium-live-service"}

@app.post("/players", response_model=Player, status_code=201)
def create_player(payload: PlayerCreate) -> Player:
    player_id = max(players.keys(), default=0) + 1
    player = Player(id=player_id, name=payload.name)
    players[player_id] = player
    leaderboard.setdefault(player_id, 0)
    return player

@app.get("/players/{player_id}", response_model=Player)
def get_player(player_id: int) -> Player:
    player = players.get(player_id)
    if player is None:
        raise HTTPException(status_code=404, detail="player not found")
    return player

@app.post("/players/{player_id}/score")
def update_score(player_id: int, payload: ScoreUpdate) -> dict[str, int]:
    if player_id not in players:
        raise HTTPException(status_code=404, detail="player not found")
    leaderboard[player_id] = leaderboard.get(player_id, 0) + payload.score
    return {"player_id": player_id, "score": leaderboard[player_id]}

@app.get("/leaderboard")
def get_leaderboard() -> list[dict[str, int]]:
    ranked = sorted(leaderboard.items(), key=lambda item: item[1], reverse=True)
    return [{"rank": rank, "player_id": pid, "score": score} for rank, (pid, score) in enumerate(ranked, 1)]

@app.post("/matches", response_model=Match, status_code=201)
def create_match(payload: MatchCreate) -> Match:
    missing = [pid for pid in payload.player_ids if pid not in players]
    if missing:
        raise HTTPException(status_code=400, detail={"missing_player_ids": missing})
    match_id = max(matches.keys(), default=0) + 1
    match = Match(id=match_id, player_ids=payload.player_ids)
    matches[match_id] = match
    return match

@app.get("/matches/{match_id}", response_model=Match)
def get_match(match_id: int) -> Match:
    match = matches.get(match_id)
    if match is None:
        raise HTTPException(status_code=404, detail="match not found")
    return match
