from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_health():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "ok"

def test_player_score_and_leaderboard():
    player = client.post("/players", json={"name": "Scout"}).json()
    player_id = player["id"]
    score = client.post(f"/players/{player_id}/score", json={"score": 125})
    assert score.status_code == 200
    assert score.json()["score"] == 125
    leaderboard = client.get("/leaderboard")
    assert leaderboard.json()[0]["player_id"] == player_id

def test_match_requires_existing_players():
    response = client.post("/matches", json={"player_ids": [999999]})
    assert response.status_code == 400
