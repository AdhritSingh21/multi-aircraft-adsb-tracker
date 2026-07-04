"""API tests with an injected fake bridge (no subprocess, no network)."""
from fastapi.testclient import TestClient

from backend.app import EMPTY_SNAPSHOT, create_app

SNAPSHOT = {
    "time": 42.0,
    "tracks": [{"id": 1, "icao": "aaa111", "lat": 40.1, "lon": -82.9,
                "alt_m": 10500.0, "speed_mps": 220.0, "heading_deg": 90.0,
                "age_s": 1.0, "hits": 5, "trail": [[40.1, -82.9]]}],
    "stats": {"measurements": 5, "tracks_created": 1, "stale_removed": 0,
              "active": 1},
}


class FakeBridge:
    def __init__(self, snapshot=None):
        self._snapshot = snapshot
        self.on_snapshot = None
        self.closed = False

    def latest(self):
        return self._snapshot

    def alive(self):
        return not self.closed

    def close(self):
        self.closed = True


def make_client(snapshot=None):
    return TestClient(create_app(bridge=FakeBridge(snapshot),
                                 start_feeder=False))


def test_healthz_reports_tracker_liveness():
    with make_client() as client:
        r = client.get("/healthz")
        assert r.status_code == 200
        assert r.json() == {"status": "ok", "tracker_alive": True}


def test_tracks_returns_latest_snapshot():
    with make_client(SNAPSHOT) as client:
        r = client.get("/tracks")
        assert r.status_code == 200
        assert r.json() == SNAPSHOT


def test_tracks_empty_before_first_snapshot():
    with make_client(None) as client:
        assert client.get("/tracks").json() == EMPTY_SNAPSHOT


def test_websocket_sends_latest_on_connect():
    with make_client(SNAPSHOT) as client:
        with client.websocket_connect("/ws") as ws:
            assert ws.receive_json() == SNAPSHOT


def test_bridge_closed_on_shutdown():
    fake = FakeBridge(SNAPSHOT)
    with TestClient(create_app(bridge=fake, start_feeder=False)) as client:
        client.get("/healthz")
    assert fake.closed
