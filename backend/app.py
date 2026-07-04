"""FastAPI app: serves the C++ tracker's snapshots over HTTP + WebSocket.

Run from the repo root:

    uvicorn backend.app:app --port 8000

Configuration (environment variables):
    ADSB_STREAM_EXE  path to adsb_stream (default: cpp/build/adsb_stream[.exe])
    ADSB_ASSOC       tracker association mode: id | nn | hungarian (default id)
    ADSB_SOURCE      replay | live                    (default replay)
    ADSB_SESSION     session CSV for replay mode      (default data/session_sim.csv)
    ADSB_SPEED       replay speed multiplier          (default 10)
    ADSB_CENTER      "lat,lon" for live mode          (default 40.0,-83.0)
    ADSB_RADIUS_NM   live query radius                (default 100)

Endpoints:
    GET /tracks   latest snapshot (tracks + stats) as JSON
    GET /healthz  liveness of the API and the tracker subprocess
    WS  /ws       latest snapshot on connect, then every new snapshot
"""
from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from .bridge import TrackerBridge
from .feeds import start_live_feeder, start_replay_feeder

EMPTY_SNAPSHOT = {
    "time": 0.0,
    "tracks": [],
    "stats": {"measurements": 0, "tracks_created": 0, "stale_removed": 0,
              "active": 0},
}


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _default_exe() -> str:
    for name in ("adsb_stream.exe", "adsb_stream"):
        candidate = _repo_root() / "cpp" / "build" / name
        if candidate.exists():
            return str(candidate)
    return str(_repo_root() / "cpp" / "build" / "adsb_stream.exe")


class Broadcaster:
    """Fan out snapshots to WebSocket client queues (event-loop thread only)."""

    def __init__(self) -> None:
        self._clients: Set[asyncio.Queue] = set()

    def register(self, q: asyncio.Queue) -> None:
        self._clients.add(q)

    def unregister(self, q: asyncio.Queue) -> None:
        self._clients.discard(q)

    def publish(self, snapshot: dict) -> None:
        for q in self._clients:
            if q.full():
                try:
                    q.get_nowait()  # drop the oldest for a slow client
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(snapshot)


def create_app(bridge: Optional[TrackerBridge] = None,
               start_feeder: bool = True) -> FastAPI:
    """App factory. Tests inject a fake ``bridge`` and disable the feeder."""

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        loop = asyncio.get_running_loop()
        broadcaster = Broadcaster()

        active_bridge = bridge
        if active_bridge is None:
            active_bridge = TrackerBridge(
                exe=os.environ.get("ADSB_STREAM_EXE", _default_exe()),
                assoc=os.environ.get("ADSB_ASSOC", "id"))
        active_bridge.on_snapshot = (
            lambda snap: loop.call_soon_threadsafe(broadcaster.publish, snap))

        if start_feeder:
            if os.environ.get("ADSB_SOURCE", "replay") == "live":
                lat_s, lon_s = os.environ.get("ADSB_CENTER",
                                              "40.0,-83.0").split(",")
                start_live_feeder(
                    active_bridge, lat=float(lat_s), lon=float(lon_s),
                    radius_nm=float(os.environ.get("ADSB_RADIUS_NM", "100")))
            else:
                session = os.environ.get(
                    "ADSB_SESSION", str(_repo_root() / "data" /
                                        "session_sim.csv"))
                start_replay_feeder(
                    active_bridge, session,
                    speed=float(os.environ.get("ADSB_SPEED", "10")))

        app.state.bridge = active_bridge
        app.state.broadcaster = broadcaster
        yield
        active_bridge.close()

    app = FastAPI(title="adsb-tracker backend", lifespan=lifespan)

    @app.get("/tracks")
    def get_tracks():
        return app.state.bridge.latest() or EMPTY_SNAPSHOT

    @app.get("/healthz")
    def healthz():
        return {"status": "ok", "tracker_alive": app.state.bridge.alive()}

    @app.websocket("/ws")
    async def ws_tracks(ws: WebSocket):
        await ws.accept()
        latest = ws.app.state.bridge.latest()
        await ws.send_json(latest or EMPTY_SNAPSHOT)

        q: asyncio.Queue = asyncio.Queue(maxsize=16)
        ws.app.state.broadcaster.register(q)
        try:
            while True:
                snapshot = await q.get()
                await ws.send_json(snapshot)
        except WebSocketDisconnect:
            pass
        finally:
            ws.app.state.broadcaster.unregister(q)

    return app


app = create_app()
