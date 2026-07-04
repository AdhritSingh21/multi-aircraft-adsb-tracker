"""Feeders: push measurements into the TrackerBridge on a daemon thread.

Replay mode reuses ingest.replayer.replay() — the same repeatable-demo path
introduced in Milestone 2 — so a recorded session drives the tracker with
original (or speed-scaled) timing. Live mode reuses the M2 readsb client.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from ingest.net import IngestError
from ingest.readsb import PROVIDERS, fetch_point, normalize_readsb
from ingest.replayer import replay

from .bridge import BridgeError, TrackerBridge


def start_replay_feeder(bridge: TrackerBridge, session_path: str,
                        speed: float = 10.0,
                        tick_interval_s: float = 0.5) -> threading.Thread:
    """Replay a session file into the bridge. Ticks (snapshot requests) are
    rate-limited to at most one per ``tick_interval_s`` of wall time so a
    dense session doesn't emit hundreds of snapshots per second."""

    def run() -> None:
        state = {"last_tick_wall": 0.0, "last_ts": 0.0}

        def emit(m) -> None:
            bridge.feed(m.to_csv_row())
            state["last_ts"] = m.timestamp
            now = time.monotonic()
            if now - state["last_tick_wall"] >= tick_interval_s:
                state["last_tick_wall"] = now
                bridge.tick(m.timestamp)

        try:
            replay(session_path, emit=emit, speed=speed, max_gap_s=30.0)
            bridge.tick(state["last_ts"])  # final snapshot at session end
        except (BridgeError, OSError) as e:
            print(f"replay feeder stopped: {e}")

    thread = threading.Thread(target=run, name="replay-feeder", daemon=True)
    thread.start()
    return thread


def start_live_feeder(bridge: TrackerBridge, provider: str = "adsblol",
                      lat: float = 40.0, lon: float = -83.0,
                      radius_nm: float = 100.0, interval_s: float = 10.0,
                      stop: Optional[threading.Event] = None
                      ) -> threading.Thread:
    """Poll a live readsb-style source into the bridge (M2 client reuse)."""
    base_url = PROVIDERS[provider]
    stop = stop or threading.Event()

    def run() -> None:
        while not stop.is_set():
            started = time.monotonic()
            try:
                measurements = normalize_readsb(
                    fetch_point(base_url, lat, lon, radius_nm))
                for m in measurements:
                    bridge.feed(m.to_csv_row())
                bridge.tick(time.time())
            except IngestError as e:
                print(f"live feeder poll failed: {e}")
            except BridgeError as e:
                print(f"live feeder stopped: {e}")
                return
            wait = interval_s - (time.monotonic() - started)
            if wait > 0:
                stop.wait(wait)

    thread = threading.Thread(target=run, name="live-feeder", daemon=True)
    thread.stop_event = stop  # type: ignore[attr-defined]
    thread.start()
    return thread
