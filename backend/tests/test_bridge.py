"""TrackerBridge integration tests against the real adsb_stream executable.

Skipped (not failed) when the C++ build output is missing, so the Python
suite still runs on a machine that hasn't built the core yet.
"""
import time
from pathlib import Path

import pytest

from backend.bridge import BridgeError, TrackerBridge

REPO = Path(__file__).resolve().parent.parent.parent
EXE = next((p for p in (REPO / "cpp" / "build" / "adsb_stream.exe",
                        REPO / "cpp" / "build" / "adsb_stream")
            if p.exists()), None)

pytestmark = pytest.mark.skipif(
    EXE is None, reason="adsb_stream not built (run cmake --build cpp/build)")


def wait_for(predicate, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def test_bridge_produces_snapshots_and_fans_out():
    received = []
    bridge = TrackerBridge(str(EXE), assoc="id",
                           on_snapshot=received.append)
    try:
        assert bridge.alive()
        bridge.feed("aaa111,10.0,40.10,-82.90,10500,220,90")
        bridge.feed("bbb222,10.0,39.90,-83.10,3000,150,225")
        bridge.feed("ccc333,10.0,40.30,-82.70,5000,180,10")
        bridge.tick(10.0)

        assert wait_for(lambda: bridge.latest() is not None)
        snap = bridge.latest()
        assert snap["time"] == 10.0
        assert len(snap["tracks"]) == 3
        assert snap["stats"]["active"] == 3
        assert {t["icao"] for t in snap["tracks"]} == {"aaa111", "bbb222",
                                                       "ccc333"}
        assert received and received[-1] == snap  # callback fan-out

        # Stale pruning visible through the protocol.
        bridge.tick(60.0)
        assert wait_for(lambda: bridge.latest()["stats"]["active"] == 0)
        assert bridge.latest()["stats"]["stale_removed"] == 3
    finally:
        bridge.close()
    assert not bridge.alive()


def test_bridge_write_after_close_raises():
    bridge = TrackerBridge(str(EXE))
    bridge.close()
    with pytest.raises(BridgeError):
        bridge.feed("aaa111,10.0,40.10,-82.90,10500,220,90")


def test_bridge_bad_exe_raises():
    with pytest.raises(BridgeError):
        TrackerBridge(str(REPO / "does-not-exist.exe"))
