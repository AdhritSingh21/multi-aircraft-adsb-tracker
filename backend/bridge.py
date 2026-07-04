"""TrackerBridge: owns the adsb_stream subprocess.

Protocol (see cpp/src/stream_main.cpp):
    stdin  <- normalized measurement CSV lines, then "TICK <t>"
    stdout -> one JSON snapshot line per TICK

A daemon reader thread parses each snapshot, stores the latest, and invokes
``on_snapshot`` (from the reader thread — consumers adapt to their own
threading model, e.g. asyncio's call_soon_threadsafe).
"""
from __future__ import annotations

import json
import subprocess
import threading
from typing import Callable, Optional


class BridgeError(RuntimeError):
    """The adsb_stream process is gone or refused input."""


class TrackerBridge:
    def __init__(self, exe: str, assoc: str = "id",
                 on_snapshot: Optional[Callable[[dict], None]] = None):
        self.on_snapshot = on_snapshot
        try:
            self._proc = subprocess.Popen(
                [exe, "--assoc", assoc],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=None,  # inherit: tracker errors surface in server logs
                text=True,
                encoding="utf-8",
                bufsize=1,  # line buffered
            )
        except OSError as e:
            raise BridgeError(f"cannot start tracker '{exe}': {e}") from e

        self._latest: Optional[dict] = None
        self._lock = threading.Lock()
        self._reader = threading.Thread(
            target=self._read_loop, name="adsb-stream-reader", daemon=True)
        self._reader.start()

    def feed(self, csv_row: str) -> None:
        """Queue one measurement line into the current scan."""
        self._write(csv_row + "\n")

    def tick(self, t: float) -> None:
        """Process the buffered scan at time t; a snapshot will arrive on
        the reader thread shortly after."""
        self._write(f"TICK {t:.1f}\n")

    def latest(self) -> Optional[dict]:
        with self._lock:
            return self._latest

    def alive(self) -> bool:
        return self._proc.poll() is None

    def close(self) -> None:
        try:
            if self._proc.stdin and not self._proc.stdin.closed:
                self._proc.stdin.close()  # EOF -> clean exit
            self._proc.wait(timeout=3)
        except (OSError, subprocess.TimeoutExpired):
            self._proc.kill()

    def _write(self, data: str) -> None:
        try:
            assert self._proc.stdin is not None
            self._proc.stdin.write(data)
            self._proc.stdin.flush()
        except (OSError, ValueError, AssertionError) as e:
            raise BridgeError(f"tracker process not accepting input: {e}") from e

    def _read_loop(self) -> None:
        assert self._proc.stdout is not None
        for line in self._proc.stdout:
            try:
                snapshot = json.loads(line)
            except json.JSONDecodeError:
                continue  # truncated line during shutdown
            with self._lock:
                self._latest = snapshot
            callback = self.on_snapshot
            if callback is not None:
                callback(snapshot)
