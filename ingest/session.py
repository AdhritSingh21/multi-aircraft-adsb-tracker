"""Normalized measurement format and session-file I/O.

A "session" is a CSV of normalized ADS-B measurements in the exact schema the
C++ tracking core replays:

    aircraft_id,timestamp,latitude,longitude,altitude,velocity,heading

Recorded live sessions, simulated sessions, and generated samples are all
interchangeable: anything in this format can be fed to ``adsb_replay`` or the
Python replayer.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Union

CSV_HEADER = "aircraft_id,timestamp,latitude,longitude,altitude,velocity,heading"


@dataclass(frozen=True)
class Measurement:
    aircraft_id: str   # ICAO24 hex address, lowercase
    timestamp: float   # seconds (unix epoch for live data)
    latitude: float    # degrees
    longitude: float   # degrees
    altitude: float    # meters
    velocity: float    # m/s ground speed
    heading: float     # degrees clockwise from true north

    def to_csv_row(self) -> str:
        return (
            f"{self.aircraft_id},{self.timestamp:.1f},"
            f"{self.latitude:.6f},{self.longitude:.6f},"
            f"{self.altitude:.1f},{self.velocity:.1f},{self.heading:.1f}"
        )


def parse_csv_line(line: str) -> Optional[Measurement]:
    """One CSV line -> Measurement, or None for header/comment/bad rows.

    Mirrors the C++ loader's tolerance: malformed rows are skipped, never
    fatal (live radio data is messy; one bad row must not kill a replay).
    """
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    fields = line.split(",")
    if len(fields) < 7:
        return None
    try:
        return Measurement(
            aircraft_id=fields[0],
            timestamp=float(fields[1]),
            latitude=float(fields[2]),
            longitude=float(fields[3]),
            altitude=float(fields[4]),
            velocity=float(fields[5]),
            heading=float(fields[6]),
        )
    except ValueError:
        return None  # header row or unparsable numeric field


def read_session(path: Union[str, Path]) -> List[Measurement]:
    """Load a session file, sorted by timestamp for in-order replay."""
    measurements: List[Measurement] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = parse_csv_line(line)
            if m is not None:
                measurements.append(m)
    measurements.sort(key=lambda m: m.timestamp)
    return measurements


class SessionWriter:
    """Appends measurements to a session CSV, writing the header once.

    Flushes after every batch so an interrupted recording (Ctrl-C, network
    drop) still leaves a valid, replayable file.
    """

    def __init__(self, path: Union[str, Path]):
        self._path = Path(path)
        is_new = not self._path.exists() or self._path.stat().st_size == 0
        self._f = open(self._path, "a", encoding="utf-8", newline="")
        if is_new:
            self._f.write(CSV_HEADER + "\n")
            self._f.flush()

    def write(self, measurements: Iterable[Measurement]) -> int:
        n = 0
        for m in measurements:
            self._f.write(m.to_csv_row() + "\n")
            n += 1
        self._f.flush()
        return n

    def close(self) -> None:
        self._f.close()

    def __enter__(self) -> "SessionWriter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
