"""Deterministic simulated 'live' ADS-B source.

Produces payloads with the exact shape of OpenSky's ``/states/all`` response,
so the entire record pipeline (fetch -> normalize -> dedupe -> session file)
runs offline and repeatably — no network, no rate limits, identical output
for a given seed. One aircraft goes silent partway through so recorded sim
sessions also demonstrate stale-track pruning downstream.
"""
from __future__ import annotations

import math
from typing import List, Optional

EARTH_R = 6371000.0


class _Lcg:
    """Tiny deterministic noise generator (uniform in [-1, 1))."""

    def __init__(self, seed: int):
        self._s = seed & 0xFFFFFFFFFFFFFFFF

    def next(self) -> float:
        self._s = (self._s * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFFFFFFFFFF
        return ((self._s >> 17) & 0xFFFFF) / 524288.0 - 1.0


class _SimAircraft:
    def __init__(self, icao24: str, callsign: str, lat: float, lon: float,
                 alt: float, speed: float, heading: float, climb: float = 0.0,
                 turn_rate: float = 0.0, silent_after: Optional[float] = None):
        self.icao24 = icao24
        self.callsign = callsign
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.speed = speed
        self.heading = heading
        self.climb = climb
        self.turn_rate = turn_rate       # deg/s
        self.silent_after = silent_after  # sim seconds; then stops reporting

    def advance(self, dt: float) -> None:
        h = math.radians(self.heading)
        east = self.speed * math.sin(h) * dt
        north = self.speed * math.cos(h) * dt
        self.lat += math.degrees(north / EARTH_R)
        self.lon += math.degrees(east / (EARTH_R * math.cos(math.radians(self.lat))))
        self.alt += self.climb * dt
        self.heading = (self.heading + self.turn_rate * dt) % 360.0


class SimulatedSource:
    """Drop-in replacement for :func:`ingest.opensky.fetch_states`.

    Each ``fetch()`` advances the simulation by ``step_s`` seconds and returns
    an OpenSky-shaped payload for the current instant.
    """

    def __init__(self, t0: float = 0.0, step_s: float = 5.0, seed: int = 7,
                 noise_sigma_m: float = 40.0):
        self._t = float(t0)
        self._elapsed = 0.0
        self._step = float(step_s)
        self._noise = _Lcg(seed)
        self._noise_m = noise_sigma_m
        # Fleet mirrors the M1 sample: cruiser with a late turn, a climber
        # that goes silent (stale-track demo), and a slow low-altitude target.
        self._fleet: List[_SimAircraft] = [
            _SimAircraft("s1mA01", "SIM101", 40.05, -83.10, 10500.0, 220.0,
                         90.0, turn_rate=0.0),
            _SimAircraft("s1mB02", "SIM202", 39.90, -82.95, 4500.0, 180.0,
                         20.0, climb=8.0, silent_after=40.0),
            _SimAircraft("s1mC03", "SIM303", 40.15, -82.70, 3000.0, 150.0,
                         225.0),
        ]

    def fetch(self) -> dict:
        now = self._t
        states = []
        for a in self._fleet:
            if a.silent_after is not None and self._elapsed > a.silent_after:
                continue
            nlat = a.lat + math.degrees(
                self._noise.next() * self._noise_m / EARTH_R)
            nlon = a.lon + math.degrees(
                self._noise.next() * self._noise_m
                / (EARTH_R * math.cos(math.radians(a.lat))))
            # Same field order as an OpenSky state vector (17 entries).
            states.append([
                a.icao24, a.callsign, "SIM", now, now, nlon, nlat, a.alt,
                False, a.speed, a.heading, a.climb, None, a.alt + 30.0,
                None, False, 0,
            ])
        for a in self._fleet:
            a.advance(self._step)
        self._t += self._step
        self._elapsed += self._step
        return {"time": now, "states": states}
