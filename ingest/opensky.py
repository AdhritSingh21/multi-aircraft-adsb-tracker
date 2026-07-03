"""OpenSky Network REST client and state-vector normalizer.

Uses the anonymous tier of ``GET /api/states/all`` (no credentials needed):
~10 s data resolution, a few hundred request credits per day per IP, and a
bounding box reduces the credit cost per call. Good enough for short recorded
sessions; heavier use requires an OpenSky account (OAuth2 client credentials).

API reference: https://openskynetwork.github.io/opensky-api/rest.html
"""
from __future__ import annotations

import urllib.parse
from dataclasses import dataclass
from typing import List, Optional

from .net import IngestError, get_json
from .session import Measurement

__all__ = ["BoundingBox", "IngestError", "fetch_states", "normalize_states",
           "STATES_URL"]

STATES_URL = "https://opensky-network.org/api/states/all"

# Indices into an OpenSky state vector (order defined by the REST API docs).
# NOTE: longitude comes BEFORE latitude in the raw vector.
ICAO24 = 0
CALLSIGN = 1
ORIGIN_COUNTRY = 2
TIME_POSITION = 3
LAST_CONTACT = 4
LONGITUDE = 5
LATITUDE = 6
BARO_ALTITUDE = 7
ON_GROUND = 8
VELOCITY = 9
TRUE_TRACK = 10
VERTICAL_RATE = 11
SENSORS = 12
GEO_ALTITUDE = 13


@dataclass(frozen=True)
class BoundingBox:
    """Geographic query window (degrees). Shrinks OpenSky credit cost."""

    lamin: float  # south edge
    lomin: float  # west edge
    lamax: float  # north edge
    lomax: float  # east edge

    def as_params(self) -> dict:
        return {
            "lamin": self.lamin,
            "lomin": self.lomin,
            "lamax": self.lamax,
            "lomax": self.lomax,
        }


def fetch_states(bbox: Optional[BoundingBox] = None, timeout_s: float = 15.0,
                 url: str = STATES_URL) -> dict:
    """One GET /states/all call. Returns the decoded JSON payload."""
    if bbox is not None:
        url = url + "?" + urllib.parse.urlencode(bbox.as_params())
    return get_json(url, timeout_s=timeout_s)


def normalize_states(payload: dict, include_ground: bool = False
                     ) -> List[Measurement]:
    """OpenSky payload -> normalized measurements.

    Rules (each keeps live data usable without inventing information):
      - no latitude/longitude            -> skip (nothing to track)
      - on ground                        -> skip unless include_ground
      - timestamp: time_position ONLY. OpenSky retains the last known lat/lon
        while nulling time_position once the position is >15 s old; stamping
        that stale fix with last_contact would defeat recorder dedupe and
        produce frozen-but-"fresh" tracks downstream. No position time ->
        no measurement.
      - altitude: barometric, falling back to geometric, falling back to 0
      - velocity/heading: 0 when not reported (filter re-estimates velocity
        from positions anyway; the seed is just a hint)
    """
    states = payload.get("states") or []
    out: List[Measurement] = []
    for s in states:
        if len(s) <= TRUE_TRACK:
            continue  # truncated vector
        lon, lat = s[LONGITUDE], s[LATITUDE]
        if lat is None or lon is None:
            continue
        if bool(s[ON_GROUND]) and not include_ground:
            continue
        ts = s[TIME_POSITION]
        if ts is None:
            continue  # stale/retained position — not a live measurement

        altitude = s[BARO_ALTITUDE]
        if altitude is None and len(s) > GEO_ALTITUDE:
            altitude = s[GEO_ALTITUDE]
        if altitude is None:
            altitude = 0.0

        out.append(Measurement(
            aircraft_id=str(s[ICAO24]).strip().lower(),
            timestamp=float(ts),
            latitude=float(lat),
            longitude=float(lon),
            altitude=float(altitude),
            velocity=float(s[VELOCITY] or 0.0),
            heading=float(s[TRUE_TRACK] or 0.0),
        ))
    return out
