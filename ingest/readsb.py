"""readsb-style v2 "point" API client and normalizer.

Covers the keyless community aggregators that expose the ADS-B Exchange v2
response shape (adsb.lol, airplanes.live):

    GET {base}/v2/point/{lat}/{lon}/{radius_nm}

Unit notes (this API is aviation-flavored, unlike OpenSky's SI):
  - ``now`` is unix time in MILLISECONDS
  - ``alt_baro``/``alt_geom`` are feet — or the string "ground" when parked
  - ``gs`` is knots
  - ``seen_pos`` is seconds since the position was last updated, so the
    measurement timestamp is ``now/1000 - seen_pos``
"""
from __future__ import annotations

from typing import List

from .net import get_json
from .session import Measurement

PROVIDERS = {
    "adsblol": "https://api.adsb.lol",
    "airplanes": "https://api.airplanes.live",
}

FT_TO_M = 0.3048
KN_TO_MS = 1852.0 / 3600.0  # knots -> m/s

# Positions older than this are navigation history, not live data.
MAX_POSITION_AGE_S = 60.0


def fetch_point(base_url: str, lat: float, lon: float, radius_nm: float,
                timeout_s: float = 15.0) -> dict:
    url = f"{base_url}/v2/point/{lat:.4f}/{lon:.4f}/{radius_nm:.0f}"
    return get_json(url, timeout_s=timeout_s)


def normalize_readsb(payload: dict, include_ground: bool = False
                     ) -> List[Measurement]:
    """v2 point payload -> normalized measurements (SI units)."""
    now = float(payload.get("now") or 0.0)
    # Defensive: readsb lineage uses milliseconds, but don't break if a
    # provider ever reports seconds.
    now_s = now / 1000.0 if now > 1e11 else now

    out: List[Measurement] = []
    for ac in payload.get("ac") or []:
        aircraft_id = str(ac.get("hex", "")).strip().lower()
        lat, lon = ac.get("lat"), ac.get("lon")
        if not aircraft_id or lat is None or lon is None:
            continue

        alt_baro = ac.get("alt_baro")
        grounded = alt_baro == "ground"
        if grounded and not include_ground:
            continue

        seen_pos = float(ac.get("seen_pos") or 0.0)
        if seen_pos > MAX_POSITION_AGE_S:
            continue
        # Round to the session file's 0.1 s precision. Residual cross-poll
        # jitter in now/seen_pos is absorbed by the recorder's per-aircraft
        # minimum-separation dedupe.
        timestamp = round(now_s - seen_pos, 1)

        altitude_ft = alt_baro if isinstance(alt_baro, (int, float)) \
            else ac.get("alt_geom")
        altitude_m = float(altitude_ft) * FT_TO_M \
            if isinstance(altitude_ft, (int, float)) else 0.0

        out.append(Measurement(
            aircraft_id=aircraft_id,
            timestamp=timestamp,
            latitude=float(lat),
            longitude=float(lon),
            altitude=altitude_m,
            velocity=float(ac.get("gs") or 0.0) * KN_TO_MS,
            heading=float(ac.get("track") or 0.0),
        ))
    return out
