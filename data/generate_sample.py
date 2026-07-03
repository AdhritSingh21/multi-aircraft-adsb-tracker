"""Generate data/sample_adsb.csv: a deterministic replayable ADS-B sample.

Three aircraft near Columbus, OH (frame origin ~40.0N, -82.9W):
  a1b2c3  eastbound at 220 m/s, FL345, reports every 2 s for the full 90 s,
          starts a gentle right turn at t=60 (tests filter under mild maneuver)
  b2d4e6  north-northeast at 180 m/s, climbing; goes SILENT after t=40 so the
          replay demonstrates stale-track pruning (30 s coast limit)
  c3f5a7  southwest at 150 m/s, low altitude, reports every 5 s

Position noise: ~40 m 1-sigma, fixed seed => identical output every run.
"""

import math
import random

EARTH_R = 6371000.0
NOISE_SIGMA_M = 40.0
OUT = "sample_adsb.csv"

random.seed(20260703)


def offset(lat, lon, east_m, north_m):
    dlat = math.degrees(north_m / EARTH_R)
    dlon = math.degrees(east_m / (EARTH_R * math.cos(math.radians(lat))))
    return lat + dlat, lon + dlon


def fly(icao, t0, t1, step, lat0, lon0, alt0, speed, heading, climb=0.0,
        turn_rate=0.0, turn_after=None):
    """Integrate a simple flight path and emit noisy reports."""
    rows = []
    lat, lon, alt, hdg = lat0, lon0, alt0, heading
    t = t0
    while t <= t1 + 1e-9:
        # noisy report of the current true position
        nlat, nlon = offset(lat, lon,
                            random.gauss(0.0, NOISE_SIGMA_M),
                            random.gauss(0.0, NOISE_SIGMA_M))
        rows.append((icao, round(t, 1), round(nlat, 6), round(nlon, 6),
                     round(alt, 1), round(speed, 1), round(hdg % 360.0, 1)))
        # advance truth by `step` seconds
        h = math.radians(hdg)
        lat, lon = offset(lat, lon, speed * math.sin(h) * step,
                          speed * math.cos(h) * step)
        alt += climb * step
        if turn_after is not None and t >= turn_after:
            hdg += turn_rate * step
        t += step
    return rows


rows = []
rows += fly("a1b2c3", 0.0, 90.0, 2.0, 40.05, -83.10, 10500.0, 220.0, 90.0,
            turn_rate=1.5, turn_after=60.0)
rows += fly("b2d4e6", 0.0, 40.0, 3.0, 39.90, -82.95, 4500.0, 180.0, 20.0,
            climb=8.0)
rows += fly("c3f5a7", 2.0, 90.0, 5.0, 40.15, -82.70, 3000.0, 150.0, 225.0)

rows.sort(key=lambda r: r[1])

with open(OUT, "w", newline="") as f:
    f.write("aircraft_id,timestamp,latitude,longitude,altitude,velocity,heading\n")
    for r in rows:
        f.write(",".join(str(v) for v in r) + "\n")

print(f"wrote {len(rows)} measurements to {OUT}")
