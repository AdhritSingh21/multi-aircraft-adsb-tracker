# Multi-Aircraft Tracker from ADS-B Data

Real-time multi-target aircraft tracking system built on public ADS-B data.
A C++ tracking core maintains aircraft tracks with a constant-velocity Kalman
filter and prunes stale tracks; ingestion, association, a WebSocket API, and a
live map dashboard are layered on incrementally (see
[PROJECT_PLAN.md](PROJECT_PLAN.md)).

**Status: Milestone 1 complete** — sample-data replay tracker with tested
Kalman filtering and track management.

```
=== t=70s | active tracks: 2 ===
TRK  ICAO24          LAT         LON   ALT[m] SPD[m/s]    HDG   AGE  HITS
1    a1b2c3      40.0491    -82.9243    10500   218.7    102   2.0    35
3    c3f5a7      40.0882    -82.7806     3000   145.4    225   3.0    14

--- replay summary ---
measurements processed : 78
tracks created         : 3
stale tracks removed   : 1
tracks active at end   : 2
```

## What it does (Milestone 1)

- Normalizes ADS-B reports into a common `Measurement` format
  (`aircraft_id, timestamp, lat, lon, altitude, velocity, heading`)
- Projects lat/lon into a local east/north tangent-plane frame (meters) so the
  filter stays linear
- Runs a 4-state constant-velocity Kalman filter per aircraft
  (`[x, y, vx, vy]`, white-noise-acceleration process model, position-only
  updates) — including Mahalanobis distance for association gating (used in
  Milestone 3)
- `TrackManager` creates tracks, fuses measurements into them, smooths
  altitude, keeps a bounded position trail, and prunes tracks that stop
  reporting (30 s coast limit)
- Replays a recorded CSV deterministically: one aircraft goes silent mid-run
  and is pruned; another performs a turn the filter follows

No external C++ dependencies — the 4x4 filter math is written out directly and
tests use a small assert-based harness wired into CTest.

## Layout

```
cpp/                C++ tracking core (library + replay demo + tests)
  include/adsb/     measurement.hpp, kalman_filter.hpp, track.hpp,
                    track_manager.hpp, csv_replay.hpp
  src/              implementations + main.cpp (adsb_replay)
  tests/            unit tests (assert-based harness, CTest-registered)
data/               sample_adsb.csv + generate_sample.py (deterministic)
ingest/ backend/ dashboard/   later milestones (see PROJECT_PLAN.md)
```

## Build & run

Requirements: CMake ≥ 3.16, a C++17 compiler (GCC/Clang/MSVC), Ninja or Make.

```sh
cmake -S cpp -B cpp/build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build

# run unit tests
ctest --test-dir cpp/build --output-on-failure

# run the replay demo (from the repo root)
cpp/build/adsb_replay data/sample_adsb.csv
```

On Windows with MinGW GCC, executables are linked `-static` so they run
without the compiler's `bin` directory on PATH.

### Regenerating sample data

```sh
cd data && python generate_sample.py
```

Fixed seed — the CSV is identical on every run, so replay results (tracks
created, stale tracks pruned) are reproducible.

## Tests

`cpp/tests/test_main.cpp` proves the core claims:

| test | proves |
|------|--------|
| `local_frame_roundtrip` | geodetic ↔ local projection is consistent |
| `kalman_predict` | CV propagation; coasting grows uncertainty; dt ≤ 0 safe |
| `kalman_converges_on_cv_target` | filter learns velocity from noisy positions only |
| `mahalanobis_gate` | innovation-based gating metric behaves |
| `manager_creates_track` / `updates_existing_track` | tracks form and fuse measurements |
| `manager_separate_tracks_per_aircraft` | multi-target bookkeeping |
| `manager_prunes_stale_tracks` | stale tracks removed after coast limit |
| `csv_loader_parses_sorts_and_skips_bad_rows` | replay input is robust |

## Roadmap

M2 live/recorded OpenSky ingestion → M3 gated nearest-neighbor (+ Hungarian)
association → M4 FastAPI/WebSocket backend → M5 React map dashboard →
M6 metrics & polish. Details and acceptance criteria:
[PROJECT_PLAN.md](PROJECT_PLAN.md).
