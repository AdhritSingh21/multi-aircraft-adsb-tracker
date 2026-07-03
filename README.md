# Multi-Aircraft Tracker from ADS-B Data

Real-time multi-target aircraft tracking system built on public ADS-B data.
A C++ tracking core maintains aircraft tracks with a constant-velocity Kalman
filter and prunes stale tracks; ingestion, association, a WebSocket API, and a
live map dashboard are layered on incrementally (see
[PROJECT_PLAN.md](PROJECT_PLAN.md)).

**Status: Milestone 3 complete** — geometric track association (chi²-gated
nearest-neighbor and Hungarian assignment) with identities hidden from the
tracker: **100%** association accuracy on the sample replay, **99.23%** on a
real 36-aircraft live session, scored against ADS-B ids as ground truth.

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

## What it does (Milestone 2)

- **Live ingestion** from keyless community ADS-B aggregators speaking the
  readsb/ADS-B-Exchange v2 API (`adsb.lol`, `airplanes.live`), plus an
  OpenSky Network `/states/all` client. Normalizers convert everything to SI
  (feet→m, knots→m/s, ms-epoch→s, positions backdated by their age) so every
  layer downstream speaks one measurement format.
- **Recorder**: polls a source, dedupes re-reported states on
  `(aircraft, timestamp)`, and appends to a session CSV — flushed every poll,
  so an interrupted recording is still replayable; transient poll failures
  retry, repeated failures abort cleanly.
- **Replay mode**: re-emits any session with original (or scaled) timing —
  the repeatable-demo path the M4 backend will consume.
- **Simulated source**: deterministic OpenSky-shaped fleet (fixed seed) so
  the full record pipeline runs offline with identical output every run.

```sh
# one live fetch (100 nm around Columbus, OH), normalized CSV to stdout
python -m ingest snapshot

# record 90 s of live traffic, then replay it through the C++ tracker
python -m ingest record --out data/session_live.csv --duration 90
cpp/build/adsb_replay data/session_live.csv

# offline: deterministic simulated session
python -m ingest record --out data/session_sim.csv --source sim --duration 75 --interval 5

# repeatable demo: emit a session with live pacing at 10x
python -m ingest replay data/session_live.csv --speed 10
```

## What it does (Milestone 3)

- **Geometric association** — the tracker no longer needs to trust ADS-B
  identity. Each scan, every track is predicted forward, measurements are
  gated by squared Mahalanobis distance against the filter's innovation
  covariance (chi², 2 DOF, 99%), and assignments are solved by either
  gated greedy nearest-neighbor or the Hungarian algorithm (Kuhn–Munkres,
  global-optimal). Unassigned measurements spawn new tracks.
- **Honest scoring** — in `--assoc nn|hungarian` modes the replay tool hides
  identities from the tracker and scores every association decision against
  the held-back ADS-B ids:

```sh
cpp/build/adsb_replay data/session_live_columbus.csv --assoc hungarian
# --- association score (hungarian, ids hidden) ---
# associations scored    : 259
# correct (vs ADS-B id)  : 257
# association accuracy   : 99.23%
```

  Measured: 100% (70/70) on the sample replay, 99.23% (257/259) on the live
  36-aircraft session. Greedy NN and Hungarian score identically on this
  data (traffic is rarely close enough for the crossing case; the unit tests
  demonstrate where Hungarian wins). Known limitation: sustained maneuvers
  can fragment a track (visible as extra `tracks created`) — process-noise
  tuning planned for M6.

## Layout

```
cpp/                C++ tracking core (library + replay demo + tests)
  include/adsb/     measurement.hpp, kalman_filter.hpp, track.hpp,
                    track_manager.hpp, csv_replay.hpp
  src/              implementations + main.cpp (adsb_replay)
  tests/            unit tests (assert-based harness, CTest-registered)
ingest/             Python ingestion: opensky.py + readsb.py (clients),
                    recorder.py, replayer.py, simsource.py, cli.py
  tests/            pytest suites (no network required)
data/               sample_adsb.csv + generate_sample.py, recorded sessions
backend/ dashboard/  later milestones (see PROJECT_PLAN.md)
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

### Python ingestion tests

```sh
python -m pytest ingest/tests -q     # from the repo root; stdlib-only code, pytest to run
```

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

M4 FastAPI/WebSocket backend → M5 React map dashboard → M6 metrics & polish
(incl. process-noise tuning for maneuver robustness). Details and acceptance
criteria: [PROJECT_PLAN.md](PROJECT_PLAN.md).
