# Multi-Aircraft Tracker from Live ADS-B Data — Project Plan

A defense-relevant real-time aircraft tracking system built on public ADS-B data.
Core competencies demonstrated: state estimation (Kalman filtering), multi-target
tracking, track management, data association, C++ systems programming, and
real-time visualization.

## Architecture

```
+------------------+     +--------------------+     +-------------------+
|  ADS-B Ingestion |     |   Tracking Core    |     |  API / Dashboard  |
|  (Python)        | --> |   (C++)            | --> |  (FastAPI/React)  |
|                  |     |                    |     |                   |
| - OpenSky feed   |     | - Kalman filter    |     | - JSON /tracks    |
| - CSV replay     |     | - Track manager    |     | - WebSocket push  |
| - Normalization  |     | - Association      |     | - Live map + table|
+------------------+     +--------------------+     +-------------------+
```

Measurement format (normalized, all layers speak this):

| field       | type   | units                      |
|-------------|--------|----------------------------|
| aircraft_id | string | ICAO24 hex address         |
| timestamp   | double | seconds (unix epoch or t0) |
| latitude    | double | degrees                    |
| longitude   | double | degrees                    |
| altitude    | double | meters                     |
| velocity    | double | m/s ground speed           |
| heading     | double | degrees from north         |

## Folder structure

```
adsb-tracker/
├── PROJECT_PLAN.md          # this file
├── README.md                # build/run/test instructions
├── cpp/                     # C++ tracking core (M1)
│   ├── CMakeLists.txt
│   ├── include/adsb/        # public headers
│   │   ├── measurement.hpp
│   │   ├── kalman_filter.hpp
│   │   ├── track.hpp
│   │   ├── track_manager.hpp
│   │   └── csv_replay.hpp
│   ├── src/                 # implementations + replay demo main
│   └── tests/               # unit tests (self-contained harness)
├── data/                    # sample/replay ADS-B data (M1)
│   └── sample_adsb.csv
├── ingest/                  # Python ADS-B ingestion (M2)
├── backend/                 # FastAPI + WebSocket server (M4)
├── dashboard/               # React/Vite map dashboard (M5)
└── docs/                    # architecture diagram, metrics (M6)
```

## Design decisions

- **Local tangent-plane tracking frame.** Lat/lon measurements are projected to
  a local ENU (east-north-up) x/y frame in meters around a reference point via
  equirectangular approximation — adequate for regional tracking scopes and keeps
  the filter linear.
- **Constant-velocity Kalman filter, state = [x, y, vx, vy].** Process noise via
  the white-noise-acceleration model. Altitude is smoothed separately (alpha
  filter) since vertical dynamics differ from horizontal.
- **No external C++ dependencies.** Fixed-size 4x4 matrix math is implemented
  directly (small, testable, shows the math). Tests use a minimal assert-based
  harness registered with CTest. Keeps the build trivially portable.
- **Sample-data replay first, live feed second.** Replay from CSV gives
  deterministic demos and repeatable association-accuracy metrics; live OpenSky
  ingestion is added on top in M2 without touching the core.

## Milestones & acceptance criteria

### M0 — Workspace inspection & plan  ✅
- [x] Inspect workspace (`C:\Users\19176\Downloads`, project isolated in `adsb-tracker/`)
- [x] Verify/provision toolchain (Python 3.11, git; CMake + Ninja via pip; GCC via winget/WinLibs)
- [x] This plan committed

### M1 — Local sample-data tracker (C++ core)  ✅
- [x] `Measurement` struct + local ENU projection
- [x] Constant-velocity Kalman filter (predict/update, 4-state)
- [x] `Track` struct: filter state, last-update age, hit count, history
- [x] `TrackManager`: create tracks, update by aircraft_id, prune stale tracks
- [x] CSV replay file with ≥3 aircraft, one going stale mid-replay
- [x] `adsb_replay` executable: replays CSV, prints live track table + summary
- **Accepted when:** `cmake --build` succeeds and `ctest` passes tests proving
  (a) filter converges on a constant-velocity target, (b) tracks update from
  measurements, (c) stale tracks are pruned, (d) multiple aircraft yield
  separate tracks. README documents build/run/test.

### M2 — Live/simulated ADS-B ingestion  ✅
- [x] Live ingestion clients: readsb-style v2 point API (adsb.lol,
  airplanes.live — keyless, default) and OpenSky `/states/all` (implemented
  and unit-tested; the host was unreachable from the dev network, so live
  acceptance ran against adsb.lol)
- [x] Normalizers → common measurement format (SI units; feet/knots/ms-epoch
  conversions for readsb sources)
- [x] Recorder (poll → dedupe → session CSV, failure-tolerant) + deterministic
  simulated source + timed replay mode for repeatable demos
- **Accepted when:** a recorded live session can be replayed through the C++
  core. ✅ 90 s live recording (300 measurements, 36 aircraft) → `adsb_replay`
  → 37 tracks, 5 stale pruned, clean exit.

### M3 — Track association  ✅
- [x] Mahalanobis gating using filter innovation covariance (chi², 2 DOF, 99%)
- [x] Nearest-neighbor assignment (ADS-B ids hidden from the tracker)
- [x] Hungarian algorithm (Kuhn–Munkres) for global assignment
- [x] Process model corrected to continuous WNA discretization so covariance
  growth is independent of prediction cadence (predict(a)+predict(b) ≡
  predict(a+b) — locked in by a unit test)
- **Accepted when:** association accuracy ≥ target on replay data with id
  hidden, measured against ADS-B ids as ground truth. ✅ Measured: **100%**
  (70/70) on the 3-aircraft sample; **99.23%** (257/259) on the 36-aircraft
  live Columbus session; nn and Hungarian identical on this data. Known
  limitation (deferred to M6 tuning): track fragmentation during sustained
  maneuvers (8 tracks created for 3 sample aircraft).

### M4 — API/backend
- [ ] FastAPI service exposing `GET /tracks` (JSON)
- [ ] WebSocket streaming of track updates
- **Accepted when:** client receives live track updates over WebSocket.

### M5 — Dashboard
- [ ] React/Vite app, live map (Leaflet) with aircraft positions + trails
- [ ] Track table: ID, callsign, speed, altitude, update age
- [ ] Metrics bar: active tracks, update rate, stale tracks removed
- **Accepted when:** dashboard renders live replay session end-to-end.

### M6 — Polish for GitHub/resume
- [ ] Architecture diagram, README polish, screenshots/GIF
- [ ] Performance metrics on replay data: active tracks maintained, update
  rate, association accuracy
- **Accepted when:** repo is presentable cold to a hiring manager.

## Rules of engagement
- Work stays inside `adsb-tracker/`; no unrelated files touched.
- Tests run green before a milestone is called complete.
- Working code over fancy architecture; metrics reported honestly (no inflated
  accuracy claims — replay-measured numbers only).
