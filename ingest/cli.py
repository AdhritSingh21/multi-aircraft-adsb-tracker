"""Command-line interface: snapshot / record / replay.

Examples (run from the repo root):

  # one live fetch (adsb.lol, 100 nm around Columbus OH), print normalized CSV
  python -m ingest snapshot

  # record 90 s of live traffic to a session file
  python -m ingest record --out data/session_live.csv --duration 90

  # OpenSky instead (SI-native but rate-limited; sometimes unreachable)
  python -m ingest record --out data/s.csv --source opensky --bbox 38.5 -85.0 41.5 -81.0

  # deterministic simulated session — no network needed, same output every run
  python -m ingest record --out data/session_sim.csv --source sim --duration 60 --interval 5

  # replay a session at 10x speed (repeatable demo mode)
  python -m ingest replay data/session_live.csv --speed 10
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Callable, List

from .net import IngestError
from .opensky import BoundingBox, fetch_states, normalize_states
from .readsb import PROVIDERS, fetch_point, normalize_readsb
from .recorder import record
from .replayer import replay
from .session import CSV_HEADER, Measurement
from .simsource import SimulatedSource


def _build_poll(args) -> Callable[[], List[Measurement]]:
    """Compose fetch + normalize for the chosen source."""
    include_ground = args.include_ground
    if args.source == "sim":
        sim = SimulatedSource(t0=time.time(), step_s=args.interval)
        return lambda: normalize_states(sim.fetch(), include_ground)
    if args.source == "opensky":
        bbox = None
        if args.bbox is not None:
            lamin, lomin, lamax, lomax = args.bbox
            bbox = BoundingBox(lamin=lamin, lomin=lomin,
                               lamax=lamax, lomax=lomax)
        return lambda: normalize_states(fetch_states(bbox=bbox),
                                        include_ground)
    base_url = PROVIDERS[args.source]
    lat, lon = args.center
    return lambda: normalize_readsb(
        fetch_point(base_url, lat, lon, args.radius), include_ground)


def _cmd_snapshot(args) -> int:
    measurements = _build_poll(args)()
    print(CSV_HEADER)
    for m in measurements:
        print(m.to_csv_row())
    print(f"# {len(measurements)} aircraft", file=sys.stderr)
    return 0


def _cmd_record(args) -> int:
    stats = record(
        out_path=args.out,
        poll=_build_poll(args),
        duration_s=args.duration,
        interval_s=args.interval,
    )
    print(f"session saved: {args.out} | {stats.written} measurements, "
          f"{len(stats.aircraft)} aircraft, {stats.polls} polls, "
          f"{stats.failures} failed")
    return 0 if stats.written > 0 else 1


def _cmd_replay(args) -> int:
    print(CSV_HEADER, flush=True)
    count = replay(
        args.session,
        emit=lambda m: print(m.to_csv_row(), flush=True),
        speed=args.speed,
        max_gap_s=args.max_gap,
    )
    print(f"# replayed {count} measurements", file=sys.stderr)
    return 0 if count > 0 else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m ingest",
        description="ADS-B ingestion: fetch/record live or simulated traffic, "
                    "replay recorded sessions.")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_source_args(p):
        p.add_argument("--source",
                       choices=["adsblol", "airplanes", "opensky", "sim"],
                       default="adsblol",
                       help="data source (default adsblol: keyless, reliable; "
                            "opensky needs no key but is rate-limited; "
                            "sim is offline + deterministic)")
        p.add_argument("--center", nargs=2, type=float, default=[40.0, -83.0],
                       metavar=("LAT", "LON"),
                       help="query center for adsblol/airplanes "
                            "(default: Columbus OH)")
        p.add_argument("--radius", type=float, default=100.0,
                       help="query radius in nautical miles, max 250 "
                            "(adsblol/airplanes)")
        p.add_argument("--bbox", nargs=4, type=float, default=None,
                       metavar=("LAMIN", "LOMIN", "LAMAX", "LOMAX"),
                       help="bounding box in degrees (opensky only)")
        p.add_argument("--include-ground", action="store_true",
                       help="keep aircraft reported on the ground")
        p.add_argument("--interval", type=float, default=10.0,
                       help="poll/step interval in seconds (default 10)")

    p_snap = sub.add_parser("snapshot", help="one fetch, print normalized CSV")
    add_source_args(p_snap)
    p_snap.set_defaults(func=_cmd_snapshot)

    p_rec = sub.add_parser("record", help="poll into a session CSV")
    add_source_args(p_rec)
    p_rec.add_argument("--out", required=True, help="session file to write")
    p_rec.add_argument("--duration", type=float, default=60.0,
                       help="recording length in seconds (default 60)")
    p_rec.set_defaults(func=_cmd_record)

    p_rep = sub.add_parser("replay", help="emit a session with live timing")
    p_rep.add_argument("session", help="session CSV to replay")
    p_rep.add_argument("--speed", type=float, default=1.0,
                       help="time multiplier (10 = 10x faster than real time)")
    p_rep.add_argument("--max-gap", type=float, default=30.0,
                       help="cap any single wait at this many wall seconds")
    p_rep.set_defaults(func=_cmd_replay)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except IngestError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
