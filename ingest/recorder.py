"""Recorder: poll a measurement source and write a session file.

The source is any zero-argument callable returning normalized measurements
(OpenSky, a readsb-style aggregator, or the simulator — composed in cli.py).

Dedupe: live APIs re-report an aircraft's last known state on every poll, and
reconstructed timestamps (e.g. readsb's ``now - seen_pos``) can jitter by a
fraction of a second between polls. A measurement is therefore accepted only
if it is at least ``min_separation_s`` newer than the aircraft's last accepted
one — exact repeats AND jittered near-duplicates are dropped, and memory stays
bounded by fleet size rather than session length.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Set, Union

from .net import IngestError
from .session import Measurement, SessionWriter


@dataclass
class RecordStats:
    polls: int = 0
    failures: int = 0
    written: int = 0
    aircraft: Set[str] = field(default_factory=set)


def record(out_path: Union[str, Path],
           poll: Callable[[], List[Measurement]],
           duration_s: float = 60.0,
           interval_s: float = 10.0,
           min_separation_s: float = 0.5,
           max_consecutive_failures: int = 5,
           sleep: Callable[[float], None] = time.sleep,
           clock: Callable[[], float] = time.monotonic,
           log: Callable[[str], None] = print) -> RecordStats:
    """Call ``poll`` every ``interval_s`` for ``duration_s``, appending
    deduplicated measurements to ``out_path``.

    Individual poll failures (IngestError) are logged and retried on the next
    cycle; ``max_consecutive_failures`` in a row aborts (dead network,
    exhausted rate limit). The session file is flushed after every poll, so
    whatever was captured before an abort or Ctrl-C is replayable.
    """
    stats = RecordStats()
    last_accepted: Dict[str, float] = {}
    consecutive_failures = 0

    with SessionWriter(out_path) as writer:
        start = clock()
        while True:
            poll_started = clock()
            stats.polls += 1
            try:
                measurements = poll()
                fresh = []
                for m in measurements:
                    last = last_accepted.get(m.aircraft_id)
                    if last is None or m.timestamp >= last + min_separation_s:
                        fresh.append(m)
                        last_accepted[m.aircraft_id] = m.timestamp
                writer.write(fresh)
                stats.written += len(fresh)
                stats.aircraft.update(m.aircraft_id for m in fresh)
                consecutive_failures = 0
                log(f"poll {stats.polls}: {len(fresh)} new of "
                    f"{len(measurements)} reports | total {stats.written} "
                    f"measurements, {len(stats.aircraft)} aircraft")
            except IngestError as e:
                stats.failures += 1
                consecutive_failures += 1
                log(f"poll {stats.polls} failed: {e}")
                if consecutive_failures >= max_consecutive_failures:
                    log(f"aborting after {consecutive_failures} consecutive "
                        f"failures; partial session saved to {out_path}")
                    break

            if clock() - start >= duration_s:
                break
            wait = interval_s - (clock() - poll_started)
            if wait > 0:
                sleep(wait)

    return stats
