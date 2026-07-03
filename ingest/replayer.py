"""Replayer: emit a recorded session with original (or scaled) timing.

This is the repeatable-demo mode: the same session file produces the same
measurement stream every run. The M4 backend will consume this to feed the
tracker + WebSocket clients as if the data were arriving live.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Callable, Optional, Union

from .session import Measurement, read_session


def replay(path: Union[str, Path],
           emit: Callable[[Measurement], None],
           speed: float = 1.0,
           max_gap_s: Optional[float] = None,
           sleep: Callable[[float], None] = time.sleep) -> int:
    """Emit each measurement in timestamp order, sleeping the inter-report
    gap (divided by ``speed``) between distinct timestamps.

    ``max_gap_s`` caps a single wait (wall-clock, after scaling) so a session
    with a long quiet stretch doesn't stall a demo. Returns the number of
    measurements emitted.
    """
    if speed <= 0:
        raise ValueError("speed must be > 0")

    measurements = read_session(path)
    previous_ts: Optional[float] = None
    for m in measurements:
        if previous_ts is not None and m.timestamp > previous_ts:
            gap = (m.timestamp - previous_ts) / speed
            if max_gap_s is not None:
                gap = min(gap, max_gap_s)
            if gap > 0:
                sleep(gap)
        emit(m)
        previous_ts = m.timestamp
    return len(measurements)
