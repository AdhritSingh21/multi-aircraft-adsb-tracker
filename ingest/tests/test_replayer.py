"""Replayer timing tests and session-file round-trip."""
import pytest

from ingest.replayer import replay
from ingest.session import (CSV_HEADER, Measurement, SessionWriter,
                            parse_csv_line, read_session)


def meas(aid, ts):
    return Measurement(aircraft_id=aid, timestamp=ts, latitude=40.0,
                       longitude=-82.9, altitude=9000.0, velocity=200.0,
                       heading=90.0)


def write_session(path, measurements):
    with SessionWriter(path) as w:
        w.write(measurements)


def test_replay_orders_and_paces_measurements(tmp_path):
    path = tmp_path / "s.csv"
    # deliberately out of order in the file; two reports share t=100
    write_session(path, [meas("b", 105.0), meas("a", 100.0),
                         meas("c", 100.0), meas("d", 117.0)])

    sleeps, emitted = [], []
    count = replay(path, emit=emitted.append, speed=2.0, sleep=sleeps.append)

    assert count == 4
    assert [m.timestamp for m in emitted] == [100.0, 100.0, 105.0, 117.0]
    # gaps: 0 (same ts), 5/2, 12/2 — no sleep between same-timestamp reports
    assert sleeps == pytest.approx([2.5, 6.0])


def test_replay_caps_long_gaps_in_wall_clock_terms(tmp_path):
    path = tmp_path / "s.csv"
    write_session(path, [meas("a", 0.0), meas("a", 300.0)])

    sleeps = []
    # speed=2 distinguishes cap-after-scaling (5.0, the intent: max_gap is a
    # WALL-clock bound) from cap-before-scaling (which would give 2.5).
    replay(path, emit=lambda m: None, speed=2.0, max_gap_s=5.0,
           sleep=sleeps.append)
    assert sleeps == [5.0]


def test_replay_rejects_nonpositive_speed(tmp_path):
    path = tmp_path / "s.csv"
    write_session(path, [meas("a", 0.0)])
    with pytest.raises(ValueError):
        replay(path, emit=lambda m: None, speed=0.0)


def test_session_round_trip(tmp_path):
    path = tmp_path / "s.csv"
    original = [meas("aaa", 1751500000.0), meas("bbb", 1751500010.0)]
    write_session(path, original)

    loaded = read_session(path)
    assert loaded == original  # exact: writer precision covers these values


def test_parse_csv_line_tolerates_junk():
    assert parse_csv_line(CSV_HEADER) is None          # header
    assert parse_csv_line("# comment") is None
    assert parse_csv_line("") is None
    assert parse_csv_line("abc,1.0,40.0") is None      # short row
    assert parse_csv_line("abc,nan?,40,-82,1,2,x") is None
    m = parse_csv_line("abc123,10.0,40.1,-82.9,10500.0,220.0,90.0")
    assert m is not None and m.aircraft_id == "abc123"


def test_session_writer_appends_without_duplicate_header(tmp_path):
    path = tmp_path / "s.csv"
    write_session(path, [meas("aaa", 1.0)])
    write_session(path, [meas("bbb", 2.0)])  # reopen + append

    text = path.read_text(encoding="utf-8")
    assert text.count(CSV_HEADER) == 1
    assert len(read_session(path)) == 2
