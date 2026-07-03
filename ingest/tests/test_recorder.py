"""Recorder tests with injected clock/sleep/poll (no network, no waiting)."""
from ingest.net import IngestError
from ingest.recorder import record
from ingest.session import Measurement, read_session


class FakeTime:
    """Monotonic clock advanced only by sleep()."""

    def __init__(self):
        self.t = 0.0
        self.sleeps = []

    def clock(self):
        return self.t

    def sleep(self, seconds):
        # The recorder must never request a negative wait (poll overruns).
        assert seconds >= 0, f"negative sleep requested: {seconds}"
        self.sleeps.append(seconds)
        self.t += seconds


def meas(aid, ts):
    return Measurement(aircraft_id=aid, timestamp=ts, latitude=40.0,
                       longitude=-82.9, altitude=9000.0, velocity=200.0,
                       heading=90.0)


def test_recorder_dedupes_repeated_reports(tmp_path):
    # Live APIs re-report an aircraft's last state every poll; only rows with
    # a new (aircraft, timestamp) may reach the session file.
    batches = iter([
        [meas("aaa", 100.0), meas("bbb", 100.0)],
        [meas("aaa", 100.0), meas("bbb", 110.0)],  # aaa unchanged
        [meas("aaa", 100.0), meas("bbb", 110.0)],  # all unchanged
        [meas("aaa", 120.0)],
    ])
    ft = FakeTime()
    out = tmp_path / "session.csv"

    stats = record(out, poll=lambda: next(batches), duration_s=30.0,
                   interval_s=10.0, sleep=ft.sleep, clock=ft.clock,
                   log=lambda _msg: None)

    assert stats.polls == 4
    assert stats.written == 4  # 2 + 1 + 0 + 1
    assert stats.aircraft == {"aaa", "bbb"}
    rows = read_session(out)
    assert [(m.aircraft_id, m.timestamp) for m in rows] == [
        ("aaa", 100.0), ("bbb", 100.0), ("bbb", 110.0), ("aaa", 120.0)]


def test_recorder_drops_jittered_near_duplicates(tmp_path):
    # readsb-style timestamps (now - seen_pos) jitter a few tenths of a
    # second between polls for an unchanged position; the min-separation
    # dedupe must swallow them but keep genuinely new reports.
    batches = iter([
        [meas("aaa", 100.0)],
        [meas("aaa", 100.2)],   # same fix, +0.2 s jitter -> dropped
        [meas("aaa", 99.9)],    # same fix, -0.1 s jitter -> dropped
        [meas("aaa", 101.0)],   # new fix -> kept
    ])
    ft = FakeTime()
    out = tmp_path / "session.csv"

    stats = record(out, poll=lambda: next(batches), duration_s=30.0,
                   interval_s=10.0, min_separation_s=0.5, sleep=ft.sleep,
                   clock=ft.clock, log=lambda _msg: None)

    assert stats.written == 2
    assert [m.timestamp for m in read_session(out)] == [100.0, 101.0]


def test_recorder_aborts_after_consecutive_failures(tmp_path):
    def failing_poll():
        raise IngestError("HTTP 429")

    ft = FakeTime()
    out = tmp_path / "session.csv"
    stats = record(out, poll=failing_poll, duration_s=10_000.0,
                   interval_s=10.0, max_consecutive_failures=3,
                   sleep=ft.sleep, clock=ft.clock, log=lambda _msg: None)

    assert stats.polls == 3
    assert stats.failures == 3
    assert stats.written == 0
    assert read_session(out) == []  # header-only file is still valid


def test_recorder_recovers_when_a_poll_succeeds(tmp_path):
    calls = {"n": 0}

    def flaky_poll():
        calls["n"] += 1
        if calls["n"] % 2 == 1:
            raise IngestError("transient")
        return [meas("aaa", 100.0 + calls["n"])]

    ft = FakeTime()
    out = tmp_path / "session.csv"
    # fail, ok, fail, ok — never 2 consecutive failures, so no abort
    stats = record(out, poll=flaky_poll, duration_s=30.0, interval_s=10.0,
                   max_consecutive_failures=2, sleep=ft.sleep, clock=ft.clock,
                   log=lambda _msg: None)

    assert stats.polls == 4
    assert stats.failures == 2
    assert stats.written == 2


def test_recorder_paces_polls_with_interval(tmp_path):
    batches = iter([[meas("aaa", 100.0)], [meas("aaa", 110.0)],
                    [meas("aaa", 120.0)]])
    ft = FakeTime()
    record(tmp_path / "s.csv", poll=lambda: next(batches), duration_s=20.0,
           interval_s=10.0, sleep=ft.sleep, clock=ft.clock,
           log=lambda _msg: None)
    # poll and write are instant under the fake clock -> full-interval waits
    assert ft.sleeps == [10.0, 10.0]
