"""Normalizer tests for the readsb-style v2 point API (adsb.lol et al.)."""
import pytest

from ingest.readsb import FT_TO_M, KN_TO_MS, normalize_readsb

# Field shapes taken from a real api.adsb.lol response (2026-07-03).
PAYLOAD = {
    "now": 1783103091501,  # unix MILLISECONDS
    "total": 5,
    "ac": [
        # normal airborne aircraft, fresh position
        {"hex": "A8FB06", "flight": "N678SF  ", "alt_baro": 30000,
         "alt_geom": 31800, "gs": 295.4, "track": 357.09,
         "lat": 40.017838, "lon": -85.12262, "seen_pos": 0.0, "seen": 0.0},
        # position 2.5 s old -> timestamp backdated by seen_pos
        {"hex": "a0b7b8", "alt_baro": 32975, "gs": 469.7, "track": 66.27,
         "lat": 40.41, "lon": -84.99, "seen_pos": 2.5},
        # on the ground: alt_baro is the STRING "ground"
        {"hex": "abc001", "alt_baro": "ground", "gs": 3.0, "track": 180.0,
         "lat": 39.99, "lon": -82.88, "seen_pos": 0.0},
        # no position fields at all (mode-S only) -> skipped
        {"hex": "abc002", "alt_baro": 35000, "gs": 400.0},
        # stale position (older than MAX_POSITION_AGE_S) -> skipped
        {"hex": "abc003", "alt_baro": 20000, "gs": 250.0, "track": 90.0,
         "lat": 40.2, "lon": -83.3, "seen_pos": 300.0},
        # null gs/track -> defaults, aircraft kept
        {"hex": "abc004", "alt_baro": None, "alt_geom": 10000,
         "gs": None, "track": None, "lat": 40.1, "lon": -83.1,
         "seen_pos": 1.0},
    ],
}


def test_units_converted_to_si():
    m = normalize_readsb(PAYLOAD)[0]
    assert m.aircraft_id == "a8fb06"  # lowercased
    assert m.altitude == pytest.approx(30000 * FT_TO_M)   # feet -> meters
    assert m.velocity == pytest.approx(295.4 * KN_TO_MS)  # knots -> m/s
    assert m.heading == pytest.approx(357.09)
    assert m.latitude == pytest.approx(40.017838)
    assert m.longitude == pytest.approx(-85.12262)


def test_timestamp_is_seconds_backdated_by_position_age():
    ms = normalize_readsb(PAYLOAD)
    now_s = 1783103091501 / 1000.0
    # Exact equality: pytest.approx's default relative tolerance is ~1800 s
    # at epoch scale, which would make these assertions vacuous.
    assert ms[0].timestamp == round(now_s, 1)
    assert ms[1].timestamp == round(now_s - 2.5, 1)


def test_ground_missing_position_and_stale_rows_filtered():
    ids = [m.aircraft_id for m in normalize_readsb(PAYLOAD)]
    assert "abc001" not in ids  # grounded
    assert "abc002" not in ids  # no position
    assert "abc003" not in ids  # stale position
    assert "abc001" in [m.aircraft_id
                        for m in normalize_readsb(PAYLOAD, include_ground=True)]


def test_null_altitude_falls_back_to_geometric():
    m = [m for m in normalize_readsb(PAYLOAD) if m.aircraft_id == "abc004"][0]
    assert m.altitude == pytest.approx(10000 * FT_TO_M)
    assert m.velocity == 0.0
    assert m.heading == 0.0


def test_empty_payloads():
    assert normalize_readsb({}) == []
    assert normalize_readsb({"now": 1, "ac": None}) == []
