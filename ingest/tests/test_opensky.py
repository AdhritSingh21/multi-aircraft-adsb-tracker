"""Normalizer tests against a canned OpenSky /states/all payload."""
import pytest

from ingest.opensky import normalize_states

# Realistic payload: field order per the OpenSky REST docs
# (longitude at index 5, latitude at index 6).
PAYLOAD = {
    "time": 1751500000,
    "states": [
        # normal airborne aircraft
        ["A1B2C3", "UAL123  ", "United States", 1751499998, 1751499999,
         -82.99, 40.01, 10668.0, False, 231.5, 88.4, 0.3, None, 10972.8,
         "1200", False, 0],
        # null altitude/velocity/track -> defaults, not skipped; time_position
        # deliberately far from last_contact to pin which one is used
        ["b2d4e6", None, "Canada", 1751499900, 1751499997,
         -83.20, 39.90, None, False, None, None, None, None, None,
         None, False, 0],
        # on ground -> excluded unless include_ground
        ["c3f5a7", "GND1", "US", 1751499998, 1751499998,
         -82.90, 40.00, None, True, 5.0, 180.0, 0.0, None, None,
         None, False, 0],
        # no position -> never usable
        ["d4e6f8", "NOPOS", "US", 1751499998, 1751499998,
         None, None, 9000.0, False, 200.0, 90.0, 0.0, None, None,
         None, False, 0],
        # retained stale position: lat/lon present but time_position null.
        # OpenSky nulls time_position when the fix is >15 s old; stamping it
        # with last_contact would create frozen-but-"fresh" tracks -> skip.
        ["e6f8aa", "STALE", "US", None, 1751499999,
         -83.05, 40.20, 8000.0, False, 190.0, 45.0, 0.0, None, None,
         None, False, 0],
        # truncated vector -> skipped
        ["e5f7a9", "SHORT", "US", 1751499998, 1751499998, -82.0, 40.0],
    ],
}


def test_normalizes_airborne_aircraft():
    ms = normalize_states(PAYLOAD)
    assert [m.aircraft_id for m in ms] == ["a1b2c3", "b2d4e6"]

    ual = ms[0]
    # id lowercased, lat/lon NOT swapped, baro altitude preferred
    assert ual.latitude == pytest.approx(40.01)
    assert ual.longitude == pytest.approx(-82.99)
    assert ual.timestamp == 1751499998.0  # exact: epoch-scale approx is vacuous
    assert ual.altitude == pytest.approx(10668.0)
    assert ual.velocity == pytest.approx(231.5)
    assert ual.heading == pytest.approx(88.4)


def test_timestamp_comes_from_time_position_not_last_contact():
    m = normalize_states(PAYLOAD)[1]
    assert m.timestamp == 1751499900.0  # exact; last_contact is ...997


def test_null_fields_default_instead_of_dropping_the_aircraft():
    m = normalize_states(PAYLOAD)[1]
    assert m.altitude == 0.0
    assert m.velocity == 0.0
    assert m.heading == 0.0


def test_retained_position_without_time_position_is_skipped():
    ids = [m.aircraft_id for m in normalize_states(PAYLOAD)]
    assert "e6f8aa" not in ids


def test_ground_traffic_excluded_by_default_included_on_request():
    assert "c3f5a7" not in [m.aircraft_id for m in normalize_states(PAYLOAD)]
    with_ground = normalize_states(PAYLOAD, include_ground=True)
    assert "c3f5a7" in [m.aircraft_id for m in with_ground]


def test_empty_and_null_states():
    assert normalize_states({"time": 1, "states": None}) == []
    assert normalize_states({"time": 1, "states": []}) == []
    assert normalize_states({}) == []
