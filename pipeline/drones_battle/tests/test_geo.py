import math

import pytest

from pipeline.math.geodesy import GeoOrigin, gps_to_ned, haversine_m, ned_to_enu, ned_to_gps

ORIGIN = GeoOrigin(-35.36335, 149.165, 584.0)


def test_round_trip_is_identity() -> None:
    for ned in [(0.0, 0.0, 0.0), (150.0, -5.0, -15.0), (-40.0, 60.0, 3.0)]:
        lat, lon, alt = ned_to_gps(ORIGIN, *ned)
        # Micrometre tolerance: rounding of the float degrees.
        assert gps_to_ned(ORIGIN, lat, lon, alt) == pytest.approx(ned, abs=1e-6)


def test_distance_matches_haversine_within_a_centimetre() -> None:
    for north, east in [(150.0, 0.0), (0.0, 120.0), (200.0, -60.0)]:
        lat, lon, _ = ned_to_gps(ORIGIN, north, east, 0.0)
        assert math.hypot(north, east) == pytest.approx(
            haversine_m(ORIGIN.lat_deg, ORIGIN.lon_deg, lat, lon), abs=0.01
        )


def test_specification_coordinates() -> None:
    # The specification's -35.3635 is ~167 m from -35.3620; the plan uses -35.36335 (150 m).
    north, _, _ = gps_to_ned(GeoOrigin(-35.3635, 149.165), -35.3620, 149.165, 0.0)
    assert north == pytest.approx(167.0, abs=0.1)
    north, _, _ = gps_to_ned(ORIGIN, -35.3620, 149.165, 584.0)
    assert north == pytest.approx(150.3, abs=0.1)


def test_altitude_is_down_positive() -> None:
    assert gps_to_ned(ORIGIN, ORIGIN.lat_deg, ORIGIN.lon_deg, 599.0)[2] == pytest.approx(-15.0)


def test_enu_is_right_handed_mapping() -> None:
    assert ned_to_enu((1.0, 2.0, 3.0)) == (2.0, 1.0, -3.0)
