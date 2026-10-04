"""Pure-Python conversions between WGS84 coordinates and a local NED frame.

The module has no third-party dependencies, so it can be imported (and unit
tested) on machines without DroneKit or ArduPilot installed.

The local frame uses the equirectangular approximation around a fixed origin
``(lat0, lon0, alt0)``::

    N = R * (lat - lat0)
    E = R * cos(lat0) * (lon - lon0)
    D = -(alt - alt0)

Angles are in radians in the formulas above. Using ``cos(lat0)`` (a constant)
instead of ``cos(lat)`` keeps the transform linear, hence exactly invertible.
Over a few hundred metres the error versus the great-circle distance is at the
millimetre level.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

EARTH_RADIUS_M: float = 6378137.0
"""Equatorial radius of the WGS84 ellipsoid (the same constant as in physics.py)."""

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class GeoOrigin:
    """Origin of a local NED frame (degrees, degrees, metres AMSL)."""

    lat_deg: float
    lon_deg: float
    alt_amsl_m: float = 0.0


def gps_to_ned(origin: GeoOrigin, lat_deg: float, lon_deg: float, alt_amsl_m: float) -> Vec3:
    """Converts a WGS84 position into metres (North, East, Down) relative to ``origin``."""
    d_lat = math.radians(lat_deg - origin.lat_deg)
    d_lon = math.radians(lon_deg - origin.lon_deg)
    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(math.radians(origin.lat_deg))
    down = -(alt_amsl_m - origin.alt_amsl_m)
    return (north, east, down)


def ned_to_gps(origin: GeoOrigin, north: float, east: float, down: float) -> Vec3:
    """Inverse of :func:`gps_to_ned`. Returns ``(lat_deg, lon_deg, alt_amsl_m)``."""
    d_lat = north / EARTH_RADIUS_M
    d_lon = east / (EARTH_RADIUS_M * math.cos(math.radians(origin.lat_deg)))
    return (
        origin.lat_deg + math.degrees(d_lat),
        origin.lon_deg + math.degrees(d_lon),
        origin.alt_amsl_m - down,
    )


def ned_to_enu(vec: Vec3) -> Vec3:
    """Maps NED to ENU (East, North, Up), a right-handed frame suitable for 3D plots."""
    north, east, down = vec
    return (east, north, -down)


def haversine_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """Great-circle distance on a sphere of radius ``EARTH_RADIUS_M`` (reference for tests)."""
    phi1, phi2 = math.radians(lat1_deg), math.radians(lat2_deg)
    d_phi = phi2 - phi1
    d_lambda = math.radians(lon2_deg - lon1_deg)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))
