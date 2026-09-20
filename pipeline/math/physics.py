from dronekit import LocationGlobalRelative
import math

def get_location_meters(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    """
    Przesuwa koordynaty GPS o wektor w metrach.
    """
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)
