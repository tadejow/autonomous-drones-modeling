from dronekit import LocationGlobalRelative

from pipeline.math.geodesy import GeoOrigin, ned_to_gps


def get_location_meters(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    """
    Przesuwa koordynaty GPS o wektor w metrach.
    """
    origin = GeoOrigin(original_location.lat, original_location.lon)
    new_lat, new_lon, _ = ned_to_gps(origin, d_north, d_east, 0.0)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)
