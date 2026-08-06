# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative

# ==========================================
# MATH & KINEMATICS UTILITIES
# ==========================================

def get_location_metres(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    """
    Calculates a new GPS coordinate based on a shift in meters (North, East) from an original point.
    Uses spherical earth approximation.
    """
    earth_radius = 6378137.0 # Earth's radius in meters
    
    # Coordinate offsets in radians
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    
    # New positions in decimal degrees
    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)
    
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def get_distance_metres(loc1: LocationGlobalRelative, loc2: LocationGlobalRelative) -> float:
    """
    Calculates the Euclidean distance in meters between two GPS points using a simplified Haversine formula.
    """
    d_lat = loc2.lat - loc1.lat
    d_lon = loc2.lon - loc1.lon
    return math.sqrt((d_lat**2) + (d_lon**2)) * 1.113195e5

def arm_and_takeoff(vehicle, target_altitude: float) -> None:
    """
    Standard procedure to arm the vehicle and takeoff to a specified altitude.
    """
    print("Waiting for vehicle initialization...")
    while not vehicle.is_armable:
        time.sleep(1)
        
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
        
    print(f"Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)
    
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        if current_altitude >= target_altitude * 0.95:
            print("Cruising altitude reached.")
            break
        time.sleep(1)

# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    # 1. Takeoff
    arm_and_takeoff(vehicle, 10.0)

    # Save the starting point as origin (0,0)
    start_point = vehicle.location.global_relative_frame

    # 2. Define the trajectory (A square with side length L)
    L = 45.0
    trajectory_points = [
        (L, 0),   # Point 1: L meters North
        (L, L),   # Point 2: L meters North, L meters East
        (0, L),   # Point 3: 0 meters North, L meters East
        (0, 0)    # Point 4: Back to the origin
    ]

    print("\nStarting trajectory execution (Square)...")
    vehicle.airspeed = 5.0 # Set default cruising speed to 5 m/s

    # 3. Control loop iterating through the points
    for idx, (d_north, d_east) in enumerate(trajectory_points):
        print(f"-> Navigating to Point {idx+1} (North: {d_north}m, East: {d_east}m)")
        
        # Convert Cartesian offset to GPS coordinates
        target_gps = get_location_metres(start_point, d_north, d_east)
        
        # Send navigation command
        vehicle.simple_goto(target_gps)
        
        # Proximity checking loop (Tolerance: 1.5 meters)
        while True:
            current_pos = vehicle.location.global_relative_frame
            distance_to_target = get_distance_metres(current_pos, target_gps)
            
            if distance_to_target <= 1.5:
                print(f"   Point {idx+1} reached!")
                time.sleep(2) # Short hover at the corner
                break
            time.sleep(0.5)

    # 4. End of mission
    print("\nTrajectory completed. Returning to launch (RTL)...")
    vehicle.mode = VehicleMode("RTL")

    vehicle.close()
    print("Script finished.")

if __name__ == "__main__":
    main()
