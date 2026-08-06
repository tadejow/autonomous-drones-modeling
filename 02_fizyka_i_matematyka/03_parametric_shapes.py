# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import time
import math
from typing import List, Tuple
from dronekit import connect, VehicleMode, LocationGlobalRelative

# ==========================================
# MATH & KINEMATICS UTILITIES
# ==========================================

def get_location_metres(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    """Calculates a new GPS coordinate based on a shift in meters (North, East)."""
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def get_distance_metres(loc1: LocationGlobalRelative, loc2: LocationGlobalRelative) -> float:
    """Calculates the Euclidean distance in meters between two GPS points."""
    d_lat = loc2.lat - loc1.lat
    d_lon = loc2.lon - loc1.lon
    return math.sqrt((d_lat**2) + (d_lon**2)) * 1.113195e5

def arm_and_takeoff(vehicle, target_altitude: float) -> None:
    print("Arming and taking off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    vehicle.simple_takeoff(target_altitude)
    while True:
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            break
        time.sleep(1)

# ==========================================
# PARAMETRIC TRAJECTORY GENERATORS
# ==========================================

def generate_circle(radius: float = 15.0, num_points: int = 36) -> List[Tuple[float, float]]:
    """Circle: x = R*cos(t), y = R*sin(t)"""
    points = []
    for i in range(num_points):
        t = (2 * math.pi * i) / num_points
        x = radius * math.cos(t) - radius # Offset so it starts at (0,0)
        y = radius * math.sin(t)
        points.append((x, y))
    return points

def generate_infinity(scale: float = 15.0, num_points: int = 50) -> List[Tuple[float, float]]:
    """Lemniscate of Gerono (Figure-8): x = scale*sin(t), y = scale*sin(t)*cos(t)"""
    points = []
    for i in range(num_points):
        t = (2 * math.pi * i) / num_points
        x = scale * math.sin(t)
        y = scale * math.sin(t) * math.cos(t)
        points.append((x, y))
    return points

def generate_heart(scale: float = 1.0, num_points: int = 60) -> List[Tuple[float, float]]:
    """Cardioid (Heart): Math parametric equations."""
    points = []
    for i in range(num_points):
        t = (2 * math.pi * i) / num_points
        x_raw = 16 * (math.sin(t) ** 3)
        y_raw = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
        
        # Multiply by scale and orient North=Y, East=X
        north = y_raw * scale
        east = x_raw * scale
        points.append((north - (12*scale), east)) # Offset to start smoothly
    return points

# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("=== TRAJECTORY GENERATOR ===")
    print("1. Circle")
    print("2. Infinity (Figure-8)")
    print("3. Heart (Cardioid)")
    choice = input("Select a shape (1/2/3): ")

    if choice == '1':
        trajectory = generate_circle(radius=15.0)
        shape_name = "Circle"
    elif choice == '2':
        trajectory = generate_infinity(scale=20.0)
        shape_name = "Infinity"
    elif choice == '3':
        trajectory = generate_heart(scale=1.5)
        shape_name = "Heart"
    else:
        print("Unknown choice, defaulting to Circle.")
        trajectory = generate_circle(radius=15.0)
        shape_name = "Circle"

    print(f"\nConnecting to vehicle (Preparing for: {shape_name})...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    arm_and_takeoff(vehicle, 15.0)
    origin_point = vehicle.location.global_relative_frame
    vehicle.airspeed = 8.0 

    print(f"\nExecuting trajectory: {shape_name}")
    print(f"Total waypoints to navigate: {len(trajectory)}")

    for idx, (d_north, d_east) in enumerate(trajectory):
        target_gps = get_location_metres(origin_point, d_north, d_east)
        vehicle.simple_goto(target_gps)
        
        # Tight control loop with 2.0m tolerance for smooth cornering
        while True:
            current_pos = vehicle.location.global_relative_frame
            distance = get_distance_metres(current_pos, target_gps)
            
            if distance <= 2.0:
                break
            time.sleep(0.1) 

    print("\nTrajectory completed! Returning to Launch (RTL)...")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()
    print("Mission finished.")

if __name__ == "__main__":
    main()
