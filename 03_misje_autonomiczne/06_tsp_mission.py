# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import time
import math
import random
from typing import List, Tuple
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil

# ==========================================
# MATH & UTILITIES
# ==========================================

def get_location_meters(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    """Calculates a new GPS coordinate based on a shift in meters (North, East)."""
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def distance_2d(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculates Euclidean distance between two (x, y) points in meters."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def clear_mission(vehicle) -> None:
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()

# ==========================================
# TSP ALGORITHMS
# ==========================================

def generate_random_points(num_points: int, max_north: float, max_east: float) -> List[Tuple[float, float]]:
    """Generates N random points (North, East) within the specified operational radius."""
    points = []
    for _ in range(num_points):
        n = random.uniform(-max_north, max_north)
        e = random.uniform(-max_east, max_east)
        points.append((n, e))
    return points

def solve_tsp_nearest_neighbor(start_point: Tuple[float, float], points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Solves the Traveling Salesperson Problem using the Greedy Nearest Neighbor approach.
    Returns the ordered list of points to visit.
    """
    print("\nSolving TSP (Nearest Neighbor heuristic)...")
    unvisited = points.copy()
    current_point = start_point
    tour = []
    total_distance = 0.0

    while unvisited:
        # Find the closest point
        nearest = min(unvisited, key=lambda p: distance_2d(current_point, p))
        dist = distance_2d(current_point, nearest)
        
        unvisited.remove(nearest)
        tour.append(nearest)
        
        total_distance += dist
        current_point = nearest

    # Add distance back to the base
    total_distance += distance_2d(current_point, start_point)
    print(f"Calculated route length: ~{total_distance:.1f} meters.")
    
    return tour

# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    vehicle.parameters['BATT_FS_LOW_ACT'] = 0
    vehicle.parameters['BATT_FS_CRT_ACT'] = 0

    print("Waiting for GPS lock...")
    while not vehicle.is_armable:
        time.sleep(1)

    base_gps = vehicle.location.global_relative_frame
    ALTITUDE = 15.0

    # 1. Define the logistics problem
    base_cartesian = (0.0, 0.0)
    NUM_PACKAGES = 8
    OPERATIONAL_RADIUS = 100.0 # 100 meters

    print(f"Generating {NUM_PACKAGES} random delivery points within {OPERATIONAL_RADIUS}m radius...")
    delivery_points = generate_random_points(NUM_PACKAGES, OPERATIONAL_RADIUS, OPERATIONAL_RADIUS)

    # 2. Solve TSP
    optimized_route = solve_tsp_nearest_neighbor(base_cartesian, delivery_points)

    # 3. Create the mission
    print("Uploading mission to autopilot...")
    clear_mission(vehicle)
    cmds = vehicle.commands

    # Dummy home waypoint
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, base_gps.lat, base_gps.lon, ALTITUDE))

    # Add optimized route
    for d_north, d_east in optimized_route:
        wp_gps = get_location_meters(base_gps, d_north, d_east)
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp_gps.lat, wp_gps.lon, ALTITUDE))

    # Return to base
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
    cmds.upload()

    # 4. Execute the mission
    print("\nArming motors and taking off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(ALTITUDE)
    while True:
        if vehicle.location.global_relative_frame.alt >= ALTITUDE * 0.95:
            break
        time.sleep(1)

    print("Switching to AUTO mode - Executing TSP route!")
    vehicle.commands.next = 1
    vehicle.mode = VehicleMode("AUTO")

    while True:
        next_wp = vehicle.commands.next
        total_wps = vehicle.commands.count
        
        # Calculate pretty progress (subtracting home point and RTL command)
        current_delivery = next_wp - 1
        max_deliveries = total_wps - 2
        if current_delivery > max_deliveries: 
            current_delivery = max_deliveries
        
        print(f"[Monitoring] Delivery progress: {current_delivery}/{max_deliveries}")
        
        if vehicle.mode.name == "RTL":
            print("All points visited. Returning to base!")
            break
        time.sleep(3)

    vehicle.close()
    print("Simulation finished.")

if __name__ == "__main__":
    main()
