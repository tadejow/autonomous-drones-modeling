# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import matplotlib
matplotlib.use('TkAgg')

import time
import math
from typing import List, Tuple
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt

# ==========================================
# MATH & KINEMATICS UTILITIES
# ==========================================

def get_location_meters(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    """Calculates a new GPS coordinate based on a shift in meters (North, East)."""
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    print("Disabling battery failsafe for simulation...")
    vehicle.parameters['BATT_FS_LOW_ACT'] = 0 
    vehicle.parameters['BATT_FS_CRT_ACT'] = 0
    vehicle.parameters['WP_SPEED'] = 1000  # 10 m/s

    print("Waiting for GPS lock...")
    while not vehicle.is_armable: 
        time.sleep(1)

    base_gps = vehicle.location.global_relative_frame

    # Define the irregular polygon (North, East) relative to origin
    polygon: List[Tuple[float, float]] = [
        (120.0, -40.0),
        (110.0, 100.0),
        (-30.0, 110.0),
        (-40.0, -30.0)
    ]

    FLIGHT_ALTITUDE = 25.0
    SWATH_WIDTH = 15.0

    print("Calculating geometry intersections (Sweep-Line Algorithm)...")
    cmds = vehicle.commands
    cmds.clear()
    
    # Dummy home waypoint
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, base_gps.lat, base_gps.lon, FLIGHT_ALTITUDE))

    # 1. Bounding Box (North axis)
    min_n = min(p[0] for p in polygon)
    max_n = max(p[0] for p in polygon)

    current_n = min_n + (SWATH_WIDTH / 2.0)
    direction = 1 

    # 2. Sweep-line area scanning
    while current_n <= max_n:
        intersections_east = []
        
        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]
            
            n1, e1 = p1
            n2, e2 = p2
            
            if (n1 <= current_n < n2) or (n2 <= current_n < n1):
                e_intersect = e1 + (e2 - e1) * (current_n - n1) / (n2 - n1)
                intersections_east.append(e_intersect)
                
        if len(intersections_east) >= 2:
            intersections_east.sort()
            left_e = min(intersections_east)
            right_e = max(intersections_east)
            
            if direction == 1:
                wp1 = get_location_meters(base_gps, current_n, left_e)
                wp2 = get_location_meters(base_gps, current_n, right_e)
            else:
                wp1 = get_location_meters(base_gps, current_n, right_e)
                wp2 = get_location_meters(base_gps, current_n, left_e)
                
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp1.lat, wp1.lon, FLIGHT_ALTITUDE))
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp2.lat, wp2.lon, FLIGHT_ALTITUDE))
            
            direction *= -1 
            
        current_n += SWATH_WIDTH

    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
    cmds.upload()
    print("Mission uploaded successfully!")

    # 3. Flight execution & 3D Plotting
    print("\nArming and taking off (GUIDED mode)...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed: 
        time.sleep(1)

    vehicle.simple_takeoff(FLIGHT_ALTITUDE)
    while vehicle.location.global_relative_frame.alt < FLIGHT_ALTITUDE * 0.95: 
        time.sleep(1)

    print("Cruising altitude reached. Switching to AUTO mode...")
    vehicle.commands.next = 1
    vehicle.mode = VehicleMode("AUTO")

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x_hist, y_hist, z_hist = [], [], []

    poly_north = [p[0] for p in polygon] + [polygon[0][0]]
    poly_east = [p[1] for p in polygon] + [polygon[0][1]]
    poly_z = [0] * len(poly_north)

    while vehicle.mode.name == "AUTO":
        loc = vehicle.location.local_frame
        if loc.north is not None:
            x_hist.append(loc.north)
            y_hist.append(loc.east)
            z_hist.append(-loc.down)
            
            ax.clear()
            ax.set_title('Irregular Area 3D Scanning')
            
            ax.set_xlim([min_n - 20, max_n + 20])
            ax.set_ylim([min(poly_east) - 20, max(poly_east) + 20])
            ax.set_zlim([0, FLIGHT_ALTITUDE + 10])
            
            ax.set_xlabel('North (m)')
            ax.set_ylabel('East (m)')
            ax.set_zlabel('Altitude (m)')
            
            ax.plot(x_hist, y_hist, z_hist, color='orange', linewidth=2, label='Flight Trajectory')
            ax.scatter(x_hist[-1], y_hist[-1], z_hist[-1], color='red', s=50, label='Drone')
            ax.plot(poly_north, poly_east, poly_z, color='black', linewidth=3, linestyle='-', label='Boundary')
            
            ax.legend()
            plt.draw()
            plt.pause(0.1)

    print("\nMission completed. Returning to Launch (RTL).")
    vehicle.close()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
