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
from typing import Tuple
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt

# ==========================================
# MATH UTILITIES
# ==========================================

def get_location_meters(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def get_distance_ned(loc1: LocationGlobalRelative, loc2: LocationGlobalRelative) -> Tuple[float, float]:
    """Returns the distance in meters (North, East) between two GPS points."""
    earth_radius = 6378137.0
    d_lat = math.radians(loc2.lat - loc1.lat)
    d_lon = math.radians(loc2.lon - loc1.lon)
    
    d_north = d_lat * earth_radius
    d_east = d_lon * earth_radius * math.cos(math.radians(loc1.lat))
    return d_north, d_east

def draw_3d_building(ax, center_n: float, center_e: float, size_n: float, size_e: float, height: float) -> None:
    """Renders a 3D block representing a building on the matplotlib axis."""
    n1, n2 = center_n - size_n/2, center_n + size_n/2
    e1, e2 = center_e - size_e/2, center_e + size_e/2
    X = [n1, n2, n2, n1, n1]
    Y = [e1, e1, e2, e2, e1]
    Z_bottom = [0, 0, 0, 0, 0]
    Z_top = [height, height, height, height, height]
    
    ax.plot(X, Y, Z_bottom, color='gray')
    ax.plot(X, Y, Z_top, color='gray')
    for i in range(4):
        ax.plot([X[i], X[i]], [Y[i], Y[i]], [0, height], color='gray')

# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    vehicle.parameters['BATT_FS_LOW_ACT'] = 0 
    vehicle.parameters['BATT_FS_CRT_ACT'] = 0
    vehicle.parameters['WP_SPEED'] = 700  # 7 m/s for better inspection photos

    print("Waiting for GPS lock...")
    while not vehicle.is_armable: 
        time.sleep(1)
    base_gps = vehicle.location.global_relative_frame

    # TARGET DEFINITION FROM MAVPROXY SATELLITE MAP
    TARGET_LAT = -35.36266675
    TARGET_LON = 149.16579263
    target_gps = LocationGlobalRelative(TARGET_LAT, TARGET_LON, 0)

    # Calculate Cartesian offset of the building relative to the drone
    BUILDING_N, BUILDING_E = get_distance_ned(base_gps, target_gps)

    # Physical parameters
    BUILDING_SIZE_N = 18.0
    BUILDING_SIZE_E = 14.0 
    BUILDING_HEIGHT = 12.0

    ORBIT_RADIUS = 22.0
    ALT_1 = 8.0   
    ALT_2 = 16.0  
    ANGLE_STEP = 30 

    print(f"Generating mission... Target located {BUILDING_N:.1f}m N and {BUILDING_E:.1f}m E.")
    vehicle.commands.clear()
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, base_gps.lat, base_gps.lon, ALT_1))

    for altitude in [ALT_1, ALT_2]:
        for angle_deg in range(0, 360 + ANGLE_STEP, ANGLE_STEP):
            angle_rad = math.radians(angle_deg)
            
            # Offset is applied RELATIVE TO THE BUILDING'S GPS
            x = ORBIT_RADIUS * math.cos(angle_rad)
            y = ORBIT_RADIUS * math.sin(angle_rad)
            
            wp = get_location_meters(target_gps, x, y)
            vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, altitude))

    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
    vehicle.commands.upload()

    print("\nArming and taking off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed: 
        time.sleep(1)

    vehicle.simple_takeoff(ALT_1)
    while vehicle.location.global_relative_frame.alt < ALT_1 * 0.95: 
        time.sleep(1)

    print("Cruising altitude reached. Switching to AUTO mode...")
    vehicle.commands.next = 1
    vehicle.mode = VehicleMode("AUTO")

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = [], [], []

    while vehicle.mode.name == "AUTO":
        loc = vehicle.location.local_frame
        if loc.north is not None:
            xs.append(loc.north); ys.append(loc.east); zs.append(-loc.down)
            
            ax.clear()
            ax.set_title('3D Building Inspection (Satellite Coords)')
            
            min_n = min(min(xs), BUILDING_N - ORBIT_RADIUS) - 10
            max_n = max(max(xs), BUILDING_N + ORBIT_RADIUS) + 10
            min_e = min(min(ys), BUILDING_E - ORBIT_RADIUS) - 10
            max_e = max(max(ys), BUILDING_E + ORBIT_RADIUS) + 10
            
            ax.set_xlim([min_n, max_n])
            ax.set_ylim([min_e, max_e])
            ax.set_zlim([0, ALT_2 + 10])
            
            ax.set_xlabel('North (m)')
            ax.set_ylabel('East (m)')
            ax.set_zlabel('Altitude (m)')
            
            draw_3d_building(ax, BUILDING_N, BUILDING_E, BUILDING_SIZE_N, BUILDING_SIZE_E, BUILDING_HEIGHT)
            ax.plot(xs, ys, zs, color='blue', linewidth=2, label='Flight Trajectory')
            ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=50, label='Drone')
            
            ax.legend()
            plt.draw()
            plt.pause(0.1)

    print("\nMission completed. Returning to Launch.")
    vehicle.close()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
