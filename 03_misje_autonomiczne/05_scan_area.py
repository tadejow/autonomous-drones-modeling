# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil

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

def clear_mission(vehicle) -> None:
    """Clears the current mission from the vehicle's memory."""
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()

def generate_lawnmower_mission(vehicle, start_loc: LocationGlobalRelative, width: float, length: float, swath_width: float, altitude: float) -> None:
    """
    Generates and uploads an optimal coverage path (Lawnmower algorithm).
    """
    print("Generating optimal scanning route (Boustrophedon path)...")
    cmds = vehicle.commands
    
    # 0. Required by MAVLink - add a dummy home point
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, start_loc.lat, start_loc.lon, altitude))

    current_east = 0.0
    direction = 1  # 1 for North-bound, -1 for South-bound

    # 1. Generate zig-zag pattern
    while current_east <= width:
        target_north = length if direction == 1 else 0.0
        wp_gps = get_location_meters(start_loc, target_north, current_east)
        
        # Note: autocontinue parameter is set to 1
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp_gps.lat, wp_gps.lon, altitude))
        
        current_east += swath_width
        direction *= -1 

    # 2. End mission and Return To Launch (RTL)
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))

    print("Uploading mission to autopilot...")
    cmds.upload()
    print(f"Mission uploaded successfully! Total waypoints: {cmds.count}")


# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    # Disable battery failsafe for simulation purposes
    vehicle.parameters['BATT_FS_LOW_ACT'] = 0
    vehicle.parameters['BATT_FS_CRT_ACT'] = 0

    print("Waiting for GPS lock...")
    while not vehicle.is_armable:
        time.sleep(1)

    start_location = vehicle.location.global_relative_frame

    # Mission parameters
    ALTITUDE = 20.0         
    FIELD_WIDTH = 80.0   
    FIELD_LENGTH = 100.0    
    SWATH_WIDTH = 15.0  

    clear_mission(vehicle)
    generate_lawnmower_mission(vehicle, start_location, FIELD_WIDTH, FIELD_LENGTH, SWATH_WIDTH, ALTITUDE)

    # Safe takeoff procedure in GUIDED mode
    print("\nArming motors and taking off (GUIDED mode)...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(ALTITUDE)
    while True:
        if vehicle.location.global_relative_frame.alt >= ALTITUDE * 0.95:
            print("Cruising altitude reached!")
            break
        time.sleep(1)

    # Hand over control to AUTO mode
    print("Switching to AUTO mode - Autopilot takes control over the mission!")
    vehicle.commands.next = 1 # Ensure it starts from the first actual waypoint
    vehicle.mode = VehicleMode("AUTO")

    # Monitoring loop
    while True:
        next_waypoint = vehicle.commands.next
        total_waypoints = vehicle.commands.count
        
        print(f"[Monitoring] Navigating to Waypoint: {next_waypoint} / {total_waypoints}")
        
        if vehicle.mode.name == "RTL":
            print("Scanning complete. Vehicle is returning to base!")
            break
        time.sleep(2)

    vehicle.close()
    print("Supervision ended. Connection closed.")

if __name__ == "__main__":
    main()
