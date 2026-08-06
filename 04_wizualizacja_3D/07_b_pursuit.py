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
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import matplotlib.pyplot as plt

# ==========================================
# MATH & NETWORKING UTILITIES
# ==========================================

def get_location_meters(original_location: LocationGlobalRelative, d_north: float, d_east: float) -> LocationGlobalRelative:
    earth_radius = 6378137.0
    d_lat = d_north / earth_radius
    d_lon = d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180.0))
    new_lat = original_location.lat + (d_lat * 180.0 / math.pi)
    new_lon = original_location.lon + (d_lon * 180.0 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def send_ned_velocity(vehicle, velocity_x: float, velocity_y: float, velocity_z: float) -> None:
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def spoof_adsb_target(vehicle, target_gps: LocationGlobalRelative) -> None:
    """Spoofs an ADS-B transponder signal to render the target on the GCS map."""
    flags = 1 | 2 | 4 | 8 | 16 | 32
    msg = vehicle.message_factory.adsb_vehicle_encode(
        ICAO_address=12345, 
        lat=int(target_gps.lat * 1e7), 
        lon=int(target_gps.lon * 1e7), 
        altitude_type=0, 
        altitude=int(target_gps.alt * 1000), 
        heading=0, 
        hor_velocity=0, 
        ver_velocity=0, 
        callsign=b"UFO-1", 
        emitter_type=10, 
        tslc=1, 
        flags=flags,
        squawk=1200
    )
    vehicle.send_mavlink(msg)

# ==========================================
# MAIN PROGRAM LOGIC
# ==========================================

def main():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    
    while not vehicle.is_armable: 
        time.sleep(1)

    base_gps = vehicle.location.global_relative_frame

    DRONE_SPEED = 15.0 # m/s
    print(f"Setting maximum pursuit groundspeed to: {DRONE_SPEED} m/s")
    vehicle.groundspeed = DRONE_SPEED 

    print("Taking off to 10m...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed: 
        time.sleep(1)
        
    vehicle.simple_takeoff(10.0)
    while vehicle.location.global_relative_frame.alt < 9.5: 
        time.sleep(1)

    # Initial target (UFO) state
    target_x, target_y, target_z = 150.0, 100.0, 30.0
    target_vx, target_vy, target_vz = 0.25, 1.0, -0.2 

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    drone_xs, drone_ys, drone_zs = [], [], []
    target_xs, target_ys, target_zs = [], [], []

    print("Pursuit initiated! Check the MAVProxy map and 3D Plot.")
    last_time = time.time()

    while True:
        # 1. Real-time synchronization
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        if dt > 1.0 or dt == 0: dt = 0.1 
        
        # 2. Read drone position
        loc = vehicle.location.local_frame
        if loc.north is None: continue
        dx, dy, dz = loc.north, loc.east, -loc.down
        
        # 3. Update target physics
        target_x += target_vx * dt
        target_y += target_vy * dt
        target_z += target_vz * dt
        
        # Broadcast ADS-B
        target_gps = get_location_meters(base_gps, target_x, target_y)
        target_gps.alt = target_z
        spoof_adsb_target(vehicle, target_gps)
        
        # 4. PREDICTIVE INTERCEPT ALGORITHM (Lead Pursuit)
        dist_x = target_x - dx
        dist_y = target_y - dy
        dist_z = target_z - dz
        
        v_target_sq = target_vx**2 + target_vy**2 + target_vz**2
        v_drone_sq = DRONE_SPEED**2
        
        # Quadratic equation coefficients for collision time
        a = v_drone_sq - v_target_sq
        b = 2.0 * (dist_x * target_vx + dist_y * target_vy + dist_z * target_vz)
        c = -(dist_x**2 + dist_y**2 + dist_z**2)
        
        discriminant = b**2 - 4.0 * a * c
        t_intercept = 0.0
        
        if discriminant >= 0 and a > 0:
            t_intercept = (b + math.sqrt(discriminant)) / (2.0 * a)
            if t_intercept < 0: t_intercept = 0.0
        
        dir_x = (target_x + target_vx * t_intercept) - dx
        dir_y = (target_y + target_vy * t_intercept) - dy
        dir_z = (target_z + target_vz * t_intercept) - dz
        
        real_distance = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)
        
        if real_distance < 4.0:
            print("TARGET INTERCEPTED!")
            send_ned_velocity(vehicle, 0.0, 0.0, 0.0)
            break
            
        vector_len = math.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
        if vector_len > 0:
            cmd_vx = (dir_x / vector_len) * DRONE_SPEED
            cmd_vy = (dir_y / vector_len) * DRONE_SPEED
            cmd_vz = -(dir_z / vector_len) * DRONE_SPEED 
            send_ned_velocity(vehicle, cmd_vx, cmd_vy, cmd_vz)
        
        # 5. 3D Plot Update
        drone_xs.append(dx); drone_ys.append(dy); drone_zs.append(dz)
        target_xs.append(target_x); target_ys.append(target_y); target_zs.append(target_z)
        
        ax.clear()
        ax.set_title(f'Predictive Intercept 3D (Dist: {real_distance:.1f}m)')
        
        min_x = min(min(drone_xs), min(target_xs)) - 20
        max_x = max(max(drone_xs), max(target_xs)) + 20
        min_y = min(min(drone_ys), min(target_ys)) - 20
        max_y = max(max(drone_ys), max(target_ys)) + 20
        max_z = max(max(drone_zs), max(target_zs)) + 20
        
        ax.set_xlim([min_x, max_x])
        ax.set_ylim([min_y, max_y])
        ax.set_zlim([0, max_z if max_z > 20 else 20])
        
        ax.plot(drone_xs, drone_ys, drone_zs, color='blue', label='Fighter Drone')
        ax.plot(target_xs, target_ys, target_zs, color='red', label='Target')
        ax.scatter(dx, dy, dz, color='blue')
        ax.scatter(target_x, target_y, target_z, color='red', marker='o', s=100)
        
        # Draw Line of Sight
        ax.plot([dx, target_x + target_vx * t_intercept], 
                [dy, target_y + target_vy * t_intercept], 
                [dz, target_z + target_vz * t_intercept], color='gold', linestyle='--')
                
        ax.legend()
        plt.draw()
        plt.pause(0.05)

    print("Returning to Launch (RTL).")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
