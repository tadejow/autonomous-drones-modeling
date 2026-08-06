# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import time
from dronekit import connect, VehicleMode

def main():
    # 1. Connect to the SITL simulator
    connection_string = 'udp:127.0.0.1:14550'
    print(f"Connecting to vehicle on: {connection_string}")
    vehicle = connect(connection_string, wait_ready=True)
    print("Successfully connected!\n")

    # 2. Wait for the autopilot to initialize (e.g., acquire GPS lock)
    print("Waiting for vehicle to initialize (GPS lock)...")
    while not vehicle.is_armable:
        time.sleep(1)
    print("Vehicle is ready to arm!")

    # 3. Set mode to GUIDED
    print("Setting mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")

    # 4. Arm the motors
    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Motors armed! (Caution: propellers spinning)")

    # 5. Takeoff procedure
    target_altitude = 10.0
    print(f"Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    # Monitoring the altitude
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f" Current altitude: {current_altitude:.1f} m")
        
        # Break the loop if we reach 95% of target altitude
        if current_altitude >= target_altitude * 0.95:
            print("Target altitude reached!")
            break
        time.sleep(1)

    # 6. Hover and return
    print("Hovering for 5 seconds...")
    time.sleep(5)

    print("Returning to Launch (RTL mode)...")
    vehicle.mode = VehicleMode("RTL")

    # Close connection
    vehicle.close()
    print("Mission completed. Vehicle connection closed.")

if __name__ == "__main__":
    main()
