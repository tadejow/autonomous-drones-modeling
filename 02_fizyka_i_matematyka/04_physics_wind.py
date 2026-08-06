# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import time
import math
from dronekit import connect, VehicleMode

def main():
    print("Connecting to vehicle...")
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

    # -------------------------------------------------------------
    # ADVANCED: Listening to raw MAVLink physics messages
    # -------------------------------------------------------------
    # We dynamically attach custom attributes to the vehicle object 
    # to avoid using 'global' variables in Python.
    vehicle.wind_speed = 0.0
    vehicle.wind_direction = 0.0

    @vehicle.on_message('WIND')
    def wind_listener(self, name, message):
        """Callback function triggered every time a WIND MAVLink message is received."""
        self.wind_speed = message.speed
        self.wind_direction = message.direction

    # -------------------------------------------------------------

    print("\n--- PHYSICS AND WIND RESISTANCE TEST ---")
    
    print("Waiting for GPS lock and sensor calibration...")
    while not vehicle.is_armable:
        time.sleep(1)
        
    print("Arming and taking off to 15 meters...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(15.0)

    while vehicle.location.global_relative_frame.alt < 14.5:
        time.sleep(1)

    print("\nVehicle is hovering. Enable wind in MAVProxy console:")
    print(" -> param set SIM_WIND_SPD 10")
    print("Watch how the vehicle changes Pitch/Roll to maintain its GPS position!\n")

    # Telemetry loop - runs for 30 seconds
    for i in range(30):
        # 1. Read Euler Angles (Vehicle orientation in space)
        # Angles are returned in radians; converting to degrees for readability
        roll = math.degrees(vehicle.attitude.roll)
        pitch = math.degrees(vehicle.attitude.pitch)
        yaw = math.degrees(vehicle.attitude.yaw)
        
        # 2. Read physical velocity in NED frame (m/s)
        vx, vy, vz = vehicle.velocity
        total_speed = math.sqrt(vx**2 + vy**2 + vz**2)
        
        # Print physical report
        print(f"[Second {i+1}/30] VEHICLE PHYSICS REPORT:")
        print(f" -> Attitude : Pitch={pitch:5.1f}°, Roll={roll:5.1f}°, Yaw={yaw:5.1f}°")
        print(f" -> Velocity : {total_speed:.2f} m/s (Vector thrust: X={vx:.1f}, Y={vy:.1f}, Z={vz:.1f})")
        print(f" -> Est. Wind: {vehicle.wind_speed:.1f} m/s from {vehicle.wind_direction:.0f}°")
        print("-" * 65)
        
        time.sleep(1)

    print("\nExperiment concluded. Landing...")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()

if __name__ == "__main__":
    main()
