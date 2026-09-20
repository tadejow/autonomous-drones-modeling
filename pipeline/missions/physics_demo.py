# --- PYTHON 3.14 COMPATIBILITY PATCH ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import time
import math
from pipeline.core.vehicle_manager import VehicleManager
from pipeline.visualization.plotter import DronePlotter
import numpy as np

def run_physics_wind_demo(connection_string='udp:127.0.0.1:14550'):
    print("Inicjalizacja Managera...")
    manager = VehicleManager(connection_string)
    vehicle = manager.get_vehicle()

    # Odbieranie wektora wiatru
    vehicle.wind_speed = 0.0
    vehicle.wind_direction = 0.0

    @vehicle.on_message('WIND')
    def wind_listener(self, name, message):
        self.wind_speed = message.speed
        self.wind_direction = message.direction

    manager.arm_and_takeoff(15.0)

    print("\n[INFO] Wpisz w konsoli MAVProxy aby wywołać wiatr:")
    print(" -> param set SIM_WIND_SPD 10")
    
    # Inicjalizacja Plottera
    plotter = DronePlotter(title="Fizyka lotu - Walka z Wiatrem (Bujanie)", trail_length=300)
    # Odpowiednia perspektywa żeby widzieć bujanie poziome i pionowe
    plotter.set_view(elev=15, azim=45) 
    
    print("\nNagrywanie telemetrii przez 30 sekund...")
    for i in range(300): # 30 sekund w pętli co 0.1s
        loc = vehicle.location.local_frame
        if loc.north is not None:
            # Pozycja XYZ
            dx, dy, dz = loc.north, loc.east, -loc.down
            
            # Wektory przechyłu
            pitch = vehicle.attitude.pitch
            roll = vehicle.attitude.roll
            yaw = vehicle.attitude.yaw
            
            plotter.update((dx, dy, dz), attitude=(pitch, roll, yaw))

            # Możemy wydrukować co sekundę stan
            if i % 10 == 0:
                print(f"[{i//10}/30s] Attitude: P={math.degrees(pitch):5.1f} R={math.degrees(roll):5.1f} Y={math.degrees(yaw):5.1f} | Wind: {vehicle.wind_speed:.1f}m/s")
                
        time.sleep(0.1)

    manager.rtl_and_close()
    plotter.close()

if __name__ == "__main__":
    run_physics_wind_demo()
