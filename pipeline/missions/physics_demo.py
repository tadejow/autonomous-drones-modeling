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

    # Automatyczne ustawienie silnego wiatru przez parametry MAVLink!
    print("Programowanie symulatora - ustawiam wiatr 15m/s z południa!")
    vehicle.parameters['SIM_WIND_SPD'] = 15.0
    vehicle.parameters['SIM_WIND_DIR'] = 180.0

    # Start na niższą wysokość, żeby było szybciej
    manager.arm_and_takeoff(7.0)

    # Inicjalizacja Plottera
    plotter = DronePlotter(title="Fizyka lotu - Walka z Wiatrem (Bujanie)", trail_length=300)
    # Odpowiednia perspektywa żeby widzieć bujanie poziome i pionowe
    plotter.set_view(elev=15, azim=45) 
    
    print("\nNagrywanie telemetrii...")
    for i in range(150): # Szybszy eksperyment, mniej klatek
        
        # Dynamiczna zmiana wiatru co kilkadziesiąt klatek
        if i == 0:
            print("[WIATR] Potężny podmuch ze Wschodu (270 st)!")
            vehicle.parameters['SIM_WIND_DIR'] = 270.0
        elif i == 50:
            print("[WIATR] Zmiana wiatru! Wieje z Zachodu (90 st)!")
            vehicle.parameters['SIM_WIND_DIR'] = 90.0
        elif i == 100:
            print("[WIATR] Porywisty boczny wiatr (135 st)!")
            vehicle.parameters['SIM_WIND_DIR'] = 135.0

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
                print(f"Attitude: P={math.degrees(pitch):5.1f} R={math.degrees(roll):5.1f} Y={math.degrees(yaw):5.1f}")
                
        time.sleep(0.1)

    # Powrót
    manager.rtl_and_close()
    plotter.close()

if __name__ == "__main__":
    run_physics_wind_demo()
