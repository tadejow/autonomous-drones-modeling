# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def clear_mission(vehicle):
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()

def generate_lawnmower_mission(vehicle, start_loc, width, height, swath_width, altitude):
    print("Generowanie optymalnej trasy skanowania...")
    cmds = vehicle.commands
    
    # 0. Wymagany przez protokół MAVLink - dodanie punktu domowego
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, start_loc.lat, start_loc.lon, altitude))

    current_east = 0
    direction = 1 

    # 1. Generowanie algorytmu zig-zag (Kosiarka)
    while current_east <= width:
        target_north = height if direction == 1 else 0
        wp_gps = get_location_metres(start_loc, target_north, current_east)
        
        # ZWRÓĆ UWAGĘ: Parametr autocontinue = 1 (zamiast 0)
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp_gps.lat, wp_gps.lon, altitude))
        
        current_east += swath_width
        direction *= -1 

    # 2. Zakończenie misji i powrót (RTL) z autocontinue=1
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))

    print("Wgrywanie misji do autopilota...")
    cmds.upload()
    print(f"Zakończono! Wgrano punktów misji: {cmds.count}")


# ==========================================
# GŁÓWNA LOGIKA PROGRAMU
# ==========================================

print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

print("Czekam na satelity (Fix GPS)...")
while not vehicle.is_armable:
    time.sleep(1)

start_location = vehicle.location.global_relative_frame

WYSOKOSC = 20         
SZEROKOSC_POLA = 80   
DLUGOSC_POLA = 100    
SZEROKOSC_SKANU = 15  

clear_mission(vehicle)
generate_lawnmower_mission(vehicle, start_location, SZEROKOSC_POLA, DLUGOSC_POLA, SZEROKOSC_SKANU, WYSOKOSC)

# Najbezpieczniejsza procedura: Startujemy w GUIDED...
print("\nUzbrajam silniki i startuję (GUIDED)...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    time.sleep(1)

vehicle.simple_takeoff(WYSOKOSC)
while True:
    if vehicle.location.global_relative_frame.alt >= WYSOKOSC * 0.95:
        print("Wysokość przelotowa osiągnięta!")
        break
    time.sleep(1)

# ... i dopiero w powietrzu oddajemy kontrolę trybowi AUTO!
print("Przełączam na tryb AUTO - Dron rozpoczyna skanowanie!")
vehicle.commands.next = 1 # Upewniamy się, że zaczyna od pierwszego właściwego punktu
vehicle.mode = VehicleMode("AUTO")

while True:
    nastepny_punkt = vehicle.commands.next
    ilosc_punktow = vehicle.commands.count
    
    print(f"[Monitoring] Dron leci do Waypointa: {nastepny_punkt} / {ilosc_punktow}")
    
    if vehicle.mode.name == "RTL":
        print("Skanowanie zakończone. Dron wraca do bazy!")
        break
    time.sleep(2)

vehicle.close()
print("Koniec nadzoru.")
