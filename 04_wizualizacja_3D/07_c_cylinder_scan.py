# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt
import time
import math

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
while not vehicle.is_armable: time.sleep(1)

baza_gps = vehicle.location.global_relative_frame

# ==========================================
# MATEMATYKA: Siatka Walcowa (Cylindrical)
# ==========================================
# Parametry komina (środek w odległości 20m na północ od nas)
SRODEK_N = 20.0
SRODEK_E = 0.0
PROMIEN_LOTU = 10.0
WYS_MIN = 5.0
WYS_MAX = 30.0
KROK_KATA = 30 # stopni (co ile przesuwamy się wokół komina)

print("Generowanie siatki wokół walca 3D...")
vehicle.commands.clear()
vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, baza_gps.lat, baza_gps.lon, WYS_MIN))

aktualna_wysokosc = WYS_MIN
kierunek_wys = 1 # 1 = w górę, -1 = w dół

for kat_stopnie in range(0, 360, KROK_KATA):
    kat_rad = math.radians(kat_stopnie)
    
    # 1. Pozycja u podstawy / szczytu przed przesunięciem
    x = SRODEK_N + PROMIEN_LOTU * math.cos(kat_rad)
    y = SRODEK_E + PROMIEN_LOTU * math.sin(kat_rad)
    wp = get_location_metres(baza_gps, x, y)
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, aktualna_wysokosc))
    
    # 2. Zmiana wysokości (lot pionowy wzdłuż komina)
    aktualna_wysokosc = WYS_MAX if kierunek_wys == 1 else WYS_MIN
    kierunek_wys *= -1
    
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, aktualna_wysokosc))

vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
vehicle.commands.upload()

# ==========================================
# LOT I WIZUALIZACJA 3D
# ==========================================
print("\nUzbrajam i startuję...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed: time.sleep(1)
vehicle.simple_takeoff(WYS_MIN)
while vehicle.location.global_relative_frame.alt < WYS_MIN * 0.95: time.sleep(1)

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
        ax.set_title('Inspekcja Komina 3D')
        
        # Wymuszamy proporcje osi, aby walec wyglądał okrągło
        ax.set_xlim([-10, 30]); ax.set_ylim([-20, 20]); ax.set_zlim([0, 35])
        
        ax.plot(xs, ys, zs, color='orange', linewidth=2)
        ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=50)
        
        plt.draw()
        plt.pause(0.5)

vehicle.close()
plt.ioff(); plt.show()
