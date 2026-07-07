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

# Funkcja pomocnicza: Konwersja metrów na współrzędne GPS
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

print("Czekam na GPS...")
while not vehicle.is_armable:
    time.sleep(1)

baza_gps = vehicle.location.global_relative_frame

# ==========================================
# MATEMATYKA: Generowanie Spirali 3D
# ==========================================
MAX_WYSOKOSC = 30.0
MIN_WYSOKOSC = 5.0
MAX_PROMIEN = 20.0
OBROTY = 3
ILOSC_PUNKTOW = 50

print("Generowanie trajektorii spirali zniżkującej...")
vehicle.commands.clear()
vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, baza_gps.lat, baza_gps.lon, MAX_WYSOKOSC))

# Przechodzimy po parametrze t od 0 do 1
for i in range(ILOSC_PUNKTOW + 1):
    t = i / ILOSC_PUNKTOW
    
    # Równania parametryczne spirali stożkowej
    aktualny_promien = MAX_PROMIEN * (1.0 - t * 0.8) # Promień maleje do 20%
    aktualna_wysokosc = MAX_WYSOKOSC - t * (MAX_WYSOKOSC - MIN_WYSOKOSC)
    kat = t * OBROTY * 2 * math.pi
    
    x_north = aktualny_promien * math.cos(kat)
    y_east = aktualny_promien * math.sin(kat)
    
    wp = get_location_metres(baza_gps, x_north, y_east)
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, aktualna_wysokosc))

vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
vehicle.commands.upload()

# ==========================================
# LOT I WIZUALIZACJA 3D (Matplotlib)
# ==========================================
print("\nUzbrajam i startuję...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed: time.sleep(1)

vehicle.simple_takeoff(MAX_WYSOKOSC)
while vehicle.location.global_relative_frame.alt < MAX_WYSOKOSC * 0.95: time.sleep(1)

vehicle.mode = VehicleMode("AUTO")
print("Rozpoczęto misję AUTO. Otwieranie wykresu 3D...")

# Konfiguracja Matplotlib (Tryb Interaktywny)
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('Helisa Poszukiwawcza 3D')
ax.set_xlabel('North (m)')
ax.set_ylabel('East (m)')
ax.set_zlabel('Altitude (m)')

x_hist, y_hist, z_hist = [], [], []

while vehicle.mode.name == "AUTO":
    # Odczyt pozycji lokalnej drona w metrach względem punktu startu
    # (UWAGA: Z w układzie NED jest ujemne, dlatego dajemy minus, by wysokość rosła w górę)
    loc = vehicle.location.local_frame
    if loc.north is not None:
        x_hist.append(loc.north)
        y_hist.append(loc.east)
        z_hist.append(-loc.down)
        
        ax.clear() # Czyścimy i rysujemy na nowo
        ax.set_title('Helisa Poszukiwawcza 3D (Na żywo)')
        ax.set_xlim([-MAX_PROMIEN, MAX_PROMIEN])
        ax.set_ylim([-MAX_PROMIEN, MAX_PROMIEN])
        ax.set_zlim([0, MAX_WYSOKOSC])
        
        # Rysowanie śladu drona
        ax.plot(x_hist, y_hist, z_hist, color='blue', linewidth=2)
        # Rysowanie głowy drona (ostatni punkt)
        ax.scatter(x_hist[-1], y_hist[-1], z_hist[-1], color='red', s=50)
        
        plt.draw()
        plt.pause(0.5) # Odświeżanie co 0.5 sekundy

vehicle.close()
plt.ioff()
plt.show() # Trzyma otwarte okno na końcu
