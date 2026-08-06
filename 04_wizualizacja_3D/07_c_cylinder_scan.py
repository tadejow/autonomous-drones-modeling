# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import matplotlib
matplotlib.use('TkAgg') # Wymuszenie otwierania okienek

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import matplotlib.pyplot as plt
import time
import math

# ==========================================
# FUNKCJE MATEMATYCZNE I KINEMATYCZNE
# ==========================================

def get_location_metres(original_location, dNorth, dEast):
    """Przelicza dodane metry na nowe koordynaty GPS (przybliżenie sferyczne)."""
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def get_distance_ned(loc1, loc2):
    """Zwraca dystans w metrach (North, East) pomiędzy dwoma punktami GPS."""
    earth_radius = 6378137.0
    dlat = math.radians(loc2.lat - loc1.lat)
    dlon = math.radians(loc2.lon - loc1.lon)
    
    dNorth = dlat * earth_radius
    dEast = dlon * earth_radius * math.cos(math.radians(loc1.lat))
    return dNorth, dEast

def draw_3d_building(ax, center_n, center_e, size_n, size_e, height):
    """Rysuje trójwymiarowy blok na wykresie."""
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
# INICJALIZACJA DRONA
# ==========================================
print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

vehicle.parameters['BATT_FS_LOW_ACT'] = 0 
vehicle.parameters['BATT_FS_CRT_ACT'] = 0
vehicle.parameters['WP_SPEED'] = 700  # 7 m/s

print("Czekam na satelity (Fix GPS)...")
while not vehicle.is_armable: time.sleep(1)
baza_gps = vehicle.location.global_relative_frame

# ==========================================
# DEFINICJA CELU ZE WSPÓŁRZĘDNYCH MAVPROXY
# ==========================================
# TWOJE WSPÓŁRZĘDNE Z MARKERA:
CEL_LAT = -35.36266675
CEL_LON = 149.16579263

# Wirtualny środek naszego budynku
cel_gps = LocationGlobalRelative(CEL_LAT, CEL_LON, 0)

# Obliczamy gdzie fizycznie znajduje się budynek na osiach względem drona (dla wykresu 3D)
BUDYNEK_N, BUDYNEK_E = get_distance_ned(baza_gps, cel_gps)

# Wymiary szarego klocka na wykresie
BUDYNEK_ROZMIAR_N = 18.0
BUDYNEK_ROZMIAR_E = 14.0 
BUDYNEK_WYS = 12.0

# Parametry okręgów
PROMIEN_LOTU = 22.0
WYS_1 = 8.0   
WYS_2 = 16.0  
KROK_KATA = 30 

print(f"Generowanie misji... Cel zlokalizowany {BUDYNEK_N:.1f}m na Północ i {BUDYNEK_E:.1f}m na Wschód od bazy.")
vehicle.commands.clear()
vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, baza_gps.lat, baza_gps.lon, WYS_1))

for wysokosc in [WYS_1, WYS_2]:
    for kat_stopnie in range(0, 360 + KROK_KATA, KROK_KATA):
        kat_rad = math.radians(kat_stopnie)
        
        # OBLICZAMY PRZESUNIĘCIE WZGLĘDEM SAMEGO BUDYNKU!
        x = PROMIEN_LOTU * math.cos(kat_rad)
        y = PROMIEN_LOTU * math.sin(kat_rad)
        
        wp = get_location_metres(cel_gps, x, y)
        vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wysokosc))

vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
vehicle.commands.upload()

# ==========================================
# LOT I WIZUALIZACJA 3D
# ==========================================
print("\nUzbrajam i startuję...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed: time.sleep(1)

vehicle.simple_takeoff(WYS_1)
while vehicle.location.global_relative_frame.alt < WYS_1 * 0.95: time.sleep(1)

print("Wysokość osiągnięta! Przełączam na tryb AUTO...")
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
        ax.set_title('Inspekcja 3D - Koordynaty Satelitarne')
        
        min_n = min(min(xs), BUDYNEK_N - PROMIEN_LOTU) - 10
        max_n = max(max(xs), BUDYNEK_N + PROMIEN_LOTU) + 10
        min_e = min(min(ys), BUDYNEK_E - PROMIEN_LOTU) - 10
        max_e = max(max(ys), BUDYNEK_E + PROMIEN_LOTU) + 10
        
        ax.set_xlim([min_n, max_n])
        ax.set_ylim([min_e, max_e])
        ax.set_zlim([0, WYS_2 + 10])
        
        ax.set_xlabel('North (m)')
        ax.set_ylabel('East (m)')
        ax.set_zlabel('Altitude (m)')
        
        # RYSOWANIE
        draw_3d_building(ax, BUDYNEK_N, BUDYNEK_E, BUDYNEK_ROZMIAR_N, BUDYNEK_ROZMIAR_E, BUDYNEK_WYS)
        ax.plot(xs, ys, zs, color='blue', linewidth=2, label='Trajektoria lotu')
        ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=50, label='Dron')
        
        ax.legend()
        plt.draw()
        plt.pause(0.1)

print("\nMisja zakończona, dron wraca do bazy.")
vehicle.close()
plt.ioff()
plt.show()
