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
# FUNKCJE POMOCNICZE
# ==========================================
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def draw_3d_building(ax, center_n, center_e, size_n, size_e, height):
    """Rysuje trójwymiarowy blok (budynek) na wykresie matplotlib."""
    # Definicja 8 wierzchołków prostopadłościanu
    n1, n2 = center_n - size_n/2, center_n + size_n/2
    e1, e2 = center_e - size_e/2, center_e + size_e/2
    
    # Podłoga i sufit
    X = [n1, n2, n2, n1, n1]
    Y = [e1, e1, e2, e2, e1]
    Z_bottom = [0, 0, 0, 0, 0]
    Z_top = [height, height, height, height, height]
    
    # Rysowanie krawędzi
    ax.plot(X, Y, Z_bottom, color='gray') # podłoga
    ax.plot(X, Y, Z_top, color='gray')    # sufit
    for i in range(4): # Pionowe ściany
        ax.plot([X[i], X[i]], [Y[i], Y[i]], [0, height], color='gray')

# ==========================================
# KONFIGURACJA DRONA I MISJI
# ==========================================
print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

print("Konfiguracja parametrów...")
vehicle.parameters['BATT_FS_LOW_ACT'] = 0 
vehicle.parameters['BATT_FS_CRT_ACT'] = 0
vehicle.parameters['WP_SPEED'] = 700  # Zwalniamy do 7 m/s dla lepszych "zdjęć"

while not vehicle.is_armable: 
    time.sleep(1)

baza_gps = vehicle.location.global_relative_frame

# Parametry białego budynku (względem punktu startu)
BUDYNEK_N = 25.0
BUDYNEK_E = 45.0
BUDYNEK_ROZMIAR_N = 10.0
BUDYNEK_ROZMIAR_E = 20.0
BUDYNEK_WYS = 12.0

# Parametry lotu
PROMIEN_LOTU = 20.0
WYS_1 = 8.0   # Dolny okrąg
WYS_2 = 16.0  # Górny okrąg
KROK_KATA = 30 # Co ile stopni stawiamy waypoint

print(f"Generowanie dwupoziomowej orbity wokół budynku...")
vehicle.commands.clear()
# Punkt domowy
vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, baza_gps.lat, baza_gps.lon, WYS_1))

# Generowanie dwóch okręgów na dwóch wysokościach
for wysokosc in [WYS_1, WYS_2]:
    # 360 + KROK_KATA domyka okrąg w pełni
    for kat_stopnie in range(0, 360 + KROK_KATA, KROK_KATA):
        kat_rad = math.radians(kat_stopnie)
        
        # Współrzędne na okręgu
        x = BUDYNEK_N + PROMIEN_LOTU * math.cos(kat_rad)
        y = BUDYNEK_E + PROMIEN_LOTU * math.sin(kat_rad)
        
        wp = get_location_metres(baza_gps, x, y)
        vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wysokosc))

# Zakończenie RTL
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
while vehicle.location.global_relative_frame.alt < WYS_1 * 0.95: 
    time.sleep(1)

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
        ax.set_title('Inspekcja 3D Budynku')
        
        # Dynamiczne śledzenie z zachowaniem proporcji okręgu
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
        
        # 1. RYSOWANIE BUDYNKU
        draw_3d_building(ax, BUDYNEK_N, BUDYNEK_E, BUDYNEK_ROZMIAR_N, BUDYNEK_ROZMIAR_E, BUDYNEK_WYS)
        
        # 2. RYSOWANIE TRAJEKTORII DRONA
        ax.plot(xs, ys, zs, color='blue', linewidth=2, label='Trajektoria lotu')
        ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=50, label='Dron')
        
        ax.legend()
        plt.draw()
        plt.pause(0.1)

print("\nMisja zakończona, dron wraca do bazy.")
vehicle.close()
plt.ioff()
plt.show()
