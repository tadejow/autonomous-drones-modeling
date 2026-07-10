# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import matplotlib
matplotlib.use('TkAgg') # Wymuszenie okienek w Linuksie

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

print("Wyłączam Failsafe baterii...")
vehicle.parameters['BATT_FS_LOW_ACT'] = 0 
vehicle.parameters['BATT_FS_CRT_ACT'] = 0
vehicle.parameters['WP_SPD'] = 1000  # 10 m/s

print("Czekam na satelity (Fix GPS)...")
while not vehicle.is_armable: time.sleep(1)

baza_gps = vehicle.location.global_relative_frame

# ==========================================
# DEFINICJA OBSZARU Z TWOJEGO RYSUNKU
# Wierzchołki (North, East) w metrach względem bazy (0,0)
# ==========================================
wielokat = [
    (120, -40),  # Lewy górny róg
    (110, 100),  # Prawy górny róg (lekko ścięty)
    (-30, 110),  # Prawy dolny róg
    (-40, -30)   # Lewy dolny róg
]

WYSOKOSC_LOTU = 25.0
SZEROKOSC_PASKA = 15.0

print("Obliczanie przecięć geometrii (Algorytm Sweep-Line)...")
cmds = vehicle.commands
cmds.clear()
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, baza_gps.lat, baza_gps.lon, WYSOKOSC_LOTU))
# Nie mozna przechodzic w autopilot na ziemi
# cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, 0, 0, WYSOKOSC_LOTU))

# 1. Znalezienie Bounding Box (zakres osi North)
min_n = min(p[0] for p in wielokat)
max_n = max(p[0] for p in wielokat)

current_n = min_n + (SZEROKOSC_PASKA / 2.0)
direction = 1 

# 2. Skanowanie obszaru od dołu do góry
while current_n <= max_n:
    przeciecia_east = []
    
    # Sprawdzamy przecięcie linii 'current_n' z każdym bokiem wielokąta
    for i in range(len(wielokat)):
        p1 = wielokat[i]
        p2 = wielokat[(i + 1) % len(wielokat)] # Następny wierzchołek (zawija do początku)
        
        n1, e1 = p1
        n2, e2 = p2
        
        # Sprawdzenie czy nasza pozioma linia przecina ten odcinek
        if (n1 <= current_n < n2) or (n2 <= current_n < n1):
            # Równanie prostej: E = E1 + (E2 - E1) * (N - N1) / (N2 - N1)
            e_intersect = e1 + (e2 - e1) * (current_n - n1) / (n2 - n1)
            przeciecia_east.append(e_intersect)
            
    # Jeśli znaleźliśmy przecięcia (powinny być dwa dla prostego wielokąta)
    if len(przeciecia_east) >= 2:
        przeciecia_east.sort()
        
        # Wyznaczamy lewy i prawy kraniec obszaru na tej wysokości
        left_e = min(przeciecia_east)
        right_e = max(przeciecia_east)
        
        # Zależnie od kierunku, wlatujemy w obszar z lewej do prawej, albo z prawej do lewej
        if direction == 1:
            wp1 = get_location_metres(baza_gps, current_n, left_e)
            wp2 = get_location_metres(baza_gps, current_n, right_e)
        else:
            wp1 = get_location_metres(baza_gps, current_n, right_e)
            wp2 = get_location_metres(baza_gps, current_n, left_e)
            
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp1.lat, wp1.lon, WYSOKOSC_LOTU))
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp2.lat, wp2.lon, WYSOKOSC_LOTU))
        
        direction *= -1 # Zmiana kierunku na następny pasek
        
    current_n += SZEROKOSC_PASKA

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))
cmds.upload()
print("Misja obliczona i wgrana!")

# ==========================================
# LOT I WIZUALIZACJA 3D
# ==========================================
print("\nUzbrajam i startuję w trybie GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed: time.sleep(1)

# Samodzielny start przed misją AUTO
vehicle.simple_takeoff(WYSOKOSC_LOTU)
while vehicle.location.global_relative_frame.alt < WYSOKOSC_LOTU * 0.95: 
    time.sleep(1)

print("Wysokość osiągnięta! Przełączam na tryb AUTO...")
vehicle.commands.next = 1 # 1 to teraz nasz pierwszy właściwy waypoint z kosiarki
vehicle.mode = VehicleMode("AUTO")

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_hist, y_hist, z_hist = [], [], []

# Rozdzielenie wierzchołków wielokąta do rysowania
poly_n = [p[0] for p in wielokat] + [wielokat[0][0]]
poly_e = [p[1] for p in wielokat] + [wielokat[0][1]]
poly_z = [0] * len(poly_n) # Rysujemy wielokąt płasko na ziemi (Z=0)

while vehicle.mode.name == "AUTO":
    loc = vehicle.location.local_frame
    if loc.north is not None:
        x_hist.append(loc.north)
        y_hist.append(loc.east)
        z_hist.append(-loc.down)
        
        ax.clear()
        ax.set_title('Skanowanie Nieregularnego Obszaru 3D')
        
        ax.set_xlim([min_n - 20, max_n + 20])
        ax.set_ylim([min(poly_e) - 20, max(poly_e) + 20])
        ax.set_zlim([0, WYSOKOSC_LOTU + 10])
        
        ax.set_xlabel('North (m)')
        ax.set_ylabel('East (m)')
        ax.set_zlabel('Altitude (m)')
        
        # Rysowanie śladu drona
        ax.plot(x_hist, y_hist, z_hist, color='orange', linewidth=2, label='Trajektoria lotu')
        ax.scatter(x_hist[-1], y_hist[-1], z_hist[-1], color='red', s=50, label='Dron')
        
        # Rysowanie obrysu Twojego czarnego markera!
        ax.plot(poly_n, poly_e, poly_z, color='black', linewidth=3, linestyle='-', label='Granice (Twój marker)')
        
        ax.legend()
        plt.draw()
        plt.pause(0.1)

print("\nMisja zakończona, dron wraca (RTL).")
vehicle.close()
plt.ioff()
plt.show()
