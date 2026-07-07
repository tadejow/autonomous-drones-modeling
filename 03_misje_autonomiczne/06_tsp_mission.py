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
import random

# ==========================================
# FUNKCJE POMOCNICZE I MATEMATYCZNE
# ==========================================

def get_location_metres(original_location, dNorth, dEast):
    """Przelicza metry w układzie kartezjańskim na koordynaty GPS."""
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def distance_2d(p1, p2):
    """Oblicza dystans euklidesowy między dwoma punktami (x, y) w metrach."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def clear_mission(vehicle):
    """Czyści obecną misję w pamięci drona."""
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()

# ==========================================
# ALGORYTMY: GENEROWANIE I TSP
# ==========================================

def generate_random_points(num_points, max_north, max_east):
    """Generuje listę N losowych punktów (North, East) w granicach obszaru."""
    points = []
    for _ in range(num_points):
        n = random.uniform(-max_north, max_north) # Obszar wokół drona
        e = random.uniform(-max_east, max_east)
        points.append((n, e))
    return points

def solve_tsp_nearest_neighbor(start_point, points):
    """
    Rozwiązuje Problem Komiwojażera algorytmem zachłannym (Najbliższy Sąsiad).
    Zwraca posortowaną listę punktów.
    """
    print("\nRozwiązywanie problemu TSP (Nearest Neighbor)...")
    unvisited = points.copy()
    current_point = start_point # Zaczynamy z punktu (0,0) - Baza
    tour = []
    
    total_distance = 0.0

    while unvisited:
        # Szukamy punktu, do którego jest najkrótsza droga
        nearest = min(unvisited, key=lambda p: distance_2d(current_point, p))
        dist = distance_2d(current_point, nearest)
        
        # Przenosimy punkt do odwiedzonych
        unvisited.remove(nearest)
        tour.append(nearest)
        
        total_distance += dist
        current_point = nearest

    # Na koniec wracamy do bazy (0,0)
    total_distance += distance_2d(current_point, start_point)
    
    print(f"Obliczona trasa ma długość ok. {total_distance:.1f} metrów.")
    return tour

# ==========================================
# GŁÓWNA LOGIKA PROGRAMU
# ==========================================

print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

print("Czekam na satelity (Fix GPS)...")
while not vehicle.is_armable:
    time.sleep(1)

baza_gps = vehicle.location.global_relative_frame
WYSOKOSC = 15

# 1. Definiujemy problem
baza_karo = (0.0, 0.0)
ILOSC_PACZEK = 8
ZASIEG_OPERACYJNY = 100 # Promień 100 metrów

print(f"Generowanie {ILOSC_PACZEK} losowych punktów dostaw w promieniu {ZASIEG_OPERACYJNY}m...")
punkty_dostaw = generate_random_points(ILOSC_PACZEK, ZASIEG_OPERACYJNY, ZASIEG_OPERACYJNY)

# 2. Rozwiązujemy TSP
zoptymalizowana_trasa = solve_tsp_nearest_neighbor(baza_karo, punkty_dostaw)

# 3. Tworzymy misję dla Drona
print("Wgrywanie misji do autopilota...")
clear_mission(vehicle)
cmds = vehicle.commands

# Wymagany punkt domowy
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, baza_gps.lat, baza_gps.lon, WYSOKOSC))

# Dodajemy wyliczoną trasę jako Waypointy
for dNorth, dEast in zoptymalizowana_trasa:
    wp_gps = get_location_metres(baza_gps, dNorth, dEast)
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp_gps.lat, wp_gps.lon, WYSOKOSC))

# Powrót do bazy (RTL)
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1, 0, 0, 0, 0, 0, 0, 0))

cmds.upload()
print(f"Wgrano punktów misji: {cmds.count}")

# 4. Start (GUIDED) -> Lot misyjny (AUTO)
print("\nUzbrajam silniki i startuję...")
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

print("Przełączam na tryb AUTO - Dron wykonuje trasę Komiwojażera!")
vehicle.commands.next = 1
vehicle.mode = VehicleMode("AUTO")

while True:
    nastepny_punkt = vehicle.commands.next
    ilosc_punktow = vehicle.commands.count
    
    # Wyświetlamy ładny pasek postępu (odejmujemy 1 za punkt domowy i RTL)
    akt = nastepny_punkt - 1
    max_pkt = ilosc_punktow - 2
    if akt > max_pkt: akt = max_pkt
    
    print(f"[Monitoring] Dostawa: {akt}/{max_pkt}")
    
    if vehicle.mode.name == "RTL":
        print("Wszystkie punkty zaliczone. Dron wraca do bazy!")
        break
    time.sleep(3)

vehicle.close()
print("Koniec symulacji.")
