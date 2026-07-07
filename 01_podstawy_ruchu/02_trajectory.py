# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# ==========================================
# FUNKCJE POMOCNICZE (MATEMATYKA I KINEMATYKA)
# ==========================================

def get_location_metres(original_location, dNorth, dEast):
    """
    Zwraca nowy obiekt LocationGlobalRelative, przesunięty o X metrów na północ (dNorth)
    i Y metrów na wschód (dEast) względem pozycji początkowej.
    Używa przybliżenia kulistego Ziemi.
    """
    earth_radius = 6378137.0 # Promień Ziemi w metrach
    
    # Przesunięcie współrzędnych w radianach
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    
    # Nowe pozycje w stopniach dziesiętnych
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Oblicza odległość w metrach między dwoma punktami GPS za pomocą wzoru Haversine'a (uproszczonego).
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def arm_and_takeoff(target_altitude):
    """
    Standardowa procedura uzbrajania i startu (wyniesiona do funkcji, żeby kod był czystszy).
    """
    print("Inicjalizacja drona...")
    while not vehicle.is_armable:
        time.sleep(1)
        
    print("Uzbrajanie silników...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
        
    print(f"Start na {target_altitude}m...")
    vehicle.simple_takeoff(target_altitude)
    
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_altitude * 0.95:
            print("Wysokość przelotowa osiągnięta.")
            break
        time.sleep(1)

# ==========================================
# GŁÓWNA LOGIKA PROGRAMU
# ==========================================

print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# 1. Startujemy na 10 metrów
arm_and_takeoff(10)

# Pobieramy pozycję startową ("punkt zero" w układzie kartezjańskim)
punkt_startowy = vehicle.location.global_relative_frame

# 2. Definiujemy trajektorię (kwadrat o boku L metrów)
# Zapis w formacie: (przesunięcie_X_North, przesunięcie_Y_East)
L = 45
punkty_trajektorii = [
    (L, 0),   # Punkt 1: Lm na północ
    (L, L),  # Punkt 2: Lm na północ, Lm na wschód
    (0, L),   # Punkt 3: 0m na północ, Lm na wschód
    (0, 0)     # Punkt 4: Powrót nad miejsce startu
]

print("\nRozpoczynam realizację trajektorii (Kwadrat)...")

# Ustawiamy domyślną prędkość przelotową na 5 m/s
vehicle.airspeed = 5 

# 3. Pętla wykonująca lot po punktach
for i, (dnorth, deast) in enumerate(punkty_trajektorii):
    print(f"-> Lot do punktu {i+1} (North: {dnorth}m, East: {deast}m)")
    
    # Przeliczamy metry na współrzędne GPS
    cel_gps = get_location_metres(punkt_startowy, dnorth, deast)
    
    # Wysyłamy komendę lotu
    vehicle.simple_goto(cel_gps)
    
    # Pętla sprawdzająca, czy dron dotarł do celu (tolerancja 1.5 metra)
    while True:
        aktualna_pozycja = vehicle.location.global_relative_frame
        dystans_do_celu = get_distance_metres(aktualna_pozycja, cel_gps)
        
        # Opcjonalnie: print(f"   Dystans: {dystans_do_celu:.1f}m")
        
        if dystans_do_celu <= 1.5:
            print(f"   Osiągnięto punkt {i+1}!")
            time.sleep(2) # Krótki zawis na rogu kwadratu
            break
        time.sleep(0.5)

# 4. Zakończenie misji i lądowanie
print("\nTrajektoria wykonana. Powrót na ziemię (RTL)...")
vehicle.mode = VehicleMode("RTL")

# Zamykamy połączenie
vehicle.close()
print("Koniec skryptu.")
