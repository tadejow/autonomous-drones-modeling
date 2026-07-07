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
# FUNKCJE MATEMATYCZNE I KINEMATYCZNE
# ==========================================

def get_location_metres(original_location, dNorth, dEast):
    """Przelicza metry w układzie kartezjańskim na koordynaty GPS."""
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """Oblicza odległość między dwoma punktami GPS."""
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def arm_and_takeoff(target_altitude):
    print("Uzbrajanie i start...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= target_altitude * 0.95:
            break
        time.sleep(1)

# ==========================================
# GENERATORY TRAJEKTORII (RÓWNANIA PARAMETRYCZNE)
# Zwracają listę tupli (X_North, Y_East) w metrach
# t - parametr rosnący od 0 do 2*PI (lub więcej)
# ==========================================

def generate_circle(radius=15, num_points=36):
    """Okrąg: x = R*cos(t), y = R*sin(t)"""
    points = []
    for i in range(num_points):
        t = (2 * math.pi * i) / num_points
        x = radius * math.cos(t) - radius # Odejmujemy radius, by zaczynać z (0,0)
        y = radius * math.sin(t)
        points.append((x, y))
    return points

def generate_infinity(scale=15, num_points=50):
    """Znak nieskończoności (Krzywa Lissajous / Lemniskata Bernoulliego):
       x = scale * sin(t), y = scale * sin(t)*cos(t)"""
    points = []
    for i in range(num_points):
        t = (2 * math.pi * i) / num_points
        # Lemniskata Gerono (uproszczona)
        x = scale * math.sin(t)
        y = scale * math.sin(t) * math.cos(t)
        points.append((x, y))
    return points

def generate_heart(scale=1, num_points=60):
    """Kardioida / Serce:
       x = 16*sin^3(t)
       y = 13*cos(t) - 5*cos(2t) - 2*cos(3t) - cos(4t)"""
    points = []
    for i in range(num_points):
        t = (2 * math.pi * i) / num_points
        # Oryginalne równanie daje czubek serca w (0,0) u dołu. 
        # Obracamy oś Y (North/South), żeby serce było wyprostowane.
        x_raw = 16 * (math.sin(t) ** 3)
        y_raw = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
        
        # Mnożymy przez skalę i dopasowujemy orientację (North = Y, East = X)
        north = y_raw * scale
        east = x_raw * scale
        points.append((north - (12*scale), east)) # Przesunięcie, by startować płynnie
    return points

# ==========================================
# GŁÓWNA LOGIKA PROGRAMU
# ==========================================

print("=== GENERATOR TRAJEKTORII LOTU ===")
print("1. Okrąg (Kółko)")
print("2. Lemniskata (Ósemka/Nieskończoność)")
print("3. Serduszko (Parametryczne)")
wybor = input("Wybierz kształt (1/2/3): ")

if wybor == '1':
    trajektoria = generate_circle(radius=15)
    nazwa = "Okrąg"
elif wybor == '2':
    trajektoria = generate_infinity(scale=20)
    nazwa = "Ósemka"
elif wybor == '3':
    trajektoria = generate_heart(scale=1.5)
    nazwa = "Serce"
else:
    print("Nieznany wybór, domyślnie wybieram Okrąg.")
    trajektoria = generate_circle(radius=15)
    nazwa = "Okrąg"

print(f"\nŁączenie z dronem (przygotowanie do lotu: {nazwa})...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

arm_and_takeoff(15)
punkt_zero = vehicle.location.global_relative_frame

# Prędkość - im więcej punktów trajektorii i gęściej, tym wolniej lepiej lecieć
vehicle.airspeed = 8 

print(f"\nRozpoczynam rysowanie kształtu: {nazwa}")
print(f"Ilość punktów (waypointów) do przebycia: {len(trajektoria)}")

for i, (dnorth, deast) in enumerate(trajektoria):
    cel_gps = get_location_metres(punkt_zero, dnorth, deast)
    vehicle.simple_goto(cel_gps)
    
    # Tutaj pętla nadążna jest bardzo "ciasna" (tolerancja 2.0 metry).
    # Dron nie musi idealnie trafić w punkt. Jak tylko zbliży się do niego na 2 metry, 
    # skrypt natychmiast wysyła komendę leć do następnego. 
    # Dzięki temu dron "ścina zakręty" i leci super płynnie!
    while True:
        aktualna = vehicle.location.global_relative_frame
        dystans = get_distance_metres(aktualna, cel_gps)
        
        if dystans <= 2.0:
            break
        time.sleep(0.1) # Krótki czas odświeżania dla gładkości

print("\nTrajektoria ukończona! Wracam (RTL)...")
vehicle.mode = VehicleMode("RTL")
vehicle.close()
print("Koniec misji.")
