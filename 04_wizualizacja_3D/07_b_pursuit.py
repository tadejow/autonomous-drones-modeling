# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import matplotlib
matplotlib.use('TkAgg') # Wymuszenie otwierania okienek

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import matplotlib.pyplot as plt
import time
import math

# ==========================================
# FUNKCJE KINEMATYCZNE I SIECIOWE
# ==========================================

def get_location_metres(original_location, dNorth, dEast):
    """Przelicza metry na GPS."""
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """Wysyła wektor prędkości (m/s) do drona."""
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 
        0, 0, 0, 
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def spoof_adsb_target(vehicle, ufo_gps):
    """Wysyła fałszywy sygnał transpondera ADS-B."""
    flags = 1 | 2 | 4 | 8 | 16 | 32
    # Używamy nazwanych argumentów (kwargs), aby wersja Pymavlinka nie miała znaczenia
    msg = vehicle.message_factory.adsb_vehicle_encode(
        ICAO_address=12345, 
        lat=int(ufo_gps.lat * 1e7), 
        lon=int(ufo_gps.lon * 1e7), 
        altitude_type=0, 
        altitude=int(ufo_gps.alt * 1000), 
        heading=0, 
        hor_velocity=0, 
        ver_velocity=0, 
        callsign=b"UFO-1", 
        emitter_type=10, 
        tslc=1, 
        flags=flags,
        squawk=1200
    )
    vehicle.send_mavlink(msg)

# ==========================================
# INICJALIZACJA DRONA
# ==========================================

print("Łączenie z dronem...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
while not vehicle.is_armable: 
    time.sleep(1)

baza_gps = vehicle.location.global_relative_frame

print("Start na 10m...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed: time.sleep(1)
vehicle.simple_takeoff(10)
while vehicle.location.global_relative_frame.alt < 9.5: time.sleep(1)

# ==========================================
# MATEMATYKA: Wirtualny Cel i Naprowadzanie
# ==========================================
# Cel (UFO) zaczyna z pozycji 100m na wschód, 150m na północ, wysokość 30m
ufo_x, ufo_y, ufo_z = 150.0, 100.0, 30.0
# UFO przemieszcza się powoli w stronę wschodnią i lekko opada
ufo_vx, ufo_vy, ufo_vz = 0.25, 1.0, -0.2 

# Konfiguracja Wykresu 3D
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

drone_xs, drone_ys, drone_zs = [], [], []
ufo_xs, ufo_ys, ufo_zs = [], [], []

PREDKOSC_DRONA = 15.0 # m/s

print("Rozpoczęto pościg wyprzedzający! Śledź mapę MAVProxy oraz okno 3D.")

# Zmienna do śledzenia RZECZYWISTEGO czasu
last_time = time.time()

while True:
    # 1. RZECZYWISTY KROK CZASU (Zabezpiecza przed "zagięciem czasu")
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    if dt > 1.0 or dt == 0: dt = 0.1 
    
    # 2. Odczyt pozycji drona
    loc = vehicle.location.local_frame
    if loc.north is None: continue
    dx, dy, dz = loc.north, loc.east, -loc.down
    
    # 3. Aktualizacja pozycji UFO (Teraz mnożymy przez rzeczywisty czas 'dt')
    ufo_x += ufo_vx * dt
    ufo_y += ufo_vy * dt
    ufo_z += ufo_vz * dt
    
    # Wysyłanie sygnału na radar
    ufo_gps = get_location_metres(baza_gps, ufo_x, ufo_y)
    ufo_gps.alt = ufo_z
    spoof_adsb_target(vehicle, ufo_gps)
    
    # 4. === MATEMATYCZNY ALGORYTM WYPRZEDZANIA (Predictive Intercept) ===
    dist_x = ufo_x - dx
    dist_y = ufo_y - dy
    dist_z = ufo_z - dz
    
    v_target_sq = ufo_vx**2 + ufo_vy**2 + ufo_vz**2
    v_drone_sq = PREDKOSC_DRONA**2
    
    # Współczynniki równania kwadratowego
    a = v_drone_sq - v_target_sq
    b = 2.0 * (dist_x * ufo_vx + dist_y * ufo_vy + dist_z * ufo_vz)
    c = -(dist_x**2 + dist_y**2 + dist_z**2)
    
    discriminant = b**2 - 4.0 * a * c
    t_intercept = 0.0
    
    if discriminant >= 0 and a > 0:
        t_intercept = (b + math.sqrt(discriminant)) / (2.0 * a)
        if t_intercept < 0: t_intercept = 0.0
    
    # Wektor wyprzedzający (Gdzie cel BĘDZIE, a nie gdzie JEST)
    dir_x = (ufo_x + ufo_vx * t_intercept) - dx
    dir_y = (ufo_y + ufo_vy * t_intercept) - dy
    dir_z = (ufo_z + ufo_vz * t_intercept) - dz
    
    real_distance = math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)
    
    if real_distance < 4.0:
        print("CEL ZESTRZELONY (Przechwycono wektorowo)!")
        send_ned_velocity(vehicle, 0, 0, 0)
        break
        
    vector_len = math.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
    if vector_len > 0:
        cmd_vx = (dir_x / vector_len) * PREDKOSC_DRONA
        cmd_vy = (dir_y / vector_len) * PREDKOSC_DRONA
        cmd_vz = -(dir_z / vector_len) * PREDKOSC_DRONA 
        send_ned_velocity(vehicle, cmd_vx, cmd_vy, cmd_vz)
    
    # 5. Aktualizacja Wykresu 3D
    drone_xs.append(dx); drone_ys.append(dy); drone_zs.append(dz)
    ufo_xs.append(ufo_x); ufo_ys.append(ufo_y); ufo_zs.append(ufo_z)
    
    ax.clear()
    ax.set_title(f'Pościg Wyprzedzający 3D (Dystans: {real_distance:.1f}m)')
    
    min_x = min(min(drone_xs), min(ufo_xs)) - 20
    max_x = max(max(drone_xs), max(ufo_xs)) + 20
    min_y = min(min(drone_ys), min(ufo_ys)) - 20
    max_y = max(max(drone_ys), max(ufo_ys)) + 20
    max_z = max(max(drone_zs), max(ufo_zs)) + 20
    
    ax.set_xlim([min_x, max_x])
    ax.set_ylim([min_y, max_y])
    ax.set_zlim([0, max_z if max_z > 20 else 20])
    
    ax.plot(drone_xs, drone_ys, drone_zs, color='blue', label='Myśliwiec')
    ax.plot(ufo_xs, ufo_ys, ufo_zs, color='red', label='Cel')
    ax.scatter(dx, dy, dz, color='blue')
    ax.scatter(ufo_x, ufo_y, ufo_z, color='red', marker='o', s=100)
    
    # Rysujemy celownik wyprzedzający
    ax.plot([dx, ufo_x + ufo_vx * t_intercept], 
            [dy, ufo_y + ufo_vy * t_intercept], 
            [dz, ufo_z + ufo_vz * t_intercept], color='gold', linestyle='--')
            
    ax.legend()
    plt.draw()
    plt.pause(0.05)

print("Wracam do bazy (RTL).")
vehicle.mode = VehicleMode("RTL")
vehicle.close()
plt.ioff()
plt.show()
