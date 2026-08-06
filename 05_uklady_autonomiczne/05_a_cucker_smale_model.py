# --- ŁATKA DLA NOWYCH WERSJI PYTHONA ---
import collections
try:
    collections.MutableMapping = collections.abc.MutableMapping
except AttributeError:
    pass
# ---------------------------------------

import matplotlib
matplotlib.use('TkAgg')

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import matplotlib.pyplot as plt
import time
import math
import numpy as np

def send_ned_velocity(vehicle, v_x, v_y, v_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, 0, v_x, v_y, v_z, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def get_distance_ned(loc_baza, loc_cel):
    earth_radius = 6378137.0
    dlat = math.radians(loc_cel.lat - loc_baza.lat)
    dlon = math.radians(loc_cel.lon - loc_baza.lon)
    n = dlat * earth_radius
    e = dlon * earth_radius * math.cos(math.radians(loc_baza.lat))
    d = -(loc_cel.alt - loc_baza.alt)
    return np.array([n, e, d])

print("=== INICJALIZACJA CHMURY DRONÓW ===")
porty = ['udp:127.0.0.1:14550', 'udp:127.0.0.1:14560', 'udp:127.0.0.1:14570']
roje = []

for i, port in enumerate(porty):
    print(f"Łączenie z Dronem {i} na porcie {port}...")
    v = connect(port, wait_ready=True)
    v.parameters['BATT_FS_LOW_ACT'] = 0
    v.parameters['BATT_FS_CRT_ACT'] = 0
    while not v.is_armable: time.sleep(0.5)
    roje.append(v)

WYSOKOSC_LIDERA = 15.0

print(f"\nUzbrajanie roju i start...")
for v in roje:
    v.mode = VehicleMode("GUIDED")
    v.armed = True

time.sleep(2)

# Startujemy drony (Lider na 15m, skrzydłowi mogą wystartować niżej/wyżej dla testu matematyki)
roje[0].simple_takeoff(WYSOKOSC_LIDERA)
roje[1].simple_takeoff(12.0) # Skrzydłowy 1 startuje na 12m
roje[2].simple_takeoff(10.0) # Skrzydłowy 2 startuje na 10m

wszystkie_w_powietrzu = False
while not wszystkie_w_powietrzu:
    wszystkie_w_powietrzu = all(v.location.global_relative_frame.alt > 8.0 for v in roje)
    time.sleep(1)

print("Rój w powietrzu! Lider trzyma 15m, skrzydłowi dostosowują się matematycznie w 3D.")

# ==========================================
# PARAMETRY ROJU 3D
# ==========================================
K_CS = 2.0        # Cucker-Smale
BETA = 0.5        
K_COHESION = 0.3  # Przyciąganie 3D do Lidera
R_REP = 35.0      # Odpychanie 3D
D_SAFE = 5.0      # Strefa bezpieczna
V_MAX = 8.0       # Max prędkość skrzydłowych

baza_startowa_lidera = roje[0].location.global_relative_frame

czas_startu = time.time()
last_time = time.time()

plt.ion()
fig = plt.figure(figsize=(14, 6))
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122)

kolory = ['red', 'blue', 'green']
historia_x = [[], [], []]
historia_y = [[], [], []]
historia_z = [[], [], []]

t_hist = []
d01_hist, d02_hist, d12_hist = [], [], []

try:
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        if dt > 1.0 or dt == 0: dt = 0.2
        
        czas_obecny = current_time - czas_startu
        
        X = np.zeros((3, 3))
        V = np.zeros((3, 3))
        
        for i, v in enumerate(roje):
            X[i] = get_distance_ned(baza_startowa_lidera, v.location.global_relative_frame)
            vel = v.velocity
            V[i] = np.array([vel[0], vel[1], vel[2]])
            
            historia_x[i].append(X[i][0])
            historia_y[i].append(X[i][1])
            historia_z[i].append(-X[i][2])
            
        d01 = np.linalg.norm(X[0] - X[1])
        d02 = np.linalg.norm(X[0] - X[2])
        d12 = np.linalg.norm(X[1] - X[2])
        
        t_hist.append(czas_obecny)
        d01_hist.append(d01); d02_hist.append(d02); d12_hist.append(d12)
            
        # 1. LOGIKA LIDERA (Jedyne miejsce ze sztywną wysokością 15m!)
        v_leader_n = 2.5
        v_leader_e = math.sin(czas_obecny * 0.4) * 3.0 
        v_leader_d = -1.0 * (WYSOKOSC_LIDERA - (-X[0][2])) # Sztywne 15m dla Lidera
        send_ned_velocity(roje[0], v_leader_n, v_leader_e, v_leader_d)
        
        # 2. LOGIKA PODĄŻAJĄCYCH (Pełne swobodne 3D bez narzuconej wysokości)
        for i in range(1, 3):
            # A) Kohezja 3D (Ciągnie wektorem N, E, D w stronę Lidera)
            wektor_do_lidera = X[0] - X[i]
            v_cohesion = K_COHESION * wektor_do_lidera
            
            # B) Cucker-Smale 3D + Repulsja 3D
            v_cucker = np.zeros(3)
            v_repulsja = np.zeros(3)
            
            for j in range(3):
                if i == j: continue
                
                diff_pos = X[i] - X[j]
                dystans = np.linalg.norm(diff_pos)
                if dystans == 0: dystans = 0.1
                
                waga_cs = K_CS / (1.0 + (dystans / 10.0)**BETA)
                v_cucker += waga_cs * (V[j] - V[i])
                
                if dystans < D_SAFE:
                    v_repulsja += R_REP * (diff_pos / (dystans**3))
                    
            # Suma sił w 3D (Włączając oś Z!)
            v_target = V[i] + (v_cucker + v_repulsja + v_cohesion) * dt
            
            # Limit prędkości całkowitej 3D
            speed_total = np.linalg.norm(v_target)
            if speed_total > V_MAX:
                v_target = (v_target / speed_total) * V_MAX
                
            # Wysyłamy swobodny wektor 3D do skrzydłowych
            send_ned_velocity(roje[i], v_target[0], v_target[1], v_target[2])
            
        # 3. RYSOWANIE
        ax1.clear()
        ax2.clear()
        
        all_x = [x for h in historia_x for x in h]
        all_y = [y for h in historia_y for y in h]
        all_z = [z for h in historia_z for z in h]
        
        ax1.set_title(f'Swobodny Rój 3D (Czas: {czas_obecny:.1f}s)')
        ax1.set_xlim([min(all_x) - 10, max(all_x) + 10])
        ax1.set_ylim([min(all_y) - 10, max(all_y) + 10])
        ax1.set_zlim([0, max(max(all_z) + 5, 20)])
        
        ax1.set_xlabel('North (m)')
        ax1.set_ylabel('East (m)')
        ax1.set_zlabel('Altitude (m)')
        
        for i in range(3):
            rola = "Lider (15m)" if i == 0 else f"Skrzydłowy {i}"
            ax1.plot(historia_x[i], historia_y[i], historia_z[i], color=kolory[i], label=rola)
            ax1.scatter(historia_x[i][-1], historia_y[i][-1], historia_z[i][-1], color=kolory[i], s=80)
            
        ax1.legend(loc='upper left')
        
        ax2.set_title('Odległości w przestrzeni 3D (m)')
        ax2.plot(t_hist, d01_hist, color='purple', label='Dron 0 - Dron 1')
        ax2.plot(t_hist, d02_hist, color='orange', label='Dron 0 - Dron 2')
        ax2.plot(t_hist, d12_hist, color='cyan', label='Dron 1 - Dron 2')
        ax2.axhline(y=D_SAFE, color='black', linestyle='--', label=f'Strefa bezp. ({D_SAFE}m)')
        
        ax2.set_xlabel('Czas (s)')
        ax2.set_ylabel('Odległość (m)')
        max_dyst = max(max(d01_hist), max(d02_hist), max(d12_hist))
        ax2.set_ylim([0, max(max_dyst + 5, 15)])
        ax2.grid(True)
        ax2.legend(loc='upper right')
        
        plt.draw()
        plt.pause(0.05)

except KeyboardInterrupt:
    print("\nZakończono symulację. Wracam.")
    for v in roje:
        v.mode = VehicleMode("RTL")
        v.close()
    plt.ioff(); plt.show()
